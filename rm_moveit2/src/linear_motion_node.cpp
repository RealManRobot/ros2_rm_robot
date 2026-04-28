#include "moveit_node/linear_motion_node.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace
{

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose)
{
    const Eigen::Quaterniond quaternion(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = quaternion.normalized().toRotationMatrix();
    transform.translation() = Eigen::Vector3d(
        pose.position.x,
        pose.position.y,
        pose.position.z);
    return transform;
}

geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d & transform)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();

    const Eigen::Quaterniond quaternion(transform.linear());
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    return pose;
}

geometry_msgs::msg::Pose makeIdentityPose()
{
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    return pose;
}

bool isIdentityPoseOffset(const geometry_msgs::msg::Pose & pose)
{
    constexpr double kTolerance = 1e-9;
    return std::abs(pose.position.x) < kTolerance &&
        std::abs(pose.position.y) < kTolerance &&
        std::abs(pose.position.z) < kTolerance &&
        std::abs(pose.orientation.x) < kTolerance &&
        std::abs(pose.orientation.y) < kTolerance &&
        std::abs(pose.orientation.z) < kTolerance &&
        std::abs(pose.orientation.w - 1.0) < kTolerance;
}

std::vector<double> parseCsvDoubles(const std::string & raw_text)
{
    std::string normalized = raw_text;
    for (char & ch : normalized) {
        if (ch == ',' || ch == ';' || ch == '[' || ch == ']') {
            ch = ' ';
        }
    }

    std::istringstream input(normalized);
    std::vector<double> values;
    double value = 0.0;
    while (input >> value) {
        values.push_back(value);
    }
    return values;
}

}  // namespace

// TODO 添加至ros2_rm_robot文件中 提测前完成 rm_moveit
LinearMotionNode::LinearMotionNode(const rclcpp::NodeOptions & options)
: Node("linear_motion_node", options)
{
    declare_parameter_if_needed("planning_group", "rm_group");
    declare_parameter_if_needed("velocity_scaling", 0.3);
    declare_parameter_if_needed("acceleration_scaling", 0.3);
    declare_parameter_if_needed("planning_time", 5.0);
    declare_parameter_if_needed("current_state_wait_sec", 10.0);
    declare_parameter_if_needed("home_named_target", "forward");
    declare_parameter_if_needed("prefer_named_start", false);
    declare_parameter_if_needed("enable_cartesian_demo", true);
    declare_parameter_if_needed("enable_pose_target", false);
    declare_parameter_if_needed("pose_target_csv", std::string(""));
    declare_parameter_if_needed("pose_target_position_in_mm", true);
    declare_parameter_if_needed("pose_target_rpy_in_degrees", false);
    declare_parameter_if_needed("pose_reference_frame", std::string(""));
    declare_parameter_if_needed("pose_target_is_tcp", false);
    declare_parameter_if_needed("tcp_offset_xyz", std::string("0,0,0"));
    declare_parameter_if_needed("tcp_offset_rpy", std::string("0,0,0"));
    declare_parameter_if_needed("tcp_offset_position_in_mm", false);
    declare_parameter_if_needed("tcp_offset_rpy_in_degrees", false);

    RCLCPP_INFO(this->get_logger(), "直线运动节点已启动");
}

bool LinearMotionNode::init()
{
    const std::string planning_group = this->get_parameter("planning_group").as_string();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group);
    move_group_->setMaxVelocityScalingFactor(
        this->get_parameter("velocity_scaling").as_double());
    move_group_->setMaxAccelerationScalingFactor(
        this->get_parameter("acceleration_scaling").as_double());
    move_group_->setPlanningTime(
        this->get_parameter("planning_time").as_double());

    if (!move_group_->startStateMonitor(2.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start MoveIt state monitor");
        return false;
    }

    if (!refreshCurrentState()) {
        return false;
    }

    joint_names_ = move_group_->getActiveJoints();
    if (joint_names_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "规划组 %s 未找到活动关节", planning_group.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "规划组: %s", planning_group.c_str());
    RCLCPP_INFO(this->get_logger(), "参考坐标系: %s",
                move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "末端执行器: %s",
                move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "活动关节(%zu): %s",
                joint_names_.size(), joinStrings(joint_names_).c_str());
    logPlanningAlgorithm("init");
    return true;
}

bool LinearMotionNode::getCurrentPose(geometry_msgs::msg::Pose & pose)
{
    return getCurrentPoseInFrame(move_group_->getPlanningFrame(), pose);
}

bool LinearMotionNode::getCurrentJointStatus(std::vector<double> & joint_values)
{
    if (!refreshCurrentState()) {
        return false;
    }

    joint_values = move_group_->getCurrentJointValues();
    if (joint_values.size() != joint_names_.size()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "当前 joint_status 数量异常: 期望 %zu 个, 实际 %zu 个。活动关节: %s",
            joint_names_.size(),
            joint_values.size(),
            joinStrings(joint_names_).c_str());
        return false;
    }

    return true;
}

bool LinearMotionNode::moveCartesianArcByPoints(
    const std::vector<geometry_msgs::msg::Point> & points,
    double step_size)
{
    const int point_count = static_cast<int>(points.size());
    if (point_count < 3 || point_count > 10) {
        RCLCPP_ERROR(this->get_logger(), "输入的点数必须在 3 到 10 之间！");
        return false;
    }

    Eigen::MatrixXd point_matrix(3, point_count);
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int i = 0; i < point_count; ++i) {
        point_matrix(0, i) = points[i].x;
        point_matrix(1, i) = points[i].y;
        point_matrix(2, i) = points[i].z;
        centroid += point_matrix.col(i);
    }
    centroid /= point_count;

    Eigen::MatrixXd centered_points = point_matrix.colwise() - centroid;
    Eigen::Matrix3d covariance = centered_points * centered_points.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
    Eigen::Vector3d pca_normal = eigen_solver.eigenvectors().col(0);

    Eigen::Vector3d sequential_normal = Eigen::Vector3d::Zero();
    for (int i = 0; i < point_count - 2; ++i) {
        const Eigen::Vector3d v1 = point_matrix.col(i + 1) - point_matrix.col(i);
        const Eigen::Vector3d v2 = point_matrix.col(i + 2) - point_matrix.col(i + 1);
        sequential_normal += v1.cross(v2);
    }

    if (sequential_normal.norm() < 1e-6) {
        RCLCPP_ERROR(this->get_logger(), "点太少或几乎共线，无法确定唯一的圆弧平面！");
        return false;
    }

    if (pca_normal.dot(sequential_normal) < 0) {
        pca_normal = -pca_normal;
    }

    Eigen::Vector3d u = centered_points.col(0);
    if (u.norm() < 1e-4) {
        u = centered_points.col(1);
    }
    u = (u - u.dot(pca_normal) * pca_normal).normalized();
    const Eigen::Vector3d v = pca_normal.cross(u).normalized();

    Eigen::VectorXd x(point_count);
    Eigen::VectorXd y(point_count);
    for (int i = 0; i < point_count; ++i) {
        const Eigen::Vector3d point_centered = centered_points.col(i);
        x(i) = point_centered.dot(u);
        y(i) = point_centered.dot(v);
    }

    Eigen::MatrixXd a(point_count, 3);
    Eigen::VectorXd b(point_count);
    for (int i = 0; i < point_count; ++i) {
        a(i, 0) = x(i);
        a(i, 1) = y(i);
        a(i, 2) = 1.0;
        b(i) = -(x(i) * x(i) + y(i) * y(i));
    }

    const Eigen::Vector3d coefficients = a.colPivHouseholderQr().solve(b);
    const double c_x = -coefficients(0) / 2.0;
    const double c_y = -coefficients(1) / 2.0;
    const double radius = std::sqrt(c_x * c_x + c_y * c_y - coefficients(2));
    const Eigen::Vector3d center_3d = centroid + c_x * u + c_y * v;

    const double theta_start = std::atan2(y(0) - c_y, x(0) - c_x);
    double total_sweep = 0.0;
    double current_theta = theta_start;

    for (int i = 1; i < point_count; ++i) {
        const double next_theta = std::atan2(y(i) - c_y, x(i) - c_x);
        double delta = next_theta - current_theta;

        while (delta < 0) {
            delta += 2.0 * M_PI;
        }
        if (delta > 1.9 * M_PI) {
            delta -= 2.0 * M_PI;
        }

        total_sweep += delta;
        current_theta = next_theta;
    }

    RCLCPP_INFO(this->get_logger(), "拟合成功！半径: %.3f 米, 圆心角: %.1f 度",
                radius, total_sweep * 180.0 / M_PI);

    geometry_msgs::msg::Pose start_pose;
    if (!getCurrentPose(start_pose)) {
        return false;
    }

    const double arc_length = radius * std::abs(total_sweep);
    const int num_steps = std::max(2, static_cast<int>(arc_length / step_size));

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(num_steps + 1);

    for (int i = 0; i <= num_steps; ++i) {
        const double fraction = static_cast<double>(i) / num_steps;
        const double theta_i = theta_start + fraction * total_sweep;

        const Eigen::Vector3d point_i = center_3d + radius * std::cos(theta_i) * u +
            radius * std::sin(theta_i) * v;

        geometry_msgs::msg::Pose waypoint = start_pose;
        waypoint.position.x = point_i.x();
        waypoint.position.y = point_i.y();
        waypoint.position.z = point_i.z();
        waypoints.push_back(waypoint);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    constexpr double kEefStep = 0.01;
    constexpr double kJumpThreshold = 0.0;

    logPlanningAlgorithm("moveCartesianArcByPoints");
    const double fraction = move_group_->computeCartesianPath(
        waypoints, kEefStep, kJumpThreshold, trajectory);

    RCLCPP_INFO(this->get_logger(), "笛卡尔圆弧路径完成度: %.1f%%", fraction * 100.0);

    if (fraction < 0.9) {
        RCLCPP_WARN(this->get_logger(), "路径规划不完整，取消执行。可能超出工作空间或遇到奇异点");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool LinearMotionNode::moveCartesianLine(double dx, double dy, double dz, double step_size)
{
    geometry_msgs::msg::Pose start_pose;
    if (!getCurrentPose(start_pose)) {
        RCLCPP_ERROR(this->get_logger(), "无法获取当前末端位姿，取消直线运动");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "当前位置: [%.3f, %.3f, %.3f]",
                start_pose.position.x,
                start_pose.position.y,
                start_pose.position.z);

    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const int num_steps = std::max(2, static_cast<int>(distance / step_size));

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(num_steps + 1);

    for (int i = 0; i <= num_steps; ++i) {
        const double t = static_cast<double>(i) / num_steps;
        geometry_msgs::msg::Pose waypoint = start_pose;
        waypoint.position.x += dx * t;
        waypoint.position.y += dy * t;
        waypoint.position.z += dz * t;
        waypoints.push_back(waypoint);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    constexpr double kEefStep = 0.01;
    constexpr double kJumpThreshold = 0.0;

    logPlanningAlgorithm("moveCartesianLine");
    RCLCPP_WARN(
        this->get_logger(),
        "当前直线运动调用的是 MoveGroupInterface::computeCartesianPath，属于笛卡尔插值，不走常规全局采样规划");

    const double fraction = move_group_->computeCartesianPath(
        waypoints, kEefStep, kJumpThreshold, trajectory);

    RCLCPP_INFO(this->get_logger(), "笛卡尔路径完成度: %.1f%%", fraction * 100.0);

    if (fraction < 0.9) {
        RCLCPP_WARN(this->get_logger(), "路径规划不完整，取消执行。当前姿态下目标位移可能过大，建议减小位移或改用 forward 起始位姿");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    const auto result = move_group_->execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "直线运动执行成功!");
        return true;
    }

    RCLCPP_ERROR(this->get_logger(), "执行失败");
    return false;
}

bool LinearMotionNode::moveToPoseTarget(
    const geometry_msgs::msg::Pose & pose,
    const std::string & context)
{
    const std::string configured_reference_frame =
        this->get_parameter("pose_reference_frame").as_string();
    const std::string target_reference_frame = configured_reference_frame.empty() ?
        move_group_->getPlanningFrame() : configured_reference_frame;
    return moveToPoseTargetImpl(pose, target_reference_frame, context);
}

bool LinearMotionNode::moveToPoseTarget(
    const geometry_msgs::msg::PoseStamped & pose_stamped,
    const std::string & context)
{
    const std::string target_reference_frame = pose_stamped.header.frame_id.empty() ?
        move_group_->getPlanningFrame() : pose_stamped.header.frame_id;
    return moveToPoseTargetImpl(pose_stamped.pose, target_reference_frame, context);
}

bool LinearMotionNode::moveToPoseTargetImpl(
    const geometry_msgs::msg::Pose & pose,
    const std::string & target_reference_frame,
    const std::string & context)
{
    const bool pose_target_is_tcp = this->get_parameter("pose_target_is_tcp").as_bool();
    const std::string end_effector_link = move_group_->getEndEffectorLink();

    auto tcp_offset_tool0_to_tcp = makeIdentityPose();
    const std::vector<double> tcp_offset_xyz =
        parseCsvDoubles(this->get_parameter("tcp_offset_xyz").as_string());
    const std::vector<double> tcp_offset_rpy =
        parseCsvDoubles(this->get_parameter("tcp_offset_rpy").as_string());
    if (tcp_offset_xyz.size() != 3 || tcp_offset_rpy.size() != 3) {
        RCLCPP_ERROR(
            this->get_logger(),
            "[%s] TCP 固定外参格式错误: tcp_offset_xyz 需要 3 个值, tcp_offset_rpy 需要 3 个值",
            context.c_str());
        return false;
    }

    double tcp_x = tcp_offset_xyz[0];
    double tcp_y = tcp_offset_xyz[1];
    double tcp_z = tcp_offset_xyz[2];
    double tcp_rx = tcp_offset_rpy[0];
    double tcp_ry = tcp_offset_rpy[1];
    double tcp_rz = tcp_offset_rpy[2];

    if (this->get_parameter("tcp_offset_position_in_mm").as_bool()) {
        tcp_x *= 0.001;
        tcp_y *= 0.001;
        tcp_z *= 0.001;
    }

    if (this->get_parameter("tcp_offset_rpy_in_degrees").as_bool()) {
        constexpr double kPi = 3.14159265358979323846;
        tcp_rx *= kPi / 180.0;
        tcp_ry *= kPi / 180.0;
        tcp_rz *= kPi / 180.0;
    }

    tcp_offset_tool0_to_tcp.position.x = tcp_x;
    tcp_offset_tool0_to_tcp.position.y = tcp_y;
    tcp_offset_tool0_to_tcp.position.z = tcp_z;
    const Eigen::Quaterniond tcp_quaternion =
        Eigen::AngleAxisd(tcp_rz, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(tcp_ry, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(tcp_rx, Eigen::Vector3d::UnitX());
    tcp_offset_tool0_to_tcp.orientation.x = tcp_quaternion.x();
    tcp_offset_tool0_to_tcp.orientation.y = tcp_quaternion.y();
    tcp_offset_tool0_to_tcp.orientation.z = tcp_quaternion.z();
    tcp_offset_tool0_to_tcp.orientation.w = tcp_quaternion.w();
    const bool has_tcp_offset = !isIdentityPoseOffset(tcp_offset_tool0_to_tcp);

    if (pose_target_is_tcp && !has_tcp_offset) {
        RCLCPP_WARN(
            this->get_logger(),
            "[%s] pose_target_is_tcp=true 但 TCP 外参仍为单位变换，将把输入 TCP 目标直接视为 %s 目标",
            context.c_str(),
            end_effector_link.c_str());
    }

    logPoseDiagnostics(context, target_reference_frame, tcp_offset_tool0_to_tcp, has_tcp_offset);

    geometry_msgs::msg::Pose resolved_pose;
    if (!resolvePoseForLinkTarget(
            pose,
            target_reference_frame,
            end_effector_link,
            tcp_offset_tool0_to_tcp,
            pose_target_is_tcp,
            resolved_pose))
    {
        return false;
    }

    if (!refreshCurrentState()) {
        return false;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "[%s] 位姿目标参考系=%s, planning frame=%s",
        context.c_str(),
        target_reference_frame.c_str(),
        move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(
        this->get_logger(),
        "[%s] 位姿目标 position=[%.4f, %.4f, %.4f], orientation=[%.4f, %.4f, %.4f, %.4f]",
        context.c_str(),
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    RCLCPP_INFO(
        this->get_logger(),
        "[%s] 当前目标按 %s 求解，目标 link=%s",
        context.c_str(),
        pose_target_is_tcp ? "TCP 输入位姿" : "末端 link 输入位姿",
        end_effector_link.c_str());
    if (pose_target_is_tcp) {
        RCLCPP_INFO(
            this->get_logger(),
            "[%s] 等效 %s 位姿目标(%s): %s",
            context.c_str(),
            end_effector_link.c_str(),
            target_reference_frame.c_str(),
            formatPose(resolved_pose).c_str());
    }

    logPlanningAlgorithm(context);
    move_group_->clearPoseTargets();
    move_group_->setPoseReferenceFrame(target_reference_frame);
    move_group_->setPoseTarget(resolved_pose);

    const auto result = move_group_->move();
    move_group_->clearPoseTargets();
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "[%s] 位姿运动执行成功", context.c_str());
        return true;
    }

    RCLCPP_ERROR(this->get_logger(), "[%s] 位姿运动执行失败", context.c_str());
    return false;
}

bool LinearMotionNode::moveToNamedTarget(const std::string & target_name, const std::string & context)
{
    if (!refreshCurrentState()) {
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "[%s] 使用命名位姿: %s",
                context.c_str(), target_name.c_str());
    logPlanningAlgorithm(context);
    if (!move_group_->setNamedTarget(target_name)) {
        RCLCPP_ERROR(this->get_logger(), "[%s] 无法设置命名位姿: %s",
                     context.c_str(), target_name.c_str());
        return false;
    }

    const auto result = move_group_->move();
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "[%s] 命名位姿执行成功", context.c_str());
        return true;
    }

    RCLCPP_ERROR(this->get_logger(), "[%s] 命名位姿执行失败", context.c_str());
    return false;
}

bool LinearMotionNode::moveToJointTarget(
    const std::vector<double> & joint_values,
    const std::string & context)
{
    if (!refreshCurrentState()) {
        return false;
    }

    if (joint_values.size() != joint_names_.size()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "[%s] 关节目标数量不匹配: 期望 %zu 个, 实际 %zu 个。活动关节: %s",
            context.c_str(),
            joint_names_.size(),
            joint_values.size(),
            joinStrings(joint_names_).c_str());
        return false;
    }

    std::map<std::string, double> target;
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        target[joint_names_[i]] = joint_values[i];
    }

    RCLCPP_INFO(this->get_logger(), "[%s] 关节目标: %s",
                context.c_str(), joinDoubles(joint_values).c_str());
    logPlanningAlgorithm(context);
    move_group_->setJointValueTarget(target);

    const auto result = move_group_->move();
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "[%s] 关节运动执行成功", context.c_str());
        return true;
    }

    RCLCPP_ERROR(this->get_logger(), "[%s] 关节运动执行失败", context.c_str());
    return false;
}

std::size_t LinearMotionNode::getJointCount() const
{
    return joint_names_.size();
}

bool LinearMotionNode::refreshCurrentState()
{
    if (!move_group_) {
        RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 尚未初始化");
        return false;
    }

    const double current_state_wait_sec =
        this->get_parameter("current_state_wait_sec").as_double();
    const auto current_state = move_group_->getCurrentState(current_state_wait_sec);
    if (!current_state) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to fetch current robot state after %.1f seconds",
            current_state_wait_sec);
        return false;
    }

    move_group_->setStartState(*current_state);
    return true;
}

bool LinearMotionNode::getCurrentState(moveit::core::RobotStatePtr & current_state)
{
    if (!move_group_) {
        RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 尚未初始化");
        return false;
    }

    const double current_state_wait_sec =
        this->get_parameter("current_state_wait_sec").as_double();
    current_state = move_group_->getCurrentState(current_state_wait_sec);
    if (!current_state) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to fetch current robot state after %.1f seconds",
            current_state_wait_sec);
        return false;
    }

    return true;
}

bool LinearMotionNode::getCurrentPoseInFrame(
    const std::string & reference_frame,
    geometry_msgs::msg::Pose & pose)
{
    if (!refreshCurrentState()) {
        return false;
    }

    const std::string resolved_reference_frame =
        reference_frame.empty() ? move_group_->getPlanningFrame() : reference_frame;
    moveit::core::RobotStatePtr current_state;
    if (!getCurrentState(current_state)) {
        return false;
    }

    const std::string end_effector_link = move_group_->getEndEffectorLink();
    if (!current_state->knowsFrameTransform(end_effector_link)) {
        RCLCPP_ERROR(this->get_logger(), "当前状态缺少末端 link 变换: %s", end_effector_link.c_str());
        return false;
    }
    if (!current_state->knowsFrameTransform(resolved_reference_frame)) {
        RCLCPP_ERROR(
            this->get_logger(),
            "当前状态缺少参考坐标系变换: %s",
            resolved_reference_frame.c_str());
        return false;
    }

    const Eigen::Isometry3d reference_to_world =
        current_state->getGlobalLinkTransform(resolved_reference_frame);
    const Eigen::Isometry3d tool0_to_world =
        current_state->getGlobalLinkTransform(end_effector_link);
    pose = isometryToPose(reference_to_world.inverse() * tool0_to_world);
    return true;
}

bool LinearMotionNode::resolvePoseForLinkTarget(
    const geometry_msgs::msg::Pose & input_pose,
    const std::string & reference_frame,
    const std::string & target_link,
    const geometry_msgs::msg::Pose & tcp_offset_tool0_to_tcp,
    bool input_is_tcp,
    geometry_msgs::msg::Pose & resolved_pose)
{
    (void)target_link;

    if (!input_is_tcp) {
        resolved_pose = input_pose;
        return true;
    }

    const std::string resolved_reference_frame =
        reference_frame.empty() ? move_group_->getPlanningFrame() : reference_frame;
    if (resolved_reference_frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "无法解析目标参考系");
        return false;
    }

    const Eigen::Isometry3d reference_to_tcp_target = poseToIsometry(input_pose);
    const Eigen::Isometry3d tool0_to_tcp = poseToIsometry(tcp_offset_tool0_to_tcp);
    const Eigen::Isometry3d reference_to_tool0_target =
        reference_to_tcp_target * tool0_to_tcp.inverse();
    resolved_pose = isometryToPose(reference_to_tool0_target);
    return true;
}

void LinearMotionNode::logPoseDiagnostics(
    const std::string & context,
    const std::string & reference_frame,
    const geometry_msgs::msg::Pose & tcp_offset_tool0_to_tcp,
    bool has_tcp_offset)
{
    geometry_msgs::msg::Pose current_pose_planning;
    const std::string end_effector_link = move_group_->getEndEffectorLink();
    if (getCurrentPose(current_pose_planning)) {
        RCLCPP_INFO(
            this->get_logger(),
            "[%s] 当前 %s 位姿(%s): %s",
            context.c_str(),
            end_effector_link.c_str(),
            move_group_->getPlanningFrame().c_str(),
            formatPose(current_pose_planning).c_str());
    }

    geometry_msgs::msg::Pose current_pose_reference;
    if (getCurrentPoseInFrame(reference_frame, current_pose_reference)) {
        RCLCPP_INFO(
            this->get_logger(),
            "[%s] 当前 %s 位姿(%s): %s",
            context.c_str(),
            end_effector_link.c_str(),
            reference_frame.c_str(),
            formatPose(current_pose_reference).c_str());

        if (has_tcp_offset) {
            const Eigen::Isometry3d reference_to_tcp =
                poseToIsometry(current_pose_reference) * poseToIsometry(tcp_offset_tool0_to_tcp);
            RCLCPP_INFO(
                this->get_logger(),
                "[%s] 当前 TCP 位姿(%s): %s",
                context.c_str(),
                reference_frame.c_str(),
                formatPose(isometryToPose(reference_to_tcp)).c_str());
        }
    }
}

void LinearMotionNode::logPlanningAlgorithm(const std::string & context) const
{
    const std::string pipeline_id = move_group_->getPlanningPipelineId().empty() ?
        "default" : move_group_->getPlanningPipelineId();
    const std::string planner_id = move_group_->getPlannerId().empty() ?
        "default" : move_group_->getPlannerId();
    RCLCPP_WARN(
        this->get_logger(),
        "[%s] planning pipeline: %s, planner id: %s",
        context.c_str(),
        pipeline_id.c_str(),
        planner_id.c_str());
}

std::string LinearMotionNode::joinStrings(const std::vector<std::string> & values)
{
    std::ostringstream oss;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            oss << ", ";
        }
        oss << values[i];
    }
    return oss.str();
}

std::string LinearMotionNode::joinDoubles(const std::vector<double> & values)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            oss << ", ";
        }
        oss << values[i];
    }
    return oss.str();
}

std::string LinearMotionNode::formatPose(const geometry_msgs::msg::Pose & pose)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "position=[" << pose.position.x << ", "
        << pose.position.y << ", "
        << pose.position.z << "], orientation=["
        << pose.orientation.x << ", "
        << pose.orientation.y << ", "
        << pose.orientation.z << ", "
        << pose.orientation.w << "]";
    return oss.str();
}
