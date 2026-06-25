#include <chrono>
#include <cmath>
#include <initializer_list>
#include <sstream>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "moveit_node/linear_motion_node.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Point makePoint(const double x, const double y, const double z)
{
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::msg::Quaternion makeQuaternionFromFixedXyz(
    const double rx,
    const double ry,
    const double rz)
{
    const Eigen::Quaterniond quaternion =
        Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());

    geometry_msgs::msg::Quaternion orientation;
    orientation.x = quaternion.x();
    orientation.y = quaternion.y();
    orientation.z = quaternion.z();
    orientation.w = quaternion.w();
    return orientation;
}

geometry_msgs::msg::Pose makePose(
    const double x,
    const double y,
    const double z,
    const double rx,
    const double ry,
    const double rz)
{
    geometry_msgs::msg::Pose pose;
    pose.position = makePoint(x, y, z);
    pose.orientation = makeQuaternionFromFixedXyz(rx, ry, rz);
    return pose;
}

double degreesToRadians(const double degree)
{
    return degree * kPi / 180.0;
}

std::vector<double> degreesToRadians(std::initializer_list<double> degrees)
{
    std::vector<double> radians;
    radians.reserve(degrees.size());
    for (const double degree : degrees) {
        radians.push_back(degree * kPi / 180.0);
    }
    return radians;
}

std::vector<double> getStartupJointTarget(const std::size_t joint_count)
{
    if (joint_count == 6) {
        return degreesToRadians({-2.036, 27.355, 71.288, 63.129, -0.131, 0.002});
    }

    if (joint_count == 7) {
        return degreesToRadians({-97.21, 33.403, -21.55, 74.313, 16.54, 65.03, 0.0});
    }

    return {};
}

std::vector<double> getStartupJointTarget(
    const std::string & planning_group,
    const std::size_t joint_count)
{
    (void)planning_group;
    return getStartupJointTarget(joint_count);
}

std::vector<double> getReturnJointTarget(const std::size_t joint_count)
{
    return std::vector<double>(joint_count, 0.0);
}

std::vector<double> parsePoseValues(const std::string & raw_pose_text)
{
    std::string normalized_text = raw_pose_text;
    for (char & ch : normalized_text) {
        if (ch == ',' || ch == ';' || ch == '[' || ch == ']') {
            ch = ' ';
        }
    }

    std::istringstream input(normalized_text);
    std::vector<double> values;
    double value = 0.0;
    while (input >> value) {
        values.push_back(value);
    }
    return values;
}

std::vector<geometry_msgs::msg::Point> getArcPoints(const std::size_t joint_count)
{
    if (joint_count == 6) {
        return {
            makePoint(0.517, 0.02, 0.368),
            makePoint(0.458, 0.24, 0.36),
            makePoint(0.27, 0.438, 0.368),
        };
    }

    if (joint_count == 7) {
        return {
            makePoint(-0.117, -0.346, 0.239),
            makePoint(0.232, -0.225, 0.295),
            makePoint(0.274, 0.122, 0.304),
        };
    }

    return {};
}

std::vector<geometry_msgs::msg::Point> getArcPoints(
    const std::string & planning_group,
    const std::size_t joint_count)
{
    if (joint_count == 7) {
        if (planning_group == "left_arm") {
            return {
                makePoint(-0.117, 0.346, 0.239),
                makePoint(0.232, 0.225, 0.295),
                makePoint(0.274, -0.122, 0.304),
            };
        }

        if (planning_group == "right_arm") {
            return {
                makePoint(-0.117, -0.346, 0.239),
                makePoint(0.232, -0.225, 0.295),
                makePoint(0.274, 0.122, 0.304),
            };
        }
    }

    return getArcPoints(joint_count);
}

}  // namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<LinearMotionNode>(options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]() { executor.spin(); });

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    if (!node->init()) {
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    std::vector<double> joint_status;
    if (!node->getCurrentJointStatus(joint_status)) {
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    const std::size_t joint_count = joint_status.size();
    const std::string planning_group = node->get_parameter("planning_group").as_string();
    RCLCPP_INFO(node->get_logger(), "当前 joint_status 中检测到 %zu 个活动关节", joint_count);
    RCLCPP_INFO(node->get_logger(), "当前 planning_group: %s", planning_group.c_str());
    const bool prefer_named_start = node->get_parameter("prefer_named_start").as_bool();
    const bool enable_cartesian_demo = node->get_parameter("enable_cartesian_demo").as_bool();
    const bool enable_pose_target = node->get_parameter("enable_pose_target").as_bool();
    const std::string home_named_target =
        node->get_parameter("home_named_target").as_string();

    if (enable_pose_target) {
        const std::string pose_target_csv = node->get_parameter("pose_target_csv").as_string();
        const auto pose_values = parsePoseValues(pose_target_csv);
        if (pose_values.size() != 6) {
            RCLCPP_ERROR(
                node->get_logger(),
                "enable_pose_target=true 时 pose_target_csv 必须包含 6 个值，当前输入为: %s",
                pose_target_csv.c_str());
            rclcpp::shutdown();
            spinner.join();
            return 1;
        }

        double x = pose_values[0];
        double y = pose_values[1];
        double z = pose_values[2];
        double rx = pose_values[3];
        double ry = pose_values[4];
        double rz = pose_values[5];

        if (node->get_parameter("pose_target_position_in_mm").as_bool()) {
            x *= 0.001;
            y *= 0.001;
            z *= 0.001;
        }

        if (node->get_parameter("pose_target_rpy_in_degrees").as_bool()) {
            rx = degreesToRadians(rx);
            ry = degreesToRadians(ry);
            rz = degreesToRadians(rz);
        }

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            !std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz))
        {
            RCLCPP_ERROR(node->get_logger(), "pose_target_csv 中存在非法数值，取消执行");
            rclcpp::shutdown();
            spinner.join();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "=== 运动到绝对位姿目标 ===");
        RCLCPP_INFO(
            node->get_logger(),
            "原始输入 pose_target_csv=[%s], 位置单位=%s, 姿态单位=%s, 目标参考系=%s",
            pose_target_csv.c_str(),
            node->get_parameter("pose_target_position_in_mm").as_bool() ? "mm" : "m",
            node->get_parameter("pose_target_rpy_in_degrees").as_bool() ? "deg" : "rad",
            node->get_parameter("pose_reference_frame").as_string().empty() ?
            "<planning_frame>" : node->get_parameter("pose_reference_frame").as_string().c_str());

        if (!node->moveToPoseTarget(makePose(x, y, z, rx, ry, rz), "pose_target")) {
            rclcpp::shutdown();
            spinner.join();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "位姿目标执行完成，关闭节点");
        rclcpp::shutdown();
        spinner.join();
        return 0;
    }

    std::vector<double> startup_joint_target;
    if (!prefer_named_start) {
        startup_joint_target = getStartupJointTarget(planning_group, joint_count);
    }

    if (!startup_joint_target.empty()) {
        RCLCPP_INFO(node->get_logger(), "根据 joint_status 自动选择 %zu 轴启动关节目标", joint_count);
        if (!node->moveToJointTarget(startup_joint_target, "startup_joint_target")) {
            rclcpp::shutdown();
            spinner.join();
            return 1;
        }
    } else {
        if (!home_named_target.empty()) {
            RCLCPP_WARN(
                node->get_logger(),
                "未使用关节硬编码启动位姿，改用命名位姿 %s",
                home_named_target.c_str());
            if (!node->moveToNamedTarget(home_named_target, "home_named_target")) {
                rclcpp::shutdown();
                spinner.join();
                return 1;
            }
        } else {
            RCLCPP_WARN(
                node->get_logger(),
                "既没有匹配的启动关节目标，也没有提供 home_named_target，将从当前位置继续");
        }
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    const auto arc_points = getArcPoints(planning_group, joint_count);
    if (!enable_cartesian_demo) {
        RCLCPP_WARN(node->get_logger(), "enable_cartesian_demo=false，跳过圆弧与直线示教段");
    } else if (arc_points.empty()) {
        RCLCPP_WARN(
            node->get_logger(),
            "未为 %zu 个关节的规划组配置圆弧示教点，跳过笛卡尔示教段",
            joint_count);
    } else {
        RCLCPP_INFO(node->get_logger(), "=== 基于三点拟合圆弧 ===");
        if (!node->moveCartesianArcByPoints(arc_points)) {
            rclcpp::shutdown();
            spinner.join();
            return 1;
        }

        rclcpp::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(node->get_logger(), "=== 运动1 ===");
        if (!node->moveCartesianLine(0.0, 0.0, -0.1)) {
            rclcpp::shutdown();
            spinner.join();
            return 1;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(node->get_logger(), "=== 运动2 ===");
        if (!node->moveCartesianLine(0.1, 0.0, 0.0)) {
            rclcpp::shutdown();
            spinner.join();
            return 1;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(node->get_logger(), "=== 运动3 ===");
        if (!node->moveCartesianLine(-0.2, 0.1, 0.0)) {
            rclcpp::shutdown();
            spinner.join();
            return 1;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(node->get_logger(), "=== 回到 初始 位姿 ===");
    const auto return_joint_target = getReturnJointTarget(joint_count);

    if (!node->moveToJointTarget(return_joint_target, "return_joint_target")) {
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "全部完成，关闭节点");
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
