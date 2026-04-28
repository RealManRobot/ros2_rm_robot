#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

class LinearMotionNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造直线/圆弧运动节点并声明所需参数。
     *
     * @param options ROS2 节点选项，用于接收 launch 注入的参数配置。
     */
    explicit LinearMotionNode(const rclcpp::NodeOptions & options);

    /**
     * @brief 初始化 MoveGroupInterface 并读取当前规划组信息。
     *
     * @return 初始化成功返回 true，否则返回 false。
     */
    bool init();

    /**
     * @brief 获取当前末端执行器位姿。
     *
     * @param pose 输出参数，用于返回当前末端位姿。
     * @return 获取成功返回 true，否则返回 false。
     */
    bool getCurrentPose(geometry_msgs::msg::Pose & pose);

    /**
     * @brief 获取当前关节状态中的活动关节值。
     *
     * @param joint_values 输出参数，用于返回当前规划组活动关节的关节值。
     * @return 获取成功返回 true，否则返回 false。
     */
    bool getCurrentJointStatus(std::vector<double> & joint_values);

    /**
     * @brief 根据输入的空间点拟合圆弧，并执行笛卡尔圆弧运动。
     *
     * @param points 圆弧示教点集合，数量要求在 3 到 10 个之间。
     * @param step_size 笛卡尔轨迹插补步长，单位为米。
     * @return 运动执行成功返回 true，否则返回 false。
     */
    bool moveCartesianArcByPoints(
        const std::vector<geometry_msgs::msg::Point> & points,
        double step_size = 0.01);

    /**
     * @brief 执行末端笛卡尔直线运动。
     *
     * @param dx X 方向位移，单位为米。
     * @param dy Y 方向位移，单位为米。
     * @param dz Z 方向位移，单位为米。
     * @param step_size 笛卡尔轨迹插补步长，单位为米。
     * @return 运动执行成功返回 true，否则返回 false。
     */
    bool moveCartesianLine(double dx, double dy, double dz, double step_size = 0.01);

    /**
     * @brief 按照末端绝对位姿目标执行空间运动。
     *
     * @param pose 目标末端位姿，位置单位为米，姿态使用四元数表示。
     * @param context 当前调用场景名称，用于日志打印。
     * @return 运动执行成功返回 true，否则返回 false。
     */
    bool moveToPoseTarget(const geometry_msgs::msg::Pose & pose, const std::string & context);

    /**
     * @brief 按照带参考系的末端绝对位姿目标执行空间运动。
     *
     * @param pose_stamped 目标末端位姿，header.frame_id 指定参考系。
     * @param context 当前调用场景名称，用于日志打印。
     * @return 运动执行成功返回 true，否则返回 false。
     */
    bool moveToPoseTarget(
        const geometry_msgs::msg::PoseStamped & pose_stamped,
        const std::string & context);

    /**
     * @brief 运动到 MoveIt SRDF 中定义的命名位姿。
     *
     * @param target_name 命名位姿名称，例如 forward 或 zero。
     * @param context 当前调用场景名称，用于日志打印。
     * @return 运动执行成功返回 true，否则返回 false。
     */
    bool moveToNamedTarget(const std::string & target_name, const std::string & context);

    /**
     * @brief 按照关节目标数组执行关节空间运动。
     *
     * @param joint_values 目标关节值数组，长度需与当前规划组活动关节数一致。
     * @param context 当前调用场景名称，用于日志打印。
     * @return 运动执行成功返回 true，否则返回 false。
     */
    bool moveToJointTarget(const std::vector<double> & joint_values, const std::string & context);

    /**
     * @brief 获取当前规划组的活动关节数量。
     *
     * @return 当前活动关节数量。
     */
    std::size_t getJointCount() const;

private:
    /**
     * @brief 仅在参数未声明时声明参数，避免重复声明。
     *
     * @tparam T 参数默认值类型。
     * @param name 参数名。
     * @param default_value 参数默认值。
     */
    template <typename T>
    void declare_parameter_if_needed(const std::string & name, const T & default_value)
    {
        if (!this->has_parameter(name)) {
            this->declare_parameter(name, default_value);
        }
    }

    /**
     * @brief 刷新当前机器人状态并同步为规划起始状态。
     *
     * @return 刷新成功返回 true，否则返回 false。
     */
    bool refreshCurrentState();

    /**
     * @brief 获取当前规划组末端在指定参考系下的位姿。
     *
     * @param reference_frame 目标参考系，留空时使用 planning frame。
     * @param pose 输出参数，用于返回当前位姿。
     * @return 获取成功返回 true，否则返回 false。
     */
    bool getCurrentPoseInFrame(
        const std::string & reference_frame,
        geometry_msgs::msg::Pose & pose);

    /**
     * @brief 读取当前状态对象，供链路变换与诊断日志复用。
     *
     * @param current_state 输出参数，用于返回当前状态。
     * @return 获取成功返回 true，否则返回 false。
     */
    bool getCurrentState(moveit::core::RobotStatePtr & current_state);

    /**
     * @brief 将输入位姿从参考系换算为某个 link 的位姿目标。
     *
     * @param input_pose 输入位姿。
     * @param reference_frame 输入位姿所在参考系。
     * @param target_link 目标 link 名称。
     * @param tcp_offset_tool0_to_tcp right_tool0 到 TCP 的固定偏移。
     * @param input_is_tcp 输入位姿是否表示 TCP。
     * @param resolved_pose 输出参数，返回 target_link 的等效位姿目标。
     * @return 换算成功返回 true，否则返回 false。
     */
    bool resolvePoseForLinkTarget(
        const geometry_msgs::msg::Pose & input_pose,
        const std::string & reference_frame,
        const std::string & target_link,
        const geometry_msgs::msg::Pose & tcp_offset_tool0_to_tcp,
        bool input_is_tcp,
        geometry_msgs::msg::Pose & resolved_pose);

    /**
     * @brief 位姿目标执行内部实现，可指定本次调用的参考系。
     *
     * @param pose 输入位姿。
     * @param reference_frame 本次目标位姿所在参考系。
     * @param context 当前调用场景名称，用于日志打印。
     * @return 执行成功返回 true，否则返回 false。
     */
    bool moveToPoseTargetImpl(
        const geometry_msgs::msg::Pose & pose,
        const std::string & reference_frame,
        const std::string & context);

    /**
     * @brief 打印当前末端及 TCP 诊断日志。
     *
     * @param context 当前调用场景名称，用于日志打印。
     * @param reference_frame 目标参考系。
     * @param tcp_offset_tool0_to_tcp right_tool0 到 TCP 的固定偏移。
     * @param has_tcp_offset 是否配置了 TCP 偏移。
     */
    void logPoseDiagnostics(
        const std::string & context,
        const std::string & reference_frame,
        const geometry_msgs::msg::Pose & tcp_offset_tool0_to_tcp,
        bool has_tcp_offset);

    /**
     * @brief 打印当前规划管线和规划器信息。
     *
     * @param context 当前调用场景名称，用于日志打印。
     */
    void logPlanningAlgorithm(const std::string & context) const;

    /**
     * @brief 将字符串数组拼接为日志可读文本。
     *
     * @param values 待拼接的字符串数组。
     * @return 拼接后的字符串。
     */
    static std::string joinStrings(const std::vector<std::string> & values);

    /**
     * @brief 将浮点数组拼接为日志可读文本。
     *
     * @param values 待拼接的浮点数组。
     * @return 拼接后的字符串。
     */
    static std::string joinDoubles(const std::vector<double> & values);

    /**
     * @brief 将位姿格式化为日志文本。
     *
     * @param pose 待打印位姿。
     * @return 格式化后的字符串。
     */
    static std::string formatPose(const geometry_msgs::msg::Pose & pose);

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::vector<std::string> joint_names_;
};
