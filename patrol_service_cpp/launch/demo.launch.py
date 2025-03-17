import launch
import launch_ros


def generate_launch_description():
    """节点定义函数，函数名固定不可改变"""
    # 声明一个launch参数
    action_declare_arg_background_r = launch.actions.DeclareLaunchArgument(
        # 参数名
        "launch_arg_bg_r",
        # 默认值
        default_value="100"
    )

    action_node_turtle_service = launch_ros.actions.Node(
        # 功能包名称
        package="patrol_service_cpp",
        # 可执行文件名称
        executable="patrol_service",
        # 输出方式，screen、log、both三种
        output="screen",
    )
    action_node_turtle_client = launch_ros.actions.Node(
        # 功能包名称
        package="patrol_service_cpp",
        # 可执行文件名称
        executable="patrol_client",
        # 输出方式，screen、log、both三种
        output="screen"
    )
    action_node_turtlesim_node = launch_ros.actions.Node(
        # 功能包名称
        package="turtlesim",
        # 可执行文件名称
        executable="turtlesim_node",
        # 输出方式，screen、log、both三种
        output="both",
        # --添加launch参数, 1.节点参数；2.替换launch参数
        parameters=[{
            "background_r": launch.substitutions.LaunchConfiguration("launch_arg_bg_r", default="100")}]
    )
    # 合成启动描述
    launch_description = launch.LaunchDescription([
        # 添加参数动作
        action_declare_arg_background_r,
        action_node_turtle_service,
        action_node_turtle_client,
        action_node_turtlesim_node
    ])
    return launch_description
