import launch
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1.声明rqt启动控制参数
    action_declare_arg_start_rqt = launch.actions.DeclareLaunchArgument(
        "launch_arg_start_rqt",
        default_value="False"
    )
    # 2.替换launch参数
    start_rqt = launch.substitutions.LaunchConfiguration(
        "launch_arg_start_rqt",
        default="False"
    )
    # 小海龟的一个launch文件路径
    multisim_launch_path = [get_package_share_directory("turtlesim"), "/launch/", "multisim.launch.py"]

    # 动作1-启动其他launch文件
    action_include_launch = launch.actions.IncludeLaunchDescription(
        # 填入launch文件地址(列表拼接)
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_launch_path
        )
    )
    
    # 动作2-打印日志
    action_log_info = launch.actions.LogInfo(
        msg=str(multisim_launch_path)
    )

    # 动作3-执行终端命令
    action_cmd_ctrl = launch.actions.ExecuteProcess(
        # 3.添加条件
        condition=launch.conditions.IfCondition(start_rqt),
        # 单个指令，列表拼接，例如 ['ros2', 'topic', 'list']
        cmd=['rqt']
    )

    # 动作4-多个动作 组成 动作组
    action_group = launch.actions.GroupAction([
        # 动作5-定时器，定时第几秒启动什么指令
        launch.actions.TimerAction(period=2.0, actions=[action_include_launch]),
        launch.actions.TimerAction(period=4.0, actions=[action_cmd_ctrl])
    ])

    # 合成启动描述
    launch_description = launch.LaunchDescription([
        action_declare_arg_start_rqt,
        action_log_info,
        action_group
    ])
    return launch_description
