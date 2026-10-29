#!/usr/bin/env bash

set -e

# 以脚本所在目录作为工作区根目录，因此从其他目录执行本脚本也能找到工作区。
# 移植到其他项目时，通常不需要改这里；如果脚本不放在工作区根目录，需调整此路径。
WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Catkin 工作区编译后的环境入口。它会设置 ROS_PACKAGE_PATH、PATH 等运行环境。
# 移植到其他工作区时，请确认该文件存在；若使用不同的构建目录或 ROS 安装方式，需修改此处。
SETUP_FILE="$WORKSPACE_DIR/devel/setup.bash"

# 及早检查环境入口，避免打开终端后才发现工作区尚未构建或路径配置错误。
if [[ ! -f "$SETUP_FILE" ]]; then
    printf 'ROS workspace setup not found: %s\n' "$SETUP_FILE" >&2
    exit 1
fi

# 本脚本使用 GNOME Terminal 的 --window/--tab 参数在同一窗口中创建标签页。
# 移植到其他桌面环境时，需将下面的终端调用改为对应终端模拟器支持的参数。
if ! command -v gnome-terminal >/dev/null 2>&1; then
    printf 'gnome-terminal is required to open separate terminals.\n' >&2
    exit 1
fi

# 保存 GNOME Terminal 参数，最后一次性调用它，以便所有标签页放在同一个窗口。
TERMINAL_ARGS=()

# 添加一个终端标签页。
# 参数依次为：窗口/标签类型、标签标题、要运行的命令及其参数。
# 第一个标签使用 --window 创建窗口，后续标签使用 --tab 加入该窗口。
add_terminal_tab() {
    local tab_type="$1"
    local title="$2"
    shift
    shift

    # 每个标签先 source 工作区环境，再运行传入的 ROS 命令。
    # 命令结束后执行 exec bash 保留交互式 shell，便于查看输出或手动排查问题。
    # 这里传给 GNOME Terminal 的是命令字符串，因此后续参数必须分别 shell 转义。
    local command='bash -c '\''source "$1" && { shift; "$@"; }; exec bash'\'' bash'
    local argument
    local quoted_argument
    # setup 文件作为 bash -c 脚本的第一个位置参数传入；命令及参数随后逐个追加。
    printf -v quoted_argument '%q' "$SETUP_FILE"
    command+=" $quoted_argument"
    for argument in "$@"; do
        printf -v quoted_argument '%q' "$argument"
        command+=" $quoted_argument"
    done

    TERMINAL_ARGS+=("$tab_type" "--title=$title" "--working-directory=$WORKSPACE_DIR" "--command=$command")
}

# 按需增删或替换以下启动项。每一行对应一个标签页：标题在前，后面是可执行命令及参数。
# 移植到其他项目时，主要修改这些命令；第一个 --window 保持为窗口入口，后续使用 --tab。
add_terminal_tab --window "Vehicle Simulator" roslaunch vehicle_simulator system_real_robot.launch
add_terminal_tab --tab "Waypoint Recorder" roslaunch waypoint_recorder waypoint_recorder.launch
add_terminal_tab --tab "RQT Reconfigure" rosrun rqt_reconfigure rqt_reconfigure

# 将窗口和标签页参数一次性交给终端模拟器。
gnome-terminal "${TERMINAL_ARGS[@]}"