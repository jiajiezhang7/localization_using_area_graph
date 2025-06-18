#!/bin/bash

# AGLoc自动录制启动脚本

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示帮助信息
show_help() {
    echo "AGLoc系统自动录制启动脚本"
    echo
    echo "用法: $0 [选项]"
    echo
    echo "选项:"
    echo "  -o, --output-dir DIR    录制文件输出目录 (默认: ./recordings)"
    echo "  -d, --duration SEC      录制时长（秒）"
    echo "  -s, --use-system-time  使用系统时间而非仿真时间"
    echo "  -w, --wait-all-topics  等待所有话题都检测到后再开始录制（推荐）"
    echo "  -h, --help             显示此帮助信息"
    echo
    echo "示例:"
    echo "  $0                                    # 基本使用（快速模式）"
    echo "  $0 -w                                # 等待所有话题模式（推荐）"
    echo "  $0 -o /path/to/recordings            # 指定输出目录"
    echo "  $0 -d 120 -w                        # 录制120秒，等待所有话题"
    echo "  $0 -s                               # 使用系统时间"
    echo "  $0 -o ./my_recordings -d 60 -w      # 完整配置"
}

# 检查ROS2环境
check_ros2_env() {
    print_info "检查ROS2环境..."
    
    if ! command -v ros2 &> /dev/null; then
        print_error "未找到ros2命令，请确保已正确安装ROS2"
        return 1
    fi
    
    if [ -z "$ROS_DISTRO" ]; then
        print_warning "ROS_DISTRO环境变量未设置，尝试source setup文件..."
        
        # 尝试source常见的setup文件
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
            print_info "已source ROS2 Humble环境"
        elif [ -f "/opt/ros/foxy/setup.bash" ]; then
            source /opt/ros/foxy/setup.bash
            print_info "已source ROS2 Foxy环境"
        else
            print_error "无法找到ROS2 setup文件，请手动source"
            return 1
        fi
    fi
    
    print_success "ROS2环境检查通过 (ROS_DISTRO: $ROS_DISTRO)"
    return 0
}

# 检查Python依赖
check_dependencies() {
    print_info "检查Python依赖..."
    
    python3 -c "import rclpy" 2>/dev/null
    if [ $? -ne 0 ]; then
        print_error "未找到rclpy模块，请确保已正确安装ROS2 Python包"
        return 1
    fi
    
    print_success "Python依赖检查通过"
    return 0
}

# 主函数
main() {
    # 默认参数
    OUTPUT_DIR="./recordings"
    DURATION=""
    USE_SYSTEM_TIME=false
    WAIT_ALL_TOPICS=false
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -o|--output-dir)
                OUTPUT_DIR="$2"
                shift 2
                ;;
            -d|--duration)
                DURATION="$2"
                shift 2
                ;;
            -s|--use-system-time)
                USE_SYSTEM_TIME=true
                shift
                ;;
            -w|--wait-all-topics)
                WAIT_ALL_TOPICS=true
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                print_error "未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 显示启动信息
    echo
    echo "================================================"
    echo "🎬 AGLoc系统自动录制工具启动脚本"
    echo "================================================"
    echo
    
    # 环境检查
    if ! check_ros2_env; then
        exit 1
    fi
    
    if ! check_dependencies; then
        exit 1
    fi
    
    # 创建输出目录
    if [ ! -d "$OUTPUT_DIR" ]; then
        print_info "创建输出目录: $OUTPUT_DIR"
        mkdir -p "$OUTPUT_DIR"
    fi
    
    # 切换到项目根目录
    cd "$PROJECT_ROOT"
    print_info "工作目录: $(pwd)"
    
    # 构建启动命令
    CMD="python3 localization_using_area_graph/scripts/auto_bag_recorder.py --output-dir $OUTPUT_DIR"
    
    if [ -n "$DURATION" ]; then
        CMD="$CMD --duration $DURATION"
    fi
    
    if [ "$USE_SYSTEM_TIME" = true ]; then
        CMD="$CMD --use-system-time"
    fi
    
    if [ "$WAIT_ALL_TOPICS" = true ]; then
        CMD="$CMD --wait-all-topics"
    fi
    
    print_info "启动命令: $CMD"
    echo
    
    # 显示使用提示
    print_info "使用提示:"
    echo "  1. 录制工具启动后，请在另一个终端启动AGLoc系统："
    echo "     ros2 launch localization_using_area_graph run_time_sync_fixed.launch.py"
    echo "  2. 录制工具会自动检测目标话题并开始录制"
    echo "  3. 按Ctrl+C可以安全停止录制"
    echo
    
    print_success "正在启动自动录制工具..."
    echo
    
    # 启动录制工具
    exec $CMD
}

# 信号处理
trap 'print_info "收到中断信号，正在退出..."; exit 0' INT

# 执行主函数
main "$@" 