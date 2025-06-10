#!/bin/zsh
# SLAM开发环境容器管理脚本（适配 macOS 和 Linux）
# 版本：2.1

# ------------------ 系统判断 ------------------
IS_MAC=false
if [[ "$(uname)" == "Darwin" ]]; then
  IS_MAC=true
fi

# ------------------ 配置区域 ------------------
IMAGE_NAME="registry.cn-hangzhou.aliyuncs.com/slam_project/slam_practise_env:2024-07-04"
CONTAINER_NAME="slam_hub"
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)" # 设置为脚本所在目录
DATA_MOUNTS=("$HOME:/home/$USER")              # 用户主目录挂载
PORTS=("6666:6666")                            # 可根据需要扩展端口

# ------------------ 颜色定义 ------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ------------------ 工具函数 ------------------

show_help() {
  echo -e "${GREEN}SLAM开发环境容器管理脚本${NC}"
  echo "用法: $0 [create|enter|stop|remove|save <tag>|status|help]"
  echo -e "\n${YELLOW}当前配置:${NC}"
  echo "  镜像名称: $IMAGE_NAME"
  echo "  容器名称: $CONTAINER_NAME"
  echo "  工作目录: $WORKSPACE_DIR"
  echo "  数据挂载: ${DATA_MOUNTS[@]}"
  if $IS_MAC; then
    echo -e "\n${YELLOW}[macOS 检测到]：请确保安装 XQuartz 并已启动。${NC}"
    echo "  https://www.xquartz.org/"
    echo "  在 XQuartz 设置中开启：Preferences > Security > Allow connections from network clients"
  fi
}

check_docker() {
  if ! docker info &>/dev/null; then
    echo -e "${RED}错误: Docker 守护进程未运行，请启动 Docker${NC}"
    exit 1
  fi
}

create_container() {
  # 清理已存在容器
  if docker ps -aq -f name=$CONTAINER_NAME | grep -q .; then
    echo -e "${YELLOW}容器已存在，先删除旧容器...${NC}"
    docker rm -f $CONTAINER_NAME
  fi

  # 构造挂载参数
  local mount_args=()
  for mount in "${DATA_MOUNTS[@]}"; do
    mount_args+=(-v "$mount")
  done

  # 构造端口参数
  local port_args=()
  for port in "${PORTS[@]}"; do
    port_args+=(-p "$port")
  done

  # DISPLAY 设置
  if $IS_MAC; then
    DISPLAY_ENV="host.docker.internal:0"
    NETWORK_ARG=() # macOS 不支持 --network host
  else
    DISPLAY_ENV=$DISPLAY
    NETWORK_ARG=(--network host)
    xhost +local:docker >/dev/null 2>&1
  fi

  # ssh 挂载判断
  if [ -d "$HOME/.ssh" ]; then
    SSH_MOUNT=(-v "$HOME/.ssh:/root/.ssh")
  else
    SSH_MOUNT=()
  fi

  echo -e "${BLUE}正在创建 Docker 容器...${NC}"

  docker run \
    "${NETWORK_ARG[@]}" \
    "${port_args[@]}" \
    -e DISPLAY=$DISPLAY_ENV \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    "${SSH_MOUNT[@]}" \
    "${mount_args[@]}" \
    -w "$WORKSPACE_DIR" \
    -itd --gpus all --name "$CONTAINER_NAME" "$IMAGE_NAME" zsh

  if [ $? -eq 0 ]; then
    echo -e "${GREEN}容器创建成功!${NC}"
    echo -e "容器名称: ${YELLOW}$CONTAINER_NAME${NC}"
    echo -e "镜像名称: ${YELLOW}$IMAGE_NAME${NC}"
    echo -e "工作目录: ${YELLOW}$WORKSPACE_DIR${NC}"
  else
    echo -e "${RED}容器创建失败!${NC}"
    exit 1
  fi
}

enter_container() {
  if ! docker ps -aq -f name=$CONTAINER_NAME | grep -q .; then
    echo -e "${YELLOW}容器不存在，正在创建...${NC}"
    create_container
  fi

  if ! docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
    echo -e "${YELLOW}容器未运行，正在启动...${NC}"
    docker start $CONTAINER_NAME
  fi

  echo -e "${BLUE}正在进入容器...${NC}"
  echo -e "${YELLOW}提示：使用 'exit' 退出容器${NC}"
  docker exec -it $CONTAINER_NAME zsh
}

stop_container() {
  if docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
    echo -e "${YELLOW}正在停止容器...${NC}"
    docker stop $CONTAINER_NAME
    echo -e "${GREEN}容器已停止${NC}"
  else
    echo -e "${YELLOW}容器未运行${NC}"
  fi
}

remove_container() {
  stop_container
  if docker ps -aq -f name=$CONTAINER_NAME | grep -q .; then
    echo -e "${YELLOW}正在删除容器...${NC}"
    docker rm $CONTAINER_NAME
    echo -e "${GREEN}容器已删除${NC}"
    if ! $IS_MAC; then
      xhost -local:docker >/dev/null 2>&1
    fi
  else
    echo -e "${YELLOW}容器不存在${NC}"
  fi
}

save_container() {
  if [ -z "$1" ]; then
    echo -e "${RED}错误：请提供新镜像的标签${NC}"
    echo "用法: $0 save [new-image:tag]"
    exit 1
  fi
  stop_container
  if docker ps -aq -f name=$CONTAINER_NAME | grep -q .; then
    docker commit $CONTAINER_NAME "$1"
    echo -e "${GREEN}容器已保存为新镜像: $1${NC}"
  else
    echo -e "${RED}容器不存在，无法保存${NC}"
  fi
}

container_status() {
  echo -e "${BLUE}容器状态:${NC}"
  docker ps -a -f name=$CONTAINER_NAME --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
  if docker ps -aq -f name=$CONTAINER_NAME | grep -q .; then
    echo -e "\n${BLUE}挂载点:${NC}"
    docker inspect $CONTAINER_NAME --format '{{ range .Mounts }}{{ .Source }} => {{ .Destination }}{{"\n"}}{{ end }}'
  fi
}

# ------------------ 主程序 ------------------

check_docker

case "$1" in
create)
  create_container
  ;;
enter | "")
  enter_container
  ;;
stop)
  stop_container
  ;;
remove)
  remove_container
  ;;
save)
  save_container "$2"
  ;;
status)
  container_status
  ;;
help)
  show_help
  ;;
*)
  echo -e "${RED}未知命令 '$1'${NC}"
  show_help
  exit 1
  ;;
esac
