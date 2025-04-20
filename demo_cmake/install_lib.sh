#!/bin/bash
set -e  # 任何命令失败则退出

# 定义全局参数
INSTALL_PREFIX="/home/zxliao/custom_libs"  # 自定义安装路径
NUM_JOBS=$(nproc)                        # 并行编译线程数
LOG_FILE="install.log"                   # 编译日志文件

# 创建安装目录
mkdir -p "$INSTALL_PREFIX"
# sudo chown -R $USER:$USER "$INSTALL_PREFIX"  # 确保当前用户有权限

# 安装系统级依赖（Eigen3和Boost）
# sudo apt-get update && sudo apt-get install -y \
#     libeigen3-dev \
#     libboost-all-dev


# 安装 hpp-fcl
install_hpp_fcl() {
    echo "=== 开始安装 hpp-fcl ==="
    # git clone https://github.com/humanoid-path-planner/hpp-fcl.git
    # cd hpp-fcl
    cd /home/zxliao/Documents/install_script/coal
    mkdir build && cd build
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DBUILD_PYTHON_INTERFACE=OFF \
        -DHPP_FCL_HAS_QHULL=ON \
        -DBUILD_TESTING=OFF
    make -j2
    make install
    cd ../..
    echo "=== hpp-fcl 安装完成 ==="
}

# 安装 pinocchio（依赖 hpp-fcl）
install_pinocchio() {
    echo "=== 开始安装 pinocchio ==="
    # git clone https://github.com/stack-of-tasks/pinocchio.git
    # cd pinocchio
    cd /home/zxliao/Documents/install_script/pinocchio-2.7.0
    # 清理并重建 build 目录
    rm -rf build >/dev/null 2>&1 || true
    mkdir build && cd build
    {
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DBUILD_PYTHON_INTERFACE=OFF \
        -DBUILD_WITH_COLLISION_SUPPORT=ON
    make -j VERBOSE=0
    make install
    }
    cd ../..
    echo "=== pinocchio 安装完成 ==="
}

# # 主安装流程
# install_hpp_fcl
if [ ! -f "$INSTALL_PREFIX/lib/libhpp-fcl.so" ]; then
    install_hpp_fcl
fi
install_pinocchio

# # 配置环境变量
# echo "export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
# echo "export CPLUS_INCLUDE_PATH=$INSTALL_PREFIX/include:\$CPLUS_INCLUDE_PATH" >> ~/.bashrc
# source ~/.bashrc

echo "=== 全部安装完成！ ==="
