#### 1. Ubuntu22下，飞书截屏只能截全屏且无法对截屏进行编辑  
  需要打开文件`/etc/gdm3/custom.conf`，将`#WaylandEnable=false`前的`#`去掉，保存并重启电脑  
  详情可参考[Ubuntu系统下，截图软件（飞书，flameshot等）无法正常截图与编辑](https://blog.csdn.net/weixin_43686259/article/details/140848106)  
  常用的截屏工具：火焰截图-Flameshot  
  常用的录屏工具：Kazam  
#### 2. ros2 bag play
最常见需求：回放指定topic，截取时间段，设置回放速率  
```
ros2 bag play [bag] --start-offset [xx] -r [x] --topic [topic]
ros2 bag play merge_recover.mcap --start-offset 1589 -r 3 --topic /body_drive/arm_joint_state
```
#### 3. 进入指定docker容器
  `docker start aimdev-ssh && docker exec -it aimdev-ssh /bin/zsh`  
  其中`aimdev-ssh`为指定容器名称
#### 4. 本地仿真录包
  在build/install/bin/tools/rosbag.sh脚本中实现
#### 5. 安装unity3d
  先添加`unityhub`的储存库  
  `sudo sh -c 'echo "deb https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'`  
  获取`unityhub`的安装密钥  
  `wget -qO - https://hub.unity3d.com/linux/keys/public | sudo apt-key add -`  
  更新软件源并安装`unityhub`  
  `sudo apt update`  
  `sudo apt-get install unityhub`  
  然后在`unityhub`上创建账号，安装`editor`
#### 6. `pinocchio::rosPaths()`
  调用`pinocchio::rosPaths()`时需包含头文件`#include <pinocchio/utils/file-explorer.hpp>`
#### 7. 在cmake中install时，只复制文件夹下的所有内容至指定文件夹下，不需要原文件夹路径
```
install(
  DIRECTORY ${CUR_INSTALL_SOURCE_DIR}/
  DESTINATION ./robot_model
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*")
```
#### 8. 显示ROS包路径
```
echo $ROS_PACKAGE_PATH==>ROS1
echo $AMENT_PREFIX_PATH==>ROS2
```
#### 9. 查询已安装包的版本
  查询第三方库的版本号没有统一的方法，大部分库本身不支持 库名 --version直接查询。但没关系，有一个工具叫做`apt-show-versions`，可以直接通过`apt-get install`命令从软件源安装。安装后，只需要输入`apt-show-versions | grep`包名关键字，即可查询。
#### 10. 运行aimrt下的robot_visualization包时机器人显示异常
首先删除build文件夹里的编译内容，再重新编译
```
source share/hal_msgs/local_setup.bash
source share/ros2_data_type/local_setup.bash
```
  这两个文件在build/install/路径下
#### 11. 运行rviz2时异常
当通过最简单的启动方式`ros2 run rviz2 rviz2`来启动`rviz2`时都会出现如下报错信息
```
[ERROR] [1746510336.521587368] [rviz2]: Failed to create an OpenGL context. BadValue (integer parameter out of range for operation)
[ERROR] [1746510336.521616085] [rviz2]: RenderingAPIException: Unable to create a suitable GLXContext in GLXContext::GLXContext at ./.obj-x86_64-linux-gnu/ogre-v1.12.1-prefix/src/ogre-v1.12.1/RenderSystems/GLSupport/src/GLX/OgreGLXContext.cpp (line 60)
[ERROR] [1746510336.521643812] [rviz2]: rviz::RenderSystem: error creating render window: RenderingAPIException: Unable to create a suitable GLXContext in GLXContext::GLXContext at ./.obj-x86_64-linux-gnu/ogre-v1.12.1-prefix/src/ogre-v1.12.1/RenderSystems/GLSupport/src/GLX/OgreGLXContext.cpp (line 60)
```
这个故障的原因是英伟达显卡驱动无法识别，通过输入`nvidia-smi`来确认，此时显示无法初始化显卡驱动。重启电脑后暂时解决该问题。
#### 12. plotjuggler使用
在plotjuggler上方界面，点击Help/Cheatsheet按钮进入操作帮助文档查看。
#### 13. 编译链接问题
当环境中安装了多个相同的库，在cmake编译时，通过find_package在指定路径下找到了该库，但是最终链接的可能是另一个库，譬如hpp-fcl。
编译好后，通过`ldd <可执行文件>`指令查看具体的链接路径，如果出现不一致的情况，可在`target_link_libraries`后设置link属性，举例如下
```
set_target_properties(demo_pcl_fcl PROPERTIES
LINK_FLAGS "-Wl,-rpath=/home/zy/Documents/a2w_t3d5/aimrt_motion_control/build/install/mc3th/lib -Wl,--disable-new-dtags"
)
```
重新编译完成后再利用ldd命令来确认。
#### 14. 查找文件路径
通过`sudo apt install mlocate`指令安装`locate`工具，然后使用`locate <file>`便可以查找电脑中该文件的具体路径。
有时候会出现刚在某个文件夹内增加了文件，然后将窗口关闭了，导致无法确定该文件的具体路径，此时需要先更新数据库`sudo updatedb`(可参考该[链接](https://blog.csdn.net/u012964600/article/details/138013445))，再用locate工具搜索指定文件。
#### 15. pcl与ros
[perception_pcl库](https://github.com/ros-perception/perception_pcl)包含了pcl与ROS之间常用的使用方法，主要包含`pcl_conversions`与`pcl_ros`两个功能，`pcl_ros`做了在`ros`中使用`pcl`库的一些基础功能接口，可以使得直接通过nodelet进行调用pcl的一些功能。  
`pcl_conversions` 做了一些ros的消息类型与pcl消息类型的转换。  
ros中并没有定义pcl的数据类型，所以，只要是pcl相关的数据类型，都是pcl自身的，没有ros中定义的，ros只是定义了一些转换函数。
可参考[perception_pcl理解 --- pcl_conversions 与 pcl_ros](https://blog.csdn.net/tiancailx/article/details/110816649),[如何在ROS2的环境下使用Pcl_ros以及pcl 功能包](https://fishros.org.cn/forum/topic/574/%E5%A6%82%E4%BD%95%E5%9C%A8ros2%E7%9A%84%E7%8E%AF%E5%A2%83%E4%B8%8B%E4%BD%BF%E7%94%A8pcl_ros%E4%BB%A5%E5%8F%8Apcl-%E5%8A%9F%E8%83%BD%E5%8C%85)
#### 16. ros2在zsh下无法补全
这是个隐形bug，在bash中正常，但是切换到zsh后无法自动补全，[参考解决方案](https://www.cnblogs.com/USTHzhanglu/p/16544776.html)。  
在`/opt/ros/humble/setup.zsh`文件末尾补充如下如下两个指令之一
```
# 指令1
complete -o nospace -o default -F _python_argcomplete "ros2"
# 指令2
# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```
但是在source完自己的包后自动补全功能会被覆盖导致失效，此时还得再次执行`source ~/.zshrc`，目前没找到更好的解决方法。
#### 17. pyenv管理多版本python虚拟环境
安装：选择自动安装方式
```
curl https://pyenv.run | bash
# 或
curl -L https://github.com/pyenv/pyenv-installer/raw/master/bin/pyenv-installer | bash
```
配置环境：在.bashrc或者.zshrc文件里面增加如下脚本
```
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"
```
再新开终端并输入pyenv后应能看到所有的帮助信息  
最常见的操作指令：  
pyenv install 3.x.x：安装指定版本的python解释器  
pyenv versions：查看所有安装的python版本  
pyenv virtualenvs：查看所有的虚拟环境列表  
pyenv virtualenv <python_version> <virtualenv_name>：创建对应版本的虚拟环境  
pyenv activate <virtualenv_name>：激活虚拟环境  
pyenv deactivate：退出虚拟环境  
pyenv uninstall <virtualenv_name>：删除虚拟环境  
pyenv shell <python_version>：在该终端使用特定版本  
pyenv global/local <python_version> ：在全局/局部使用特定版本  
pyenv shell system：使用系统版本python  
在使用pinocchio库时和系统环境有冲突，ROS2的全部系统环境也有影响  
#### 18. Isaac Gym弹窗黑屏且无响应
这是因为英特尔集成显卡的干扰，未使用NVIDIA的GPU。  
最简单粗暴的处理方式就是直接在终端输入
```
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
```
#### 19. ROS2源公共密钥过期的解决办法
在安装ROS2相关的包甚至`sudo apt update`时会有如下警告：
```
错误:4 Index of /ros2/ubuntu/ | 清华大学开源软件镜像站 | Tsinghua Open Source Mirror jammy InRelease
下列签名无效： EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
命中:9 404 Not Found | Packagecloud jammy InRelease
```
这是因为ROS官方GPG公钥已过期，需要清理旧密钥，下载并导入新版密钥，重新添加ROS2软件源。  
**删除原有的所有ROS GPG公钥**  
```
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
sudo rm -f /etc/apt/keyrings/ros-archive-keyring.gpg
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /etc/apt/sources.list.d/ros-fish.list
```
ros-archive-keyring.gpg：旧版 ROS 2 公钥可能位于 /usr/share/keyrings 或 /etc/apt/keyrings。  
ros2.list 与 ros-fish.list：之前添加的 ROS 2 软件源文件，删除它们以便后续重新配置。  
**安装工具依赖**  
在下载与安装新版公钥时，需要 curl、gnupg 与 lsb-release 等常用工具。执行以下命令，确保它们已安装在系统中：`sudo apt install -y curl gnupg lsb-release`  
**下载并安装新版GPG公钥**  
```
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
```
添加ROS2软件源
```
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
