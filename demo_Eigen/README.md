### 依赖Eigen  
本项目是对于Eigen库的使用总结，运行前确保已正确安装Eigen库。

* Windows
安装Eigen3：`.\vcpkg install eigen3:x64-windows`  
编译时需使用vcpkg的工具链文件
`-DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake`


* Linux
安装Eigen3：`sudo apt install libeigen3-dev`
直接编译即可