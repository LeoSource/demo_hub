### 依赖Eigen  
本项目是对于Eigen库的使用总结，运行前确保已正确安装Eigen库。

* Windows
安装Eigen3：`.\vcpkg install eigen3:x64-windows`  
编译时需使用vcpkg的工具链文件
`-DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake`

若通过VSCode的CMakeTools进行编译，可在设置中找到*cmake configure args*，添加`-DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake`。
或者直接按F1（ctrl + shift + p），输入“setting.json"，打开设置，添加如下配置：
```json
"cmake.configureSettings": {
    "CMAKE_TOOLCHAIN_FILE": "[path to vcpkg]/scripts/buildsystems/vcpkg.cmake",
    "VCPKG_TARGET_TRIPLET": "x64-windows"
}
```

* Linux
安装Eigen3：`sudo apt install libeigen3-dev`
直接编译即可