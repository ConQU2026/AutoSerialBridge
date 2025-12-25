---
trigger: always_on
---

# 项目要求
这是一个基于ros2 humble的项目, 系统是ubuntu2204



# build
到ws的根目录下使用`colcon build`进行build, 每次你修改完代码之后都需要通过build测试是否有问题


# test
使用`colcon test --packages-select auto_serial_bridge --event-handlers console_direct+`特定的包进行test, 还需使用详细输出功能