## 项目说明

- 为了从microROS的苦海中逃离, 我们打算写一个通用的ros通信包
- 与电控组解耦和
- 知根知底, 好解决报错
- 提供launch/component方式启动

## 使用说明
在protocol.hpp中配置需要sub和pub的data相关信息


### 代码说明
#### serial_controller
- 使用serial_drvier控制串口的开启与关闭, 以及串口的参数配置

#### packet_handler


#### pprotocol.hpp

#### serial_node.cpp
节点单独启动测试


## TODO
- 自动生成电控对应C代码模块
- udev script

