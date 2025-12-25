
import os
import sys
import hashlib
import time
import unittest
import pytest
import launch
import launch_ros.actions
import launch_testing.actions
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32, UInt8
import serial
import struct

# 协议常量
HEAD1 = 0x5A
HEAD2 = 0xA5
ID_HANDSHAKE = 0x00
ID_HEARTBEAT = 0x01
ID_CMD_VEL = 0x03
ID_CHASSIS_STATUS = 0x04

def get_protocol_hash():
    try:
        package_name = 'auto_serial_bridge'
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory(package_name)
        config_path = os.path.join(share_dir, 'config', 'protocol.yaml')
        with open(config_path, 'r') as f:
            content = f.read()
            # 必须匹配 codegen.py 的逻辑
            return int(hashlib.md5(content.encode('utf-8')).hexdigest()[:8], 16)
    except Exception as e:
        print(f"Error calculating protocol hash: {e}")
        return 0

PROTOCOL_HASH = get_protocol_hash()

# CRC8 表 (与生成的相同)
CRC8_TABLE = [
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC,
]

def calculate_checksum(data):
    crc = 0
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc

@pytest.mark.launch_test
def generate_test_description():
    package_name = 'auto_serial_bridge'
    
    my_pkg_share = get_package_share_directory(package_name)
    
    common_config = os.path.join(my_pkg_share, 'config', 'protocol.yaml') # Update path if needed, but node param usually separate.
    # 实际上之前使用了参数 'serial_data.yaml'
    # 新的 serial_controller 使用 `config/protocol.yaml` 进行代码生成，但运行时的参数 `baudrate` 等是标准参数。
    
    socat_cmd = ['socat', '-d', '-d', 'PTY,link=/tmp/vtty0,raw,echo=0', 'PTY,link=/tmp/vtty1,raw,echo=0']
    socat_process = ExecuteProcess(
        cmd=socat_cmd,
        output='screen'
    )

    serial_component = ComposableNode(
        package=package_name,
        plugin='auto_serial_bridge::SerialController',
        name='auto_serial_bridge_node',
        parameters=[{'port': '/tmp/vtty1', 'baudrate': 921600}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'debug'],
        composable_node_descriptions=[serial_component],
        output='screen',
    )

    return launch.LaunchDescription([
        socat_process,
        TimerAction(period=2.0, actions=[container]),
        launch_testing.actions.ReadyToTest(),
    ])

class TestSerialController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_serial_controller_client')
        while(rclpy.ok()):
            try:
                self.serial_port = serial.Serial('/tmp/vtty0', baudrate=921600, timeout=1)
                break
            except serial.SerialException:
                time.sleep(0.1)

    def tearDown(self):
        self.node.destroy_node()
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()

    def pack_packet(self, packet_id, data_bytes):
        # Header(2) + ID(1) + Len(1) + Data(N) + CRC(1)
        length = len(data_bytes)
        
        packet = struct.pack('<BBBB', HEAD1, HEAD2, packet_id, length)
        packet += data_bytes
        
        # 校验和覆盖 ID, Len, Data
        payload_for_checksum = struct.pack('<BB', packet_id, length) + data_bytes
        checksum = calculate_checksum(payload_for_checksum)
        
        packet += struct.pack('<B', checksum)
        return packet

    def wait_for_handshake(self):
        # 等待握手请求 (ID 0)
        start_time = time.time()
        buf = b''
        while time.time() - start_time < 5.0:
            if self.serial_port.in_waiting:
                buf += self.serial_port.read(self.serial_port.in_waiting)
            
            # 简单的握手解析器
            # 5A A5 00 04 [HASH 4 bytes] [CRC]
            if len(buf) >= 9: # 2+1+1+4+1 = 9
                idx = buf.find(b'\x5A')
                if idx != -1 and len(buf) >= idx + 9:
                    if buf[idx+1] == 0xA5 and buf[idx+2] == ID_HANDSHAKE:
                         # 发现握手请求
                         # 发送握手响应
                         hash_bytes = struct.pack('<I', PROTOCOL_HASH)
                         resp = self.pack_packet(ID_HANDSHAKE, hash_bytes)
                         self.serial_port.write(resp)
                         self.serial_port.flush()
                         return True
                    else:
                        buf = buf[idx+1:]
                else:
                    if idx == -1: buf = b''
            time.sleep(0.01)
        return False

    def test_communication(self):
        # 1. 执行握手
        self.assertTrue(self.wait_for_handshake(), "Handshake failed")
        
        # 2. 测试从串口接收 (模拟 MCU 发送 ChassisStatus -> ROS /odom)
        # 这里不使用 Odom，因为获取 Odom 消息可能需更多导入或检查生成的绑定映射
        # 但生成的绑定将 ChassisStatus (ID 4) 映射到了 /odom
        # 让我们尝试发送 ID 4
        
        received_msgs = []
        sub = self.node.create_subscription(
             # We need nav_msgs.msg.Odometry. But lazy, let's import
             # Assume import nav_msgs.msg
             # Dynamically get class?
             # from nav_msgs.msg import Odometry is safer
             # See imports at top, need to add if missing
             Twist, # Twist 用于 CmdVel。
             '/cmd_vel', # SerialController 订阅 /cmd_vel 并发送到串口。
             lambda msg: None, # 哑函数
             10
        )
        # 等等，我想验证 RX。从串口接收。
        # SerialController 监听串口并发布。
        # ID_CHASSIS_STATUS -> /odom (Odometry)
        # ID_HEARTBEAT -> /heartbeat (UInt8)
        
        # 让我们测试心跳接收 (更简单)
        # MCU 发送心跳 (ID 1), ROS 发布 /heartbeat
        
        sub_hb = self.node.create_subscription(
            UInt8,
            '/serial/heartbeat',
            lambda msg: received_msgs.append(msg),
            10
        )
        
        hb_data = struct.pack('<B', 123)
        packet = self.pack_packet(ID_HEARTBEAT, hb_data)
        
        # 重试循环以考虑到发现延迟 (懒加载创建的发布者需要时间)
        end_time = time.time() + 5.0
        found = False
        while time.time() < end_time:
            self.serial_port.write(packet)
            self.serial_port.flush()
            
            # 短暂自旋以检查响应
            spin_end = time.time() + 0.5
            while time.time() < spin_end:
                 rclpy.spin_once(self.node, timeout_sec=0.1)
                 for msg in received_msgs:
                     if msg.data == 123:
                         found = True
                         break
                 if found: break
            if found: break
        
        self.assertTrue(found, "Did not receive Heartbeat from ROS")

        # 3. 测试发送到串口 (ROS -> Serial)
        # 发布到 /cmd_vel
        pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        msg = Twist()
        msg.linear.x = 2.5
        msg.angular.z = 1.2
        
        # 清空串口缓冲区
        self.serial_port.reset_input_buffer()
        
        for _ in range(10):
            pub.publish(msg)
            time.sleep(0.1)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        # 检查串口是否有 CmdVel 数据包
        start_time = time.time()
        found_packet = False
        buf = b''
        while time.time() - start_time < 2.0:
            if self.serial_port.in_waiting:
                buf += self.serial_port.read(self.serial_port.in_waiting)
            
            # 搜索 ID_CMD_VEL (0x03)
            # Head(2)+ID(1)+Len(1)+Data(8)+CRC(1) = 13 字节
            if len(buf) >= 13:
                 idx = buf.find(b'\x5A')
                 if idx != -1 and len(buf) >= idx+13:
                     if buf[idx+1] == 0xA5 and buf[idx+2] == ID_CMD_VEL:
                         # Check data
                         data = buf[idx+4:idx+12]
                         lx, az = struct.unpack('<ff', data)
                         if abs(lx - 2.5) < 0.01 and abs(az - 1.2) < 0.01:
                             found_packet = True
                             break
                     else:
                          # 丢弃 1 字节
                         buf = buf[idx+1:]
                 else:
                     if idx == -1: buf = b''
            time.sleep(0.05)
            
        self.assertTrue(found_packet, "未在串口上收到 /cmd_vel 数据")
