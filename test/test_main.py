
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

import yaml

# 动态加载协议配置
def load_protocol_config():
    try:
        package_name = 'auto_serial_bridge'
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory(package_name)
        config_path = os.path.join(share_dir, 'config', 'protocol.yaml')
        
        with open(config_path, 'r') as f:
            content = f.read()
            config = yaml.safe_load(content)
            
        return config, content
    except Exception as e:
        print(f"Error loading protocol config: {e}")
        return None, None

PROTOCOL_CONFIG, PROTOCOL_YAML_CONTENT = load_protocol_config()

def get_config_value(key, default):
    if PROTOCOL_CONFIG and 'config' in PROTOCOL_CONFIG:
        return PROTOCOL_CONFIG['config'].get(key, default)
    return default

def get_message_id(name):
    if PROTOCOL_CONFIG and 'messages' in PROTOCOL_CONFIG:
        for msg in PROTOCOL_CONFIG['messages']:
            if msg['name'] == name:
                return msg['id']
    return None

# 协议常量 (动态获取)
HEAD1 = get_config_value('head_byte_1', 0x5A)
HEAD2 = get_config_value('head_byte_2', 0xA5)
ID_HANDSHAKE = get_message_id('Handshake')
ID_HEARTBEAT = get_message_id('Heartbeat')

if ID_HANDSHAKE is None: ID_HANDSHAKE = 0x00
if ID_HEARTBEAT is None: ID_HEARTBEAT = 0x01

def get_protocol_hash():
    if PROTOCOL_YAML_CONTENT:
        # 必须匹配 codegen.py 的逻辑
        return int(hashlib.md5(PROTOCOL_YAML_CONTENT.encode('utf-8')).hexdigest()[:8], 16)
    return 0

PROTOCOL_HASH = get_protocol_hash()


# CRC8 生成逻辑 (避免硬编码表)
def generate_crc8_table(polynomial=0x31):
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc = crc << 1
        table.append(crc & 0xFF)
    return table

CRC8_TABLE = generate_crc8_table()
# CRC8_TABLE = [
#    0x00, 0x31, 0x62, ... (Dynamic now)
# ]


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
        
        # 2. 测试从串口接收 (Serial -> ROS)
        # MCU 发送心跳 (ID 1), ROS 发布 /serial/heartbeat
        
        received_msgs = []
        sub_hb = self.node.create_subscription(
            UInt8,
            '/serial/heartbeat',
            lambda msg: received_msgs.append(msg),
            10
        )
        
        hb_data = struct.pack('<B', 123)
        packet = self.pack_packet(ID_HEARTBEAT, hb_data)
        
        # 重试循环以考虑到发现延迟
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
        # 协议定义 Heartbeat 为 'both' 方向。
        # 发布到 /serial/heartbeat_cmd, 应该在串口通过 ID 0x01 收到
        pub = self.node.create_publisher(UInt8, '/serial/heartbeat_cmd', 10)
        msg = UInt8()
        msg.data = 200
        
        # 清空串口缓冲区
        self.serial_port.reset_input_buffer()
        
        for _ in range(10):
            pub.publish(msg)
            time.sleep(0.1)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        # 检查串口是否有 Heartbeat 数据包
        start_time = time.time()
        found_packet = False
        buf = b''
        while time.time() - start_time < 2.0:
            if self.serial_port.in_waiting:
                buf += self.serial_port.read(self.serial_port.in_waiting)
            
            # 搜索 ID_HEARTBEAT (0x01)
            # Head(2)+ID(1)+Len(1)+Data(1)+CRC(1) = 6 字节
            if len(buf) >= 6:
                 idx = buf.find(b'\x5A')
                 if idx != -1 and len(buf) >= idx+6:
                     if buf[idx+1] == 0xA5 and buf[idx+2] == ID_HEARTBEAT:
                         # Check data (1 byte)
                         data_byte = buf[idx+4]
                         if data_byte == 200:
                             found_packet = True
                             break
                     else:
                          # 丢弃 1 字节
                         buf = buf[idx+1:]
                 else:
                     if idx == -1: buf = b''
            time.sleep(0.05)
            
        self.assertTrue(found_packet, "未在串口上收到 /serial/heartbeat_cmd 的 Heartbeat 数据")
