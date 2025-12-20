import os
import sys
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
import serial
import struct

# Protocol constants
HEAD_BYTE = 0xAA
TAIL_BYTE = 0x55
ID_CMD_VEL = 0x02

@pytest.mark.launch_test
def generate_test_description():
    package_name = 'auto_serial_bridge'
    
    my_pkg_share = get_package_share_directory(package_name)
    
    common_config = os.path.join(my_pkg_share, 'config', 'serial_data.yaml')
    
    # Start socat to create virtual serial ports
    # /tmp/vtty0 模拟下位机
    # /tmp/vtty1  , 模拟上位机
    socat_cmd = ['socat', '-d', '-d', 'PTY,link=/tmp/vtty0,raw,echo=0', 'PTY,link=/tmp/vtty1,raw,echo=0']
    socat_process = ExecuteProcess(
        cmd=socat_cmd,
        output='screen'
    )

    # Create component
    serial_component = ComposableNode(
        package=package_name,
        plugin='auto_serial_bridge::SerialController',
        name='auto_serial_bridge_node',
        parameters=[common_config, {'port': '/tmp/vtty1'}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Create container
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
        # Wait for socat to be ready (1 second should be enough)
        TimerAction(period=1.0, actions=[container]),
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
        # Connect to the OTHER end of the virtual serial pair
        while(rclpy.ok()):
            
            try:
                self.serial_port = serial.Serial('/tmp/vtty0', baudrate=115200, timeout=1)
                break
            except serial.SerialException:
                time.sleep(0.1)

    def tearDown(self):
        self.node.destroy_node()
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()

    def calculate_checksum(self, data):
        return sum(data) & 0xFF
    
    def simulate_mcu_send(self, packet_id, data_bytes):
        packet = self.pack_packet(packet_id, data_bytes)
        self.serial_port.write(packet)
        self.serial_port.flush()
        
    def test_continous_mcu_send(self):
        for _ in range(10):
            linear_x = 1.0
            angular_z = 0.5
            data = struct.pack('<ff', linear_x, angular_z)
            self.simulate_mcu_send(ID_CMD_VEL, data)
            time.sleep(0.1)
        

    def pack_packet(self, packet_id, data_bytes):
        # FrameHeader: head(1), id(1), length(1)
        # Checksum covers: ID, Length, Data
        
        # Construct payload for checksum calculation
        # ID is uint8, Length is uint8
        payload_for_checksum = struct.pack('<BB', packet_id, len(data_bytes)) + data_bytes
        checksum = self.calculate_checksum(payload_for_checksum)
        
        # Construct full packet
        # Head
        packet = struct.pack('<B', HEAD_BYTE)
        # ID, Length, Data
        packet += payload_for_checksum
        # Checksum, Tail
        packet += struct.pack('<BB', checksum, TAIL_BYTE)
        
        return packet

    def test_receive_from_serial(self):
        """Test that data sent to serial port is published as ROS message."""

        # 重置串口缓冲区，把下位机（socat）没发完的废话清掉 ---
        self.serial_port.reset_input_buffer() 
        self.serial_port.reset_output_buffer()

        linear_x = 1.5
        angular_z = -0.5
        data = struct.pack('<ff', linear_x, angular_z)
        packet = self.pack_packet(ID_CMD_VEL, data)

        received_msgs = []
        sub = self.node.create_subscription(
            Twist,
            '/cmd_vel_feedback',
            lambda msg: received_msgs.append(msg),
            10
        )

        # 持续发送数据直到收到消息或超时
        end_time = time.time() + 5.0
        found_correct_msg = False
        
        while time.time() < end_time and not found_correct_msg:
            # 持续发送，直到“握手”成功并收到消息
            self.serial_port.write(packet)
            self.serial_port.flush()
            
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            # 检查是否收到了期望的消息
            for msg in received_msgs:
                if abs(msg.linear.x - linear_x) < 0.01 and abs(msg.angular.z - angular_z) < 0.01:
                    found_correct_msg = True
                    break
            
            # 清空接收列表，准备下一轮接收，避免处理旧数据
            if not found_correct_msg:
                received_msgs.clear()
                
            time.sleep(0.1) 

        self.assertTrue(found_correct_msg, f"超时未收到期望的 ROS 消息 (linear_x={linear_x}, angular_z={angular_z})")

    def test_send_to_serial(self):
        """Test that ROS message is sent to serial port."""
        # Simulate ROS sending data to MCU
        pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        
        # Clear input buffer
        self.serial_port.reset_input_buffer()
        
        # Publish ROS message multiple times
        for _ in range(100):
            pub.publish(msg)
            time.sleep(0.01)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        # Read from serial
        # We expect a packet with ID_CMD_VEL
        start_time = time.time()
        buffer = b''
        found_packet = False
        
        while time.time() - start_time < 5.0:
            if self.serial_port.in_waiting:
                buffer += self.serial_port.read(self.serial_port.in_waiting)
            
            # Try to find a valid packet in buffer
            # Header: AA
            # Min length: 1 + 1 + 1 + 8 + 1 + 1 = 13
            if len(buffer) >= 13:
                # Search for header
                idx = buffer.find(b'\xAA')
                if idx != -1:
                    # Check if we have enough bytes for a full packet from header
                    if len(buffer) >= idx + 13:
                        packet = buffer[idx:idx+13]
                        # Verify tail
                        if packet[-1] == TAIL_BYTE:
                            # Verify ID
                            if packet[1] == ID_CMD_VEL:
                                # Verify Data
                                data = packet[3:11]
                                lx, az = struct.unpack('<ff', data)
                                if abs(lx - 2.0) < 0.001 and abs(az - 1.0) < 0.001:
                                    found_packet = True
                                    break
                        
                        # If not valid, remove header and continue
                        buffer = buffer[idx+1:]
                    else:
                        # Wait for more data
                        pass
                else:
                    # No header, discard
                    buffer = b''
            
            time.sleep(0.05)
            
        self.assertTrue(found_packet, "Did not receive correct serial packet")
