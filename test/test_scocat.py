import os
import time
import subprocess
import shutil
import pytest
import rclpy

def test_loopback(serial_ports):
	s0, s1 = serial_ports
	msg = "hello-串口测试\n".encode("utf-8")

	s0.write(msg)
	s0.flush()

	# give a small moment for transfer
	time.sleep(0.05)

	data = s1.read(len(msg))
	assert data == msg



