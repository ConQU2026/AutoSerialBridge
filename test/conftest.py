import os
import time
import subprocess
import shutil
import pytest

import rclpy


def _have_socat():
	return shutil.which("socat") is not None


@pytest.fixture(scope="module")
def socat_process():
	if not _have_socat():
		pytest.skip("socat not found; skipping virtual-serial tests")

	cmd = [
		"socat",
		"-d",
		"-d",
		"PTY,link=/tmp/vtty0,raw,echo=0",
		"PTY,link=/tmp/vtty1,raw,echo=0",
	]

	p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
											 stderr=subprocess.DEVNULL)

	# wait for device nodes
	for _ in range(40):
		if os.path.exists("/tmp/vtty0") and os.path.exists("/tmp/vtty1"):
			break
		time.sleep(0.05)
	else:
		p.terminate()
		pytest.skip("socat failed to create virtual PTYs")

	yield p

	p.terminate()
	try:
		p.wait(timeout=1)
	except Exception:
		p.kill()


@pytest.fixture(scope="function")
def serial_ports(socat_process):
	try:
		import serial
	except Exception:
		pytest.skip("pyserial not installed; skipping serial tests")

	s0 = serial.Serial("/tmp/vtty0", baudrate=115200, timeout=1)
	s1 = serial.Serial("/tmp/vtty1", baudrate=115200, timeout=1)

	# flush any initial data
	try:
		s0.reset_input_buffer()
		s1.reset_input_buffer()
	except Exception:
		pass

	yield s0, s1

	s0.close()
	s1.close()

 
# @pytest.fixture(scope="function")
# def create_cmd_vel_publisher():
# 	rclpy.init()
# 	node = rclpy.create_node("cmd_vel_publisher")

# 	publisher = node.create_publisher(
# 		msg_type=rclpy.qos.QoSProfile(depth=10),
# 		topic="cmd_vel",
# 		qos_profile=rclpy.qos.QoSProfile(depth=10),
# 	)

# 	yield node, publisher

# 	node.destroy_node()
# 	rclpy.shutdown()



		
