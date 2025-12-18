# Copyright 2025 ConQU Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ========================================================================
    # 1. 全局配置区
    # ========================================================================
    # 你的包名 (必须正确！)
    my_package_name = 'serial_pkg'
    
    # 获取 share 目录
    my_pkg_share = get_package_share_directory(my_package_name)
    
    # 全局通用的配置文件路径
    common_config = os.path.join(my_pkg_share, 'config', 'serial_data.yaml')
    
    
    # component 节点配置


