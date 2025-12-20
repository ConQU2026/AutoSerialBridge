#!/bin/bash

# ==========================================
# 自动生成 Udev 规则脚本
# 作者: Gemini Thought Partner
# 功能: 根据用户输入的设备路径，自动生成持久化别名规则
# ==========================================

# 颜色定义，为了让输出好看一点
GREEN='\033[0;32m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 1. 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then
  echo -e "${RED}[错误] 请使用 sudo 运行此脚本！${NC}"
  echo "示例: sudo ./auto_udev.sh"
  exit 1
fi

echo -e "${CYAN}=== 自动 Udev 规则生成器 ===${NC}"

# 2. 获取用户输入
# 步骤 A: 获取原始设备路径
read -p "请输入当前的设备路径 (例如 /dev/ttyACM0): " SOURCE_DEV

if [ ! -e "$SOURCE_DEV" ]; then
    echo -e "${RED}[错误] 找不到设备: $SOURCE_DEV ${NC}"
    echo "请检查插入是否正确，或使用 ls /dev/tty* 查看。"
    exit 1
fi

# 步骤 B: 获取目标别名
read -p "请输入想要绑定的别名 (不带 /dev/, 例如 stm32): " ALIAS_NAME

# 3. 提取设备信息 (核心步骤)
echo -e "\n正在读取 $SOURCE_DEV 的硬件信息..."

# 使用 udevadm info 获取详细信息
# 我们主要寻找父级 USB 设备的 idVendor, idProduct 和 serial
# head -n 1 确保我们只取最接近的一层（通常是直接挂载的 USB 属性）

VID=$(udevadm info -a -n "$SOURCE_DEV" | grep 'ATTRS{idVendor}' | head -n 1 | awk -F "==" '{print $2}' | sed 's/"//g')
PID=$(udevadm info -a -n "$SOURCE_DEV" | grep 'ATTRS{idProduct}' | head -n 1 | awk -F "==" '{print $2}' | sed 's/"//g')
SERIAL=$(udevadm info -a -n "$SOURCE_DEV" | grep 'ATTRS{serial}' | head -n 1 | awk -F "==" '{print $2}' | sed 's/"//g')

# 检查是否成功获取
if [ -z "$VID" ] || [ -z "$PID" ]; then
    echo -e "${RED}[错误] 无法读取设备的 Vendor ID 或 Product ID。${NC}"
    echo "这可能不是一个标准的 USB 设备。"
    exit 1
fi

echo -e "捕获到的硬件信息:"
echo -e "  Vendor ID : ${GREEN}$VID${NC}"
echo -e "  Product ID: ${GREEN}$PID${NC}"
echo -e "  Serial No : ${GREEN}$SERIAL${NC}"

# 4. 生成规则文件内容
# 规则逻辑：
# 如果有序列号(Serial)，则加入序列号匹配（更精准，防止同型号设备冲突）。
# 如果没有序列号，仅使用 VID 和 PID。
# MODE="0666" 赋予读写权限，免去 sudo 打开串口的麻烦。

RULES_FILE="/etc/udev/rules.d/99-${ALIAS_NAME}.rules"

echo -e "\n正在生成规则文件: $RULES_FILE ..."

if [ -n "$SERIAL" ]; then
    RULE_CONTENT="KERNEL==\"tty*\", SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", ATTRS{serial}==\"$SERIAL\", MODE=\"0666\", SYMLINK+=\"$ALIAS_NAME\""
else
    echo -e "${CYAN}[提示] 该设备没有序列号，将仅通过 VID/PID 匹配。如果有两个完全相同的设备，可能会冲突。${NC}"
    RULE_CONTENT="KERNEL==\"tty*\", SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", MODE=\"0666\", SYMLINK+=\"$ALIAS_NAME\""
fi

echo "$RULE_CONTENT" > "$RULES_FILE"

# 5. 重载规则
echo -e "正在重载 Udev 规则..."
udevadm control --reload-rules
udevadm trigger

# 6. 验证
sleep 1 # 等一秒让系统反应
if [ -e "/dev/$ALIAS_NAME" ]; then
    echo -e "\n${GREEN}[成功] 映射已建立！${NC}"
    ls -l "/dev/$ALIAS_NAME"
else
    echo -e "\n${RED}[警告] 规则已生成，但暂未检测到 /dev/$ALIAS_NAME。${NC}"
    echo "请尝试重新拔插设备。"
fi