#!/bin/bash

# ==========================================
# 自动化测试脚本：虚拟串口 + GTest
# ==========================================

# 1. 定义一个“清理函数”
# 作用：无论测试成功还是失败，或者是你中途按了 Ctrl+C，都要把后台的 socat 杀掉
cleanup() {
    echo "停止 socat 隧道 (PID: $SOCAT_PID)..."
    kill $SOCAT_PID 2>/dev/null
    exit
}

# 2. 注册信号陷阱 (Trap)
# 告诉系统：当脚本退出(EXIT)或被中断(INT/TERM)时，立刻执行 cleanup 函数
trap cleanup EXIT INT TERM

# 3. 启动 socat 并丢到后台
echo "正在创建虚拟串口隧道 (/tmp/vtty0 <-> /tmp/vtty1)..."
# 使用 & 符号让命令在后台运行
socat -d -d PTY,link=/tmp/vtty0,raw,echo=0 PTY,link=/tmp/vtty1,raw,echo=0