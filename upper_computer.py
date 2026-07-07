#!/usr/bin/env python3
"""
X型麦轮键盘控制上位机
通过串口 /dev/ttyACM0 发送速度指令到下位机

按键映射：
  W: 前进    S: 后退    A: 左移    D: 右移
  Q: 逆时针  E: 顺时针
  松开按键 → 停止
"""

import sys
import os
import select
import termios
import time

# ==================== 配置 ====================
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE    = 115200
SPEED       = 20       # 运动速度
TIMEOUT     = 0.02     # 无按键判定停止的超时（秒）

# ==================== 按键 → 速度映射 ====================
KEY_MAP = {
    'w': ( SPEED,  0,      0    ),   # 前进
    's': (-SPEED,  0,      0    ),   # 后退
    'a': ( 0,     -SPEED,  0    ),   # 左移
    'd': ( 0,      SPEED,  0    ),   # 右移
    'q': ( 0,      0,      SPEED),   # 逆时针
    'e': ( 0,      0,     -SPEED),   # 顺时针
}


def setup_serial():
    """配置并打开串口"""
    os.system(f"stty -F {SERIAL_PORT} {BAUDRATE} raw -echo 2>/dev/null")
    fd = os.open(SERIAL_PORT, os.O_RDWR | os.O_NOCTTY)
    return fd


def setup_keyboard():
    """设置终端为非阻塞模式（VMIN=0, VTIME=0）"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~(termios.ECHO | termios.ICANON)
    new[6][termios.VMIN] = 0
    new[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, new)
    return old


def restore_keyboard(old):
    """恢复终端设置"""
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)


def drain_stdin():
    """排空 stdin 缓冲区，返回最后一个有效按键，无有效按键返回 None"""
    last_key = None
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            break
        try:
            data = sys.stdin.read(4096)
            if not data:
                break
            for ch in data:
                ch_lower = ch.lower()
                if ch_lower in KEY_MAP:
                    last_key = ch_lower
                elif ch == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt
        except BlockingIOError:
            break
    return last_key


def send_speed(serial_fd, x, y, o):
    """发送速度指令到下位机"""
    cmd = f"x_speed:{x} y_speed:{y} o_speed:{o}\n"
    os.write(serial_fd, cmd.encode())


def main():
    print("=" * 50)
    print("  X型麦轮键盘控制上位机")
    print("  W/S: 前进/后退   A/D: 左移/右移")
    print("  Q: 逆时针旋转    E: 顺时针旋转")
    print("  Ctrl+C 退出")
    print("=" * 50)

    try:
        serial_fd = setup_serial()
        print(f"[OK] 串口 {SERIAL_PORT} 已打开 (波特率 {BAUDRATE})")
    except Exception as e:
        print(f"[ERROR] 无法打开串口: {e}")
        sys.exit(1)

    old_termios = setup_keyboard()
    last_x = last_y = last_o = 0
    active = False

    try:
        while True:
            key = drain_stdin()

            if key is not None:
                x, y, o = KEY_MAP[key]
                if (x, y, o) != (last_x, last_y, last_o) or not active:
                    send_speed(serial_fd, x, y, o)
                    print(f"\r→ x:{x:+3d}  y:{y:+3d}  o:{o:+3d}  按键:{key.upper()}", end='', flush=True)
                    last_x, last_y, last_o = x, y, o
                    active = True
            else:
                if active:
                    send_speed(serial_fd, 0, 0, 0)
                    print("\r→ 停止                    ", end='', flush=True)
                    last_x, last_y, last_o = 0, 0, 0
                    active = False

            time.sleep(TIMEOUT)

    except KeyboardInterrupt:
        pass
    finally:
        send_speed(serial_fd, 0, 0, 0)
        restore_keyboard(old_termios)
        os.close(serial_fd)
        print("\n[OK] 已退出，串口已关闭")


if __name__ == "__main__":
    main()
