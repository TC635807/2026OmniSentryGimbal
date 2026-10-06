#!/usr/bin/env python3
"""
云台→视觉数据包解码器（43字节）
协议格式：
  head[0] (0x53), head[1] (0x50), mode (uint8_t),
  q[0]~q[3] (float, 四元数 w,x,y,z),
  yaw (float), yaw_vel (float), pitch (float), pitch_vel (float), bullet_speed (float),
  bullet_count (uint16_t, 小端),
  crc16 (uint16_t, 小端) 覆盖 [0,40] 字节
"""

import struct
import math
import re
import sys

# ==================== CRC16 表（与上位机一致） ====================
CRC16_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
    0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
    0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
    0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
    0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78
]

def crc16_calc(data, init=0xFFFF):
    """计算 CRC16 校验值（查表法，初始 0xFFFF）"""
    crc = init
    for byte in data:
        idx = (crc ^ byte) & 0xFF
        crc = (crc >> 8) ^ CRC16_TABLE[idx]
    return crc & 0xFFFF

def parse_hex_input(hex_str):
    """
    将用户输入的十六进制字符串转换为字节串。
    支持格式：
      - 连续字符串："5350010efb7f3f..."
      - 空格分隔："53 50 01 0E FB 7F 3F ..."
      - 带0x前缀："0x53 0x50 0x01 0x0E ..."
      - 逗号分隔："0x53,0x50,0x01,0x0E,..."
      - 混合分隔："53, 50, 0x01, 0E, FB, 7F ..."
    """
    # 使用正则提取所有形如 (可选0x) + 两个十六进制字符 的片段
    pattern = r'(?:0x)?([0-9A-Fa-f]{2})'
    matches = re.findall(pattern, hex_str)
    if not matches:
        raise ValueError("未找到有效的十六进制字节")
    try:
        return bytes(int(byte, 16) for byte in matches)
    except ValueError as e:
        raise ValueError(f"无效的十六进制字符: {e}")

def decode_gimbal_to_vision(packet_bytes, verify_crc=True, verify_header=True):
    """
    解析43字节云台→视觉数据包
    返回字典包含所有字段
    """
    if len(packet_bytes) != 43:
        raise ValueError(f"数据包长度必须为43字节，当前为{len(packet_bytes)}字节")

    if verify_header:
        if packet_bytes[0] != 0x53 or packet_bytes[1] != 0x50:
            raise ValueError(f"帧头错误：应为 0x53 0x50，实际为 0x{packet_bytes[0]:02X} 0x{packet_bytes[1]:02X}")

    # 前41字节（索引0~40）为数据区，最后2字节为CRC
    data_without_crc = packet_bytes[:41]
    packet_crc = struct.unpack('<H', packet_bytes[41:43])[0]

    if verify_crc:
        calc_crc = crc16_calc(data_without_crc)
        if packet_crc != calc_crc:
            raise ValueError(f"CRC校验失败：包中CRC=0x{packet_crc:04X}，计算值=0x{calc_crc:04X}")

    # 解包格式：3字节头 + 9个float + 1个uint16_t (bullet_count)
    # 注意：9个float依次为 q[0],q[1],q[2],q[3], yaw, yaw_vel, pitch, pitch_vel, bullet_speed
    unpacked = struct.unpack('<3B9fH', data_without_crc)
    head0, head1, mode = unpacked[0:3]
    q = list(unpacked[3:7])      # 四元数 w,x,y,z
    yaw, yaw_vel, pitch, pitch_vel, bullet_speed = unpacked[7:12]
    bullet_count = unpacked[12]

    return {
        'head': (head0, head1),
        'mode': mode,
        'q': q,                 # [w, x, y, z]
        'yaw': yaw,
        'yaw_vel': yaw_vel,
        'pitch': pitch,
        'pitch_vel': pitch_vel,
        'bullet_speed': bullet_speed,
        'bullet_count': bullet_count,
        'crc': packet_crc,
        'raw_hex': packet_bytes.hex().upper()
    }

def print_result(decoded):
    """友好打印解码结果"""
    mode_desc = {0: "待机", 1: "跟随", 2: "自瞄", 3: "小陀螺"}  # 根据实际定义修改
    mode_str = mode_desc.get(decoded['mode'], f"未知({decoded['mode']})")

    print("\n========== 云台→视觉 解码结果 ==========")
    print(f"帧头: 0x{decoded['head'][0]:02X} 0x{decoded['head'][1]:02X} (应为 'S' 'P')")
    print(f"模式: {decoded['mode']} ({mode_str})")
    print("\n--- 姿态四元数 ---")
    print(f"  w: {decoded['q'][0]:.6f}")
    print(f"  x: {decoded['q'][1]:.6f}")
    print(f"  y: {decoded['q'][2]:.6f}")
    print(f"  z: {decoded['q'][3]:.6f}")
    print("\n--- 云台角度 ---")
    print(f"Yaw 角度:   {decoded['yaw']:.6f} rad = {math.degrees(decoded['yaw']):.3f}°")
    print(f"Yaw 角速度: {decoded['yaw_vel']:.6f} rad/s")
    print(f"Pitch 角度: {decoded['pitch']:.6f} rad = {math.degrees(decoded['pitch']):.3f}°")
    print(f"Pitch 角速度: {decoded['pitch_vel']:.6f} rad/s")
    print("\n--- 弹道参数 ---")
    print(f"弹速: {decoded['bullet_speed']:.3f} m/s")
    print(f"子弹计数: {decoded['bullet_count']}")
    print(f"CRC16: 0x{decoded['crc']:04X}")
    print("=========================================\n")

def main():
    # 如果命令行提供了参数，将其合并为一个字符串
    if len(sys.argv) > 1:
        hex_input = ' '.join(sys.argv[1:])
    else:
        hex_input = input("请输入43字节的十六进制数据包: ").strip()

    if not hex_input:
        print("错误：未输入任何数据。")
        sys.exit(1)

    try:
        packet_bytes = parse_hex_input(hex_input)
        decoded = decode_gimbal_to_vision(packet_bytes)
        print_result(decoded)
    except (ValueError, struct.error) as e:
        print(f"解码失败: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()