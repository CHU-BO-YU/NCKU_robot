#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from typing import List

NUM_SERVOS = 12

import sys
import select
import termios
import tty

def get_key(timeout=0.1):
    """非阻塞讀取單一鍵，無輸入則回傳 None"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class ServoTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('servo_trajectory_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/servo_trajectory', 10)
        self.positions = [0.0] * NUM_SERVOS
        self.get_logger().info(f'輸入 q 離開；可控制 1–{NUM_SERVOS} 號舵機')

    def publish_joint_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = [f'servo_{i}' for i in range(1, NUM_SERVOS+1)]
        pt = JointTrajectoryPoint()
        pt.positions = self.positions.copy()
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 0
        msg.points = [pt]
        self.pub.publish(msg)
        self.get_logger().info(f'Published → {self.positions}')

    def run_menu(self):
        try:
            while rclpy.ok():
                s = input(f'\n選擇要控制哪些舵機（逗號分隔，1–{NUM_SERVOS}，q = 離開）：').strip()
                if s.lower() == 'q':
                    break
                try:
                    ids = [int(x) - 1 for x in s.split(',') if x.strip().isdigit()]
                except ValueError:
                    print("格式錯誤，請用逗號分隔有效編號。")
                    continue
                ids = [i for i in ids if 0 <= i < NUM_SERVOS]
                if not ids:
                    print(f"沒有有效編號，請輸入 1 到 {NUM_SERVOS}。")
                    continue
                self.angle_menu(ids)
        except (KeyboardInterrupt, EOFError):
            pass

    def angle_menu(self, indices: List[int]):
        if len(indices) != 2:
            print("❌ 此功能目前僅支援兩顆舵機控制。請選擇兩個舵機。")
            return

        ids_str = ", ".join(str(i + 1) for i in indices)
        print(f'🎮 控制舵機 {ids_str}，使用鍵盤 WASD 設定角度，Q 離開')

        try:
            while rclpy.ok():
                key = get_key()
                if key is None:
                    continue

                key = key.lower()
                if key == 'w':
                    self.positions[indices[0]] = 240.0
                    self.positions[indices[1]] = 0.0
                    self.publish_joint_trajectory()
                    print(f"🠕 W：{indices[0]+1}=240, {indices[1]+1}=0")
                elif key == 'a':
                    self.positions[indices[0]] = 240.0
                    self.positions[indices[1]] = 240.0
                    self.publish_joint_trajectory()
                    print(f"🠔 A：{indices[0]+1}=240, {indices[1]+1}=240")
                elif key == 'd':
                    self.positions[indices[0]] = 0.0
                    self.positions[indices[1]] = 0.0
                    self.publish_joint_trajectory()
                    print(f"🠗 S：{indices[0]+1}=0, {indices[1]+1}=240")
                elif key == 'q':
                    print("🚪 離開控制模式")
                    break
                else:
                    continue  # 忽略其他鍵

        except KeyboardInterrupt:
            print("❗ 中斷輸入")

def main(args=None):
    rclpy.init(args=args)
    node = ServoTrajectoryPublisher()
    node.run_menu()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
