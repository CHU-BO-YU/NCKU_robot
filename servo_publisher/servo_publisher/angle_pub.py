#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from typing import List

NUM_SERVOS = 12

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
        ids_str = ", ".join(str(i + 1) for i in indices)
        prompt = (
            f'為 舵機 {ids_str} 一次設定相同角度（0–240），或輸入：\n'
            f'  b 返回\n'
            f'  u 設定 servo_2=240, servo_6=0\n'
            f'  j 設定 servo_2=0, servo_6=240\n'
            f'輸入指令或角度：'
        )

        while rclpy.ok():
            inp = input(prompt).strip().lower()
            if inp == 'b':
                return
            elif inp == 'u':
                self.positions[1] = 240.0  # servo_2（index=1）
                self.positions[5] = 0.0    # servo_6（index=5）
                self.publish_joint_trajectory()
                print("✅ servo_2=240, servo_6=0 已送出")
                continue
            elif inp == 'j':
                self.positions[1] = 0.0
                self.positions[5] = 240.0
                self.publish_joint_trajectory()
                print("✅ servo_2=0, servo_6=240 已送出")
                continue

            # 原本角度設定流程
            try:
                ang = float(inp)
                if not 0.0 <= ang <= 240.0:
                    raise ValueError
            except ValueError:
                print('❌ 輸入錯誤，請輸入 0–240 角度，或 b/u/j 等指令。')
                continue

            for i, idx in enumerate(indices):
                self.positions[idx] = ang if i == 0 else 240.0 - ang
            self.publish_joint_trajectory()
            return

def main(args=None):
    rclpy.init(args=args)
    node = ServoTrajectoryPublisher()
    node.run_menu()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
