#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

NUM_SERVOS = 12

class ServoTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('servo_trajectory_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/servo_trajectory', 10)
        self.positions = [90.0] * NUM_SERVOS
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
        ids_str = ", ".join(str(i+1) for i in indices)
        prompt = f'為 舵機 {ids_str} 一次設定相同角度（0–240），或輸入 b 返回：'
        while rclpy.ok():
            inp = input(prompt).strip()
            if inp.lower() == 'b':
                return
            try:
                ang = float(inp)
                if not 0.0 <= ang <= 240.0:
                    raise ValueError
            except ValueError:
                print('角度範圍錯誤，請輸入 0–240 之間，或 b 返回。')
                continue
            for idx in indices:
                self.positions[idx] = ang
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
