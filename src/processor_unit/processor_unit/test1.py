#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import math

class IKSolver(Node):
    def __init__(self):
        super().__init__('ik_solver_node')
        
        # --- 1. ตั้งค่าความยาวของแขน (หน่วยเป็นเมตร หรือ หน่วยเดียวกับ x,y,z) ---
        # ปรับแก้ค่าตามขนาดจริงของหุ่นยนต์คุณที่นี่
        self.L1 = 90.0  # ความสูงจากฐานถึงจุดหมุนไหล่ (Joint 2)
        self.L2 = 575.0  # ความยาวต้นแขน (Joint 2 -> 3)
        self.L3 = 485.0  # ความยาวปลายแขน (Joint 3 -> 5/Wrist)
        self.L_END = 190.0 # ความยาวจากข้อมือถึงปลายมือ (End Effector)
        
        # ตั้งค่ามุมที่ต้องการของปลายมือ (Fixed Orientation)
        # Pitch: ก้มเงย (0 คือชี้ระนาบ, -90 คือชี้ลงพื้น)
        self.target_pitch = math.radians(0) 
        # Yaw: ถ้าต้องการให้ Joint 5 ขนานแกน Y ตลอด (ตามที่ถามก่อนหน้า) ให้ใช้ logic ใน code
        self.fixed_yaw_mode = True 

        # --- 2. สร้าง Publisher และ Subscriber ---
        # รับค่า XYZ เป้าหมาย
        self.subscription = self.create_subscription(
            Point,
            'target_position',
            self.ik_callback,
            10)
            
        # ส่งค่ามุม Joint ออกไป
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        self.get_logger().info("IK Solver Node Started. Waiting for XYZ input...")

    def ik_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        
        try:
            joints = self.calculate_ik(x, y, z)
            
            # สร้าง message JointState
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
            joint_msg.position = joints # ส่งค่ามุมเป็น radian
            
            self.publisher_.publish(joint_msg)
            self.get_logger().info(f"Target: ({x:.2f}, {y:.2f}, {z:.2f}) -> Joints: {[round(math.degrees(j), 2) for j in joints]}")
            
        except ValueError as e:
            self.get_logger().error(f"Cannot reach target: {e}")

    def calculate_ik(self, x, y, z):
        # --- ขั้นตอนที่ 1: หาตำแหน่งข้อมือ (Wrist Center) ---
        # เราต้องถอยจากปลายมือ (End Effector) กลับมาหาจุดหมุนข้อมือ (Joint 5)
        # สมมติ Pitch คงที่ และ Yaw หมุนตามแขน (หรือ fix ตาม logic)
        
        # ในที่นี้สมมติ Yaw ของปลายมือ (Global Yaw)
        if self.fixed_yaw_mode:
            global_yaw = math.pi / 2 # 90 องศา (ขนานแกน Y)
        else:
            global_yaw = math.atan2(y, x) # ชี้ไปตามแนวแขนปกติ
            
        # คำนวณพิกัด Wrist (xw, yw, zw)
        # หมายเหตุ: Z ของแขนเริ่มนับจากหัวไหล่ (Joint 2) ดังนั้นต้องลบความสูงฐาน (L1) ออกก่อนถ้าคิดเทียบพื้น
        z_shoulder_frame = z - self.L1
        
        xw = x - self.L_END * math.cos(self.target_pitch) * math.cos(global_yaw)
        yw = y - self.L_END * math.cos(self.target_pitch) * math.sin(global_yaw)
        zw = z_shoulder_frame - self.L_END * math.sin(self.target_pitch)

        # --- ขั้นตอนที่ 2: Top View (หา Theta 1, Theta 5) ---
        theta1 = math.atan2(yw, xw)
        
        if self.fixed_yaw_mode:
            # สูตร: Theta 5 = Global_Yaw - Theta 1
            theta5 = (math.pi / 2) - theta1
        else:
            theta5 = 0.0 # ให้ข้อมือตรงไปตามแขน

        # --- ขั้นตอนที่ 3: Side View (หา Theta 2, 3, 4) ---
        # หาระยะห่างในแนวราบจากฐานถึงข้อมือ
        rw = math.sqrt(xw**2 + yw**2)
        # หาระยะตรงจากหัวไหล่ถึงข้อมือ (D)
        D = math.sqrt(rw**2 + zw**2)
        
        # ตรวจสอบว่าแขนเอื้อมถึงไหม
        if D > (self.L2 + self.L3):
            raise ValueError("Target out of reach")

        # กฎของ Cosine หา Theta 3 (มุมศอก)
        # cos_theta3 = (b^2 + c^2 - a^2) / 2bc
        cos_theta3 = (self.L2**2 + self.L3**2 - D**2) / (2 * self.L2 * self.L3)
        
        # ป้องกัน error จาก floating point (เช่น 1.0000001)
        cos_theta3 = max(-1.0, min(1.0, cos_theta3))
        
        # มุมภายในสามเหลี่ยม (ศอก)
        # ปกติหุ่นยนต์จะมีท่า Elbow Up หรือ Down (ในที่นี้ใช้ Elbow Up - มุมติดลบในบาง config หรือมุมบวก แล้วแต่แกน)
        # สูตรทั่วไป: D^2 = L2^2 + L3^2 - 2*L2*L3*cos(180 - theta3_robot)
        # เพื่อความง่าย ใช้สูตรหามุมภายใน alpha ก่อน
        alpha_triangle = math.acos((self.L2**2 + self.L3**2 - D**2) / (2 * self.L2 * self.L3))
        theta3 = math.pi - alpha_triangle # มุมที่ Servo ต้องขยับจริง (ขึ้นอยู่กับการตั้งศูนย์ servo)
        
        # หมายเหตุ: ถ้าใช้สูตรมาตรฐาน DH หรือเรขาคณิตตรงๆ:
        # theta3 = math.acos((rw**2 + zw**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3))
        # แต่ต้องดูทิศทางการหมุนของ servo จริงๆ อีกทีครับ

        # หา Theta 2 (มุมไหล่)
        # มุมยกระดับของเส้น D
        angle_to_wrist = math.atan2(zw, rw)
        # มุมภายในสามเหลี่ยมชิดไหล่
        angle_shoulder_triangle = math.acos((self.L2**2 + D**2 - self.L3**2) / (2 * self.L2 * D))
        
        theta2 = angle_to_wrist + angle_shoulder_triangle 

        # หา Theta 4 (มุมข้อมือก้มเงย)
        # pitch = theta2 + theta3 + theta4 (โดยประมาณ ขึ้นอยู่กับแกน)
        # หรือถ้าคิดแบบเรขาคณิต: theta4 = target_pitch - (theta2 - (มุมหักศอก))
        # สมมติระบบแกนหมุนไปทางเดียวกันหมด
        theta4 = self.target_pitch - (theta2 - (math.pi - theta3)) 
        # *หมายเหตุ: สมการ theta 2,3,4 อาจต้องปรับเครื่องหมาย +/- ตามการติดตั้ง Servo จริง*

        return [theta1, theta2, theta3, theta4, theta5]

def main(args=None):
    rclpy.init(args=args)
    node = IKSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()