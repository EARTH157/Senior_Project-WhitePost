#!/usr/bin/env python3
import time
import math
from smbus2 import SMBus
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

# ---------- Constants ----------
PCA_ADDR    = 0x40      # Address ของบอร์ด Servo
MUX_ADDR    = 0x70      # Address ของ Multiplexer
MODE1       = 0x00
PRESCALE    = 0xFE
LED0_ON_L   = 0x06

class ServoMuxNode(Node):
    def __init__(self):
        super().__init__('servo_mux_node')

        # --- Parameters ---
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('default_mux_channel', 0) # ช่อง Mux เริ่มต้น
        
        self.bus_num = self.get_parameter('i2c_bus').value
        self.freq = self.get_parameter('frequency').value
        self.default_mux = self.get_parameter('default_mux_channel').value

        # --- State ---
        self.i2c = SMBus(self.bus_num)
        self.enabled = True
        self.current_mux = -1
        # เก็บรายการช่อง Mux ที่เรา Init PCA9685 ไปแล้ว (จะได้ไม่ต้อง Init ซ้ำ)
        self.initialized_muxs = set() 

        self.get_logger().info(f"Servo Driver Ready on Bus {self.bus_num}, Default Mux: {self.default_mux}")
        
        # Init ช่อง Default ก่อนเลย เพื่อความพร้อม
        self.activate_driver(self.default_mux)

        # --- Subscribers ---
        # รับข้อมูล [mux, ch, angle] หรือ [ch, angle]
        self.create_subscription(Float32MultiArray, '/servo/set_angle', self.cb_set_angle, 10)
        self.create_subscription(Bool, '/servo/enable', self.cb_enable, 10)

    # ---------- Low Level Hardware Control ----------
    def switch_mux(self, channel):
        """สลับช่อง Multiplexer (0-7)"""
        if channel == self.current_mux:
            return # อยู่ช่องเดิมไม่ต้องสลับ
        
        try:
            # เขียนบิตเพื่อเปิดช่อง (1 << channel)
            self.i2c.write_byte(MUX_ADDR, 1 << channel)
            self.current_mux = channel
            # time.sleep(0.005) # รอแป๊บหนึ่งให้สัญญาณนิ่ง (ถ้าจำเป็น)
        except Exception as e:
            self.get_logger().error(f"Failed to switch Mux to {channel}: {e}")

    def init_pca9685(self):
        """ตั้งค่า PCA9685 (Reset & Set Hz)"""
        try:
            # 1. Reset
            self.i2c.write_byte_data(PCA_ADDR, MODE1, 0x00)
            
            # 2. Set Frequency
            prescale_val = 25_000_000.0 / (4096.0 * self.freq) - 1.0
            prescale = int(round(prescale_val))
            
            oldmode = self.i2c.read_byte_data(PCA_ADDR, MODE1)
            newmode = (oldmode & 0x7F) | 0x10 # Sleep mode
            
            self.i2c.write_byte_data(PCA_ADDR, MODE1, newmode) # Go to sleep
            self.i2c.write_byte_data(PCA_ADDR, PRESCALE, prescale) # Set prescale
            self.i2c.write_byte_data(PCA_ADDR, MODE1, oldmode) # Wake up
            
            time.sleep(0.005)
            self.i2c.write_byte_data(PCA_ADDR, MODE1, oldmode | 0xA1) # Auto increment + Restart
            
            self.get_logger().info(f"Initialized PCA9685 on Mux Ch {self.current_mux}")
        except Exception as e:
            self.get_logger().error(f"Failed to Init PCA: {e}")

    def activate_driver(self, mux_ch):
        """เตรียมพร้อมใช้งาน Driver บนช่อง Mux นั้นๆ"""
        self.switch_mux(mux_ch)
        if mux_ch not in self.initialized_muxs:
            self.init_pca9685()
            self.initialized_muxs.add(mux_ch)

    def set_pwm(self, channel, on, off):
        """เขียนค่า PWM ลง Register"""
        try:
            base_reg = LED0_ON_L + 4 * channel
            data = [
                on & 0xFF, (on >> 8) & 0x0F,
                off & 0xFF, (off >> 8) & 0x0F
            ]
            self.i2c.write_i2c_block_data(PCA_ADDR, base_reg, data)
        except Exception as e:
            self.get_logger().error(f"I2C Write Error: {e}")

    # ---------- Logic & Calculation ----------
    def angle_to_pulse(self, angle):
        """แปลงมุม (0-180) เป็นค่า Pulse (0-4095)"""
        # ปรับค่า min/max ตรงนี้ตามสเปค Servo (เช่น 0.5ms - 2.5ms)
        min_pulse = 150  # ~0.7ms (ที่ 50Hz)
        max_pulse = 600  # ~2.6ms (ที่ 50Hz)
        
        # Clamp มุมให้อยู่ในช่วง 0-180
        angle = max(0.0, min(180.0, angle))
        
        # Map linear
        pulse = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
        return int(pulse)

    # ---------- Callbacks ----------
    def cb_enable(self, msg):
        self.enabled = msg.data
        if not self.enabled:
            # ถ้า Disable ให้วนปิดทุก Mux ที่เคยเปิด
            self.get_logger().info("Disabling Servos...")
            for m in list(self.initialized_muxs):
                self.switch_mux(m)
                for i in range(16):
                    self.set_pwm(i, 0, 0)
    
    def cb_set_angle(self, msg):
        if not self.enabled: return

        data = msg.data
        
        # กรณี 1: ส่งมา [ch, angle] -> ใช้ Mux Default
        if len(data) == 2:
            target_mux = self.default_mux
            servo_ch = int(data[0])
            angle = float(data[1])
            
        # กรณี 2: ส่งมา [mux, ch, angle] -> ระบุ Mux เอง
        elif len(data) == 3:
            target_mux = int(data[0])
            servo_ch = int(data[1])
            angle = float(data[2])
        else:
            return # ข้อมูลผิดฟอร์ม

        # สลับ Mux และตรวจสอบว่า Init หรือยัง
        self.activate_driver(target_mux)
        
        # คำนวณและสั่งงาน
        pulse = self.angle_to_pulse(angle)
        self.set_pwm(servo_ch, 0, pulse)
        self.get_logger().info(f"Mux:{target_mux} Ch:{servo_ch} -> {angle:.1f} deg")

    def destroy_node(self):
        self.i2c.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()