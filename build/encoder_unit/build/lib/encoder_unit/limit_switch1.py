import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Button 

class LimitSwitchNode(Node):
    def __init__(self):
        super().__init__('limit_switch1')
        
        # Mapping ขาตาม Diagram เดิม
        self.limit_pins = {
            'Limit_1': 18,
            'Limit_2': 23,
            'Limit_3': 24,
            'Limit_4': 25,
            'Limit_5': 16
        }
        
        self.switches = {}

        for name, pin in self.limit_pins.items():
            # *** แก้ไขตรงนี้ครับ ***
            # pull_up=False หมายถึง ใช้ Internal Pull-down Resistor (รอรับไฟ 3.3V)
            # active_state=True (กดแล้วเป็น High)
            btn = Button(pin, pull_up=False, bounce_time=0.1)
            
            btn.when_pressed = lambda n=name: self.on_switch_press(n)
            btn.when_released = lambda n=name: self.on_switch_release(n)
            
            self.switches[name] = btn
            self.get_logger().info(f'Setup {name} on GPIO {pin} (Active High / 3.3V Input)')

    def on_switch_press(self, switch_name):
        # เมื่อกดสวิตช์ ไฟ 3.3V วิ่งเข้า -> สถานะเป็น High
        msg = f'{switch_name} : PRESSED (Input 3.3V)'
        self.get_logger().info(msg)

    def on_switch_release(self, switch_name):
        # เมื่อปล่อยสวิตช์ ตัวต้านทาน Pull-down ดึงลง GND -> สถานะเป็น Low
        msg = f'{switch_name} : RELEASED'
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LimitSwitchNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()