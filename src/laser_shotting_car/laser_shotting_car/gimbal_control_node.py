import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from typing import Union, Optional
from periphery import PWM  # 导入periphery的PWM库

class Servo:
    """ 舵机驱动类 """
    def __init__(
        self, 
        pwmchip: int = 0, channel: int = 0, 
        freq: float = 50, 
        pulse_min: float = 0.5, pulse_max: float = 2.5,
        angle_min: float = 0, angle_max: float = 180,
        reverse: bool = False
    ):
        self.pwm = PWM(pwmchip, channel)
        self.freq = freq
        self.pulse_min = pulse_min
        self.pulse_max = pulse_max
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.reverse = reverse
        self.period = 1000.0 / freq
        
        self.pwm.frequency = freq
        self.pwm.duty_cycle = 0
        self.pwm.enable()
        self._current_angle = angle_min
    
    def angle_to_duty(self, angle: float) -> float:
        angle = max(self.angle_min, min(self.angle_max, angle))
        if self.reverse:
            angle = self.angle_max + self.angle_min - angle
        pulse = self.pulse_min + (angle - self.angle_min) * (self.pulse_max - self.pulse_min) / (self.angle_max - self.angle_min)
        return pulse / self.period
    
    def set_angle(self, angle: float) -> None:
        angle = max(self.angle_min, min(self.angle_max, angle))
        duty = self.angle_to_duty(angle)
        self.pwm.duty_cycle = duty
        self._current_angle = angle
    
    def get_angle(self) -> float:
        return self._current_angle
    
    def close(self) -> None:
        self.pwm.duty_cycle = 0
        self.pwm.disable()
        self.pwm.close()


class PID:
    """ PID控制器类 """
    def __init__(self, p: float = 0.0, i: float = 0.0, d: float = 0.0):
        self.p = p
        self.i = i
        self.d = d
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = 0
    
    def compute(self, error: float) -> float:
        current_time = time.time()
        if self.last_time == 0:
            self.last_time = current_time
            return 0
        
        dt = current_time - self.last_time
        dt = max(dt, 0.001)  # 防止除零
        
        self.error_sum += error * dt
        derivative = (error - self.last_error) / dt
        
        self.last_error = error
        self.last_time = current_time
        
        return self.p * error + self.i * self.error_sum + self.d * derivative


class GimbalControlNode(Node):
    """ 云台控制节点：订阅String类型误差话题，控制舵机 """
    def __init__(self):
        super().__init__('gimbal_control_node')
        
        # 1. 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pitch.pwmchip', 3),
                ('pitch.channel', 0),
                ('pitch.freq', 50.0),
                ('pitch.pulse_min', 0.5),
                ('pitch.pulse_max', 2.5),
                ('pitch.angle_min', 30.0),
                ('pitch.angle_max', 150.0),
                ('pitch.reverse', False),
                
                ('yaw.pwmchip', 4),
                ('yaw.channel', 0),
                ('yaw.freq', 50.0),
                ('yaw.pulse_min', 0.5),
                ('yaw.pulse_max', 2.5),
                ('yaw.angle_min', 30.0),
                ('yaw.angle_max', 150.0),
                ('yaw.reverse', False),
                
                ('pid.pitch_p', 0.27),
                ('pid.pitch_i', 0.001),
                ('pid.pitch_d', 0.1),
                ('pid.yaw_p', 0.27),
                ('pid.yaw_i', 0.001),
                ('pid.yaw_d', 0.1),
            ]
        )
        
        # 2. 初始化舵机和PID
        self.pitch_servo = Servo(
            pwmchip=self.get_parameter('pitch.pwmchip').value,
            channel=self.get_parameter('pitch.channel').value,
            freq=self.get_parameter('pitch.freq').value,
            pulse_min=self.get_parameter('pitch.pulse_min').value,
            pulse_max=self.get_parameter('pitch.pulse_max').value,
            angle_min=self.get_parameter('pitch.angle_min').value,
            angle_max=self.get_parameter('pitch.angle_max').value,
            reverse=self.get_parameter('pitch.reverse').value,
        )
        
        self.yaw_servo = Servo(
            pwmchip=self.get_parameter('yaw.pwmchip').value,
            channel=self.get_parameter('yaw.channel').value,
            freq=self.get_parameter('yaw.freq').value,
            pulse_min=self.get_parameter('yaw.pulse_min').value,
            pulse_max=self.get_parameter('yaw.pulse_max').value,
            angle_min=self.get_parameter('yaw.angle_min').value,
            angle_max=self.get_parameter('yaw.angle_max').value,
            reverse=self.get_parameter('yaw.reverse').value,
        )
        
        self.pid_pitch = PID(
            p=self.get_parameter('pid.pitch_p').value,
            i=self.get_parameter('pid.pitch_i').value,
            d=self.get_parameter('pid.pitch_d').value,
        )
        
        self.pid_yaw = PID(
            p=self.get_parameter('pid.yaw_p').value,
            i=self.get_parameter('pid.yaw_i').value,
            d=self.get_parameter('pid.yaw_d').value,
        )
        
        # 3. 初始位置
        self.pitch_servo.set_angle(120.0)
        self.yaw_servo.set_angle(100.0)
        time.sleep(1)
        self.get_logger().info("Gimbal initialized.")
        
        # 4. 订阅String类型误差话题
        self.error_sub = self.create_subscription(
            String,
            'gimbal_error',
            self.error_callback,
            10
        )
    
    def error_callback(self, msg: String) -> None:
        """ 解析String消息并更新云台 """
        try:
            # 假设消息格式为 "pitch_error,yaw_error"
            err_pitch, err_yaw = map(float, msg.data.split(','))
            
            output_pitch = self.pid_pitch.compute(err_pitch)
            output_yaw = self.pid_yaw.compute(err_yaw)
            
            new_pitch = self.pitch_servo.get_angle() + output_pitch
            new_yaw = self.yaw_servo.get_angle() + output_yaw
            
            self.pitch_servo.set_angle(new_pitch)
            self.yaw_servo.set_angle(new_yaw)
            self.get_logger().debug(f"Pitch: {new_pitch:.2f}, yaw: {new_yaw:.2f}")
        
        except ValueError as e:
            self.get_logger().error(f"Failed to parse error message: {e}")
    
    def close(self) -> None:
        """ 释放资源 """
        self.pitch_servo.close()
        self.yaw_servo.close()
        self.get_logger().info("Servos closed.")


def main(args=None):
    rclpy.init(args=args)
    node = GimbalControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()