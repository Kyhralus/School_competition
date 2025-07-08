import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty  # 标准空请求服务
from std_msgs.msg import String
import time
from typing import Union, Optional
from periphery import PWM

class Servo:
    """基于periphery PWM库的舵机控制类"""
    def __init__(
        self, 
        pwmchip: int = 0, 
        channel: int = 0, 
        freq: float = 50, 
        pulse_min: float = 0.5, 
        pulse_max: float = 2.5,
        angle_min: float = 0,
        angle_max: float = 180,
        reverse: bool = False
    ):
        self.pwm = PWM(pwmchip, channel)  # 假设PWM类来自periphery库
        self.freq = freq
        self.pulse_min = pulse_min
        self.pulse_max = pulse_max
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.reverse = reverse
        self.period = 1000.0 / freq  # ms
        
        # 初始化PWM
        self.pwm.frequency = freq
        self.pwm.duty_cycle = 0
        self.pwm.enable()
        self._current_angle = angle_min  # 初始角度
    
    def angle_to_duty(self, angle: float) -> float:
        """角度转占空比，包含边界检查和反转逻辑"""
        angle = max(self.angle_min, min(self.angle_max, angle))
        if self.reverse:
            angle = self.angle_max + self.angle_min - angle
        # 线性映射角度到脉宽
        pulse = self.pulse_min + (angle - self.angle_min) * (self.pulse_max - self.pulse_min) / (self.angle_max - self.angle_min)
        return pulse / self.period  # 脉宽/周期 = 占空比
    
    def set_angle(self, angle: float) -> None:
        """设置舵机角度（带边界限制）"""
        angle = max(self.angle_min, min(self.angle_max, angle))  # 限制角度范围
        duty = self.angle_to_duty(angle)
        self.pwm.duty_cycle = duty
        self._current_angle = angle
    
    def get_angle(self) -> float:
        """获取当前角度"""
        return self._current_angle
    
    def close(self) -> None:
        """关闭舵机并释放资源"""
        self.pwm.duty_cycle = 0
        self.pwm.disable()
        self.pwm.close()
        print(f"Servo closed: pwmchip={self.pwm.chip}, channel={self.pwm.channel}")


import time

class PID:
    """增强型PID控制器类，支持位置式和增量式两种模式"""
    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0, 
                 output_limits: tuple = (-5.0, 5.0),  # 支持负值
                 integral_limits: tuple = (None, None),
                 sample_time: float = 0.01,  # 采样时间（秒）
                 mode: str = 'incremental'):  # 默认增量式
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits  # 输出限幅（支持负值）
        self.integral_limits = integral_limits
        self.sample_time = sample_time
        self.mode = mode
        
        # 内部状态
        self.error_sum = 0.0          # 误差累积和
        self.last_error = 0.0         # 上一次误差
        self.prev_error = 0.0         # 上上次误差
        self.last_output = 0.0        # 上一次输出
        self.last_time = time.time()  # 上次计算时间
        self.initialized = False      # 是否初始化标志

    def reset(self) -> None:
        """重置PID控制器状态"""
        self.error_sum = 0.0
        self.last_error = 0.0
        self.prev_error = 0.0
        self.last_output = 0.0
        self.last_time = time.time()
        self.initialized = False

    def compute(self, error: float) -> float:
        """根据控制模式计算PID输出"""
        if self.mode == 'position':
            return self._compute_position(error)
        else:  # 'incremental'
            return self._compute_incremental(error)

    def _compute_position(self, error: float) -> float:
        """位置式PID计算（输出：目标角度的绝对量）"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        # 首次调用或时间差异常时初始化
        if not self.initialized or dt <= 0 or dt > 1.0:
            self.last_time = current_time
            self.initialized = True
            return self.last_output  # 返回上一次输出，避免跳变
        
        # 使用固定采样时间或实际时间差
        if self.sample_time > 0:
            dt = self.sample_time
        
        # 计算积分项并应用积分限幅
        self.error_sum += error * dt
        if self.integral_limits[0] is not None:
            self.error_sum = max(self.integral_limits[0], self.error_sum)
        if self.integral_limits[1] is not None:
            self.error_sum = min(self.integral_limits[1], self.error_sum)
        
        # 计算微分项
        derivative = (error - self.last_error) / dt
        
        # 计算PID输出（目标角度的绝对量）
        output = (self.kp * error + 
                  self.ki * self.error_sum + 
                  self.kd * derivative)
        
        # 应用输出限幅（限制目标角度范围）
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)
        
        # 更新状态
        self.last_error = error
        self.last_time = current_time
        self.last_output = output
        
        return output

    def _compute_incremental(self, error: float) -> float:
        """增量式PID计算（输出：角度的增量，需累加至当前角度）"""
        current_time = time.time()
        dt = current_time - self.last_time
        print("增量式:error:",error)
        # 首次调用或时间差异常时初始化
        if not self.initialized or dt <= 0 or dt > 1.0:
            self.last_time = current_time
            self.last_error = error
            self.initialized = True
            return 0.0  # 首次无增量
        
        # 使用固定采样时间或实际时间差
        if self.sample_time > 0:
            dt = self.sample_time
        
        # 计算误差增量
        error_diff = error - self.last_error  # 当前误差 - 上一次误差
        
        # 计算PID增量（角度的调整量）
        # 使用公式：Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
        delta_output = (self.kp * error_diff + 
                       self.ki * error + 
                       self.kd * (error - 2*self.last_error + self.prev_error))
        print("增量式delta:",delta_output)
        # 应用增量限幅（限制单次调整角度）
        if self.output_limits[0] is not None:
            delta_output = max(self.output_limits[0], delta_output)
        if self.output_limits[1] is not None:
            delta_output = min(self.output_limits[1], delta_output)
        print("增量式输出限幅:",delta_output)
        # 更新状态（无需累积输出，增量由外部累加）
        self.prev_error = self.last_error
        self.last_error = error
        self.last_time = current_time
        
        return delta_output  # 仅返回增量，外部需加至当前角度


class Gimbal:
    """云台控制类 - 整合舵机和PID控制器"""
    def __init__(self, pitch_servo, pitch_pid, yaw_servo, yaw_pid):
        self.pitch_servo = pitch_servo
        self.yaw_servo = yaw_servo
        self.pid_pitch = pitch_pid
        self.pid_yaw = yaw_pid
    
    def update(self, err_pitch: float, err_yaw: float) -> None:
        """更新云台控制（根据PID模式自动适配逻辑）"""
        prev_pitch = self.pitch_servo.get_angle()
        prev_yaw = self.yaw_servo.get_angle()

        # 计算PID输出
        output_pitch = self.pid_pitch.compute(err_pitch)
        output_yaw = self.pid_yaw.compute(err_yaw)
        
        # 根据PID模式更新舵机角度
        if self.pid_pitch.mode == 'incremental':
            # 增量式：输出是角度增量，需累加至当前角度
            new_pitch = self.pitch_servo.get_angle() + output_pitch
            new_yaw = self.yaw_servo.get_angle() + output_yaw
        else:
            # 位置式：输出是目标角度的绝对量，直接设置
            new_pitch = output_pitch
            new_yaw = output_yaw
        
        # 设置舵机角度（带边界限制）
        self.pitch_servo.set_angle(new_pitch)
        self.yaw_servo.set_angle(new_yaw)
        
        # 打印详细控制信息
        # print(
        #     f"修正前角度 - pitch: {prev_pitch:.5f}°, yaw: {prev_yaw:.5f}° | "
        #     f"误差 - pitch: {err_pitch:.5f}, yaw: {err_yaw:.5f} | "
        #     f"修正量 - pitch: {output_pitch:.5f}°, yaw: {output_yaw:.5f}° | "
        #     f"修正后角度 - pitch: {new_pitch:.5f}°, yaw: {new_yaw:.5f}°"
        # )


class GimbalControlNode(Node):
    """ 云台控制节点：提供服务重置角度，接收误差话题控制舵机 """
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
                ('pitch.angle_min', 0.0),
                ('pitch.angle_max', 180.0),
                ('pitch.reverse', True),
                
                ('yaw.pwmchip', 4),
                ('yaw.channel', 0),
                ('yaw.freq', 50.0),
                ('yaw.pulse_min', 0.5),
                ('yaw.pulse_max', 2.5),
                ('yaw.angle_min', 0.0),
                ('yaw.angle_max', 180.0),
                ('yaw.reverse', False),
                
                ('pid.pitch_kp', 0.03),
                ('pid.pitch_ki', 0.01),
                ('pid.pitch_kd', 0.01),
                ('pid.pitch_output_limits', (-1, 1)),
                ('pid.mode', 'incremental'),  # PID模式：'position' 或 'incremental'



                ('pid.yaw_kp', 0.05),
                ('pid.yaw_ki', 0.01),
                ('pid.yaw_kd', 0.01),
                ('pid.yaw_output_limits', (-1.5, 1.5)),
                ('pid.mode', 'incremental'),  # PID模式：'position' 或 'incremental'
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
            kp=self.get_parameter('pid.pitch_kp').value,
            ki=self.get_parameter('pid.pitch_ki').value,
            kd=self.get_parameter('pid.pitch_kd').value,\
            output_limits=(-1, 1),  # 单次最大调整2度
            mode='incremental'
        )
        
        self.pid_yaw = PID(
            kp=self.get_parameter('pid.yaw_kp').value,
            ki=self.get_parameter('pid.yaw_ki').value,
            kd=self.get_parameter('pid.yaw_kd').value,
            output_limits=(-1.5, 1.5),
            mode='incremental'
        )
        
        # 3. 初始位置
        self.pitch_servo.set_angle(100.0)
        self.yaw_servo.set_angle(90.0)
        time.sleep(1)
        self.get_logger().info("Gimbal initialized.")
        
        # 4. 创建服务（新增）
        self.set_service = self.create_service(
            Empty,
            'set_gimbal',
            self.set_callback
        )
        self.reset_service = self.create_service(
            Empty,
            'reset_gimbal',
            self.reset_callback
        )
        
        # 5. 保留订阅（可选，根据需求决定是否保留）
        self.error_sub = self.create_subscription(
            String,
            'gimbal_error',
            self.error_callback,
            10
        )
    
    def reset_callback(self, request, response):
        """ 服务回调：重置云台角度到初始位置 """
        try:
            # 重置角度到初始位置
            self.pitch_servo.set_angle(100.0)
            self.yaw_servo.set_angle(90.0)
            
            # 重置PID控制器
            self.pid_pitch.error_sum = 0.0
            self.pid_pitch.last_error = 0.0
            self.pid_yaw.error_sum = 0.0
            self.pid_yaw.last_error = 0.0
            
            self.get_logger().info("设置角度为100.0, 90.0")
            return response
            
        except Exception as e:
            self.get_logger().error(f"Failed to set gimbal: {e}")
            return response
        
    def set_callback(self, request, response):
        """ 服务回调：重置云台角度到打靶位置 """
        try:
            # 重置角度到初始位置
            self.pitch_servo.set_angle(140.0)
            self.yaw_servo.set_angle(90.0)
            
            # 重置PID控制器
            self.pid_pitch.error_sum = 0.0
            self.pid_pitch.last_error = 0.0
            self.pid_yaw.error_sum = 0.0
            self.pid_yaw.last_error = 0.0
            
            self.get_logger().info("设置舵机角度为140.0, 90.0")
            return response
            
        except Exception as e:
            self.get_logger().error(f"Failed to reset gimbal: {e}")
            return response
    
    def error_callback(self, msg: String) -> None:
        """ 解析String消息并更新云台 """
        try:
            # 假设消息格式为 "pitch_error,yaw_error"
            err_x, err_y = map(float, msg.data.split(','))
            if abs(err_x) <= 5 and abs(err_y) <= 5:
                self.get_logger().info("打中靶心")
                return
            else:
                output_pitch = self.pid_pitch.compute(err_y)
                output_yaw = self.pid_yaw.compute(err_x)
                
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
