"""
    信号线： 黄色
    正极线： 红色
    地线：  棕色
    pwm1:  
    pwm2:
    servo参数：
        PWM频率默认 200Hz
        脉冲宽度 500-2500us ---> 0-180°
    计算：
        占空比 = 脉冲宽度 / 周期
        脉冲宽度 = 占空比 × 周期
        周期 = 1 / 频率
    实际：
        频率f = 时钟频率 /(分频系数 * 周期寄存器的值) = 24000000/(120*1000) = 200 Hz
        周期T = 1/200 = 5ms
        占空比min = 500us / 5ms = 10%
        占空比max = 2500us / 5ms = 50%
    PWM3_M2  ---- pwmchip1
    PWM14_M1 ---- pwmchip3
    PWM15_M2 ---- pwmchip4


    lfd-01m 50Hz 驱动
        占空比 0.075 ~ 0.125
        角度 0 ~ 180
"""


from periphery import PWM
import time

class Servo:
    def __init__(self, pwmchip=1, channel=0, freq=50, pulse_min=0.5, pulse_max=2.5):
        """
        pwmchip: PWM控制器编号
        channel: PWM通道编号
        freq: PWM频率(Hz)，舵机常用50Hz
        pulse_min: 最小脉宽(ms)，对应0°
        pulse_max: 最大脉宽(ms)，对应180°
        """
        self.pwm = PWM(pwmchip, channel)
        self.freq = freq
        self.pulse_min = pulse_min
        self.pulse_max = pulse_max
        self.period = 1000.0 / freq  # ms
        self.pwm.frequency = freq
        self.pwm.polarity = "normal"
        self.pwm.enable()

    def angle_to_duty(self, angle):
        # 角度转占空比
        pulse = self.pulse_min + (angle / 180.0) * (self.pulse_max - self.pulse_min)
        return pulse / self.period

    def set_angle(self, angle):
        duty = self.angle_to_duty(angle)
        self.pwm.duty_cycle = duty

    def close(self):
        self.pwm.duty_cycle = 0
        self.pwm.close()

if __name__ == "__main__":
    servo = Servo(pwmchip=4, channel=0, freq=50)  # 根据实际pwmchip和通道修改
    print("初始化水平舵机")
    try:
        for angle in range(0, 181, 30):
            servo.set_angle(angle)
            print(f"Set angle: {angle}")
            time.sleep(1)
        for angle in range(180, -1, -30):
            servo.set_angle(angle)
            print(f"Set angle: {angle}")
            time.sleep(1)
    finally:
        servo.close()