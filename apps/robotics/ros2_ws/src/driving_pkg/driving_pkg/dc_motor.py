from adafruit_pca9685 import PCA9685
import board
import busio
import time

class MotorHatB:
    def __init__(self, pwm, in1, in2, ena):
        self.pwm = pwm
        self.in1 = in1
        self.in2 = in2
        self.ena = ena
        self.pwm.frequency = 60

    def set_throttle(self, speed):  # -1.0 ~ 1.0
        pulse = int(0xFFFF * abs(speed))
        if speed > 0:
            self.pwm.channels[self.in1].duty_cycle = 0xFFFF
            self.pwm.channels[self.in2].duty_cycle = 0
        elif speed < 0:
            self.pwm.channels[self.in1].duty_cycle = 0
            self.pwm.channels[self.in2].duty_cycle = 0xFFFF
        else:
            self.pwm.channels[self.in1].duty_cycle = 0
            self.pwm.channels[self.in2].duty_cycle = 0

        self.pwm.channels[self.ena].duty_cycle = pulse

# I2C 설정
i2c = busio.I2C(board.SCL, board.SDA)
pca_throttle = PCA9685(i2c, address=0x40)

# TB6612FNG에 연결된 채널 지정 (예: IN1: 채널 3, IN2: 채널 4, ENA: 채널 5)
motor = MotorHatB(pca_throttle, in1=3, in2=4, ena=5)

try:
    print("전진")
    motor.set_throttle(0.5)
    time.sleep(3)

    print("후진")
    motor.set_throttle(-0.5)
    time.sleep(3)

    print("정지")
    motor.set_throttle(0)

finally:
    pca_throttle.deinit()
