# Untitled - By: 86178 - 周一 5月 27 2024

from Maix import MIC_ARRAY as mic  # 导入麦克风阵列库
import lcd, image
import math
from Maix import GPIO
from fpioa_manager import fm  # 可编程门阵列
from machine import Timer, PWM
import time

# 低通滤波器参数
fc = 1.05    # 截止频率
Ts = 0.02   # 采样周期
b = 2.0 * math.pi * fc * Ts
alpha = b / (b + 1)
previous_angle = 0

def low_pass_filter(current_value, previous_value, alpha):
    return alpha * current_value + (1 - alpha) * previous_value

mic.init(i2s_d0=34, i2s_d1=8, i2s_d2=33, i2s_d3=9, i2s_ws=32, i2s_sclk=10, sk9822_dat=7, sk9822_clk=35)  # 默认配置，需要根据自己接板子上的管脚号来进行修改mic.init()中参数
lcd.init()  # 320*240，初始化LCD
img = image.Image()  # 返回一个图像

tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)  # 定时器
S1 = PWM(tim, freq=50, duty=0, pin=17)

fm.register(0, fm.fpioa.GPIOHS0)  # 激光灯

# 假设按钮1连接到GPIO2，按钮2连接到GPIO3
fm.register(20, fm.fpioa.GPIOHS1)
fm.register(21, fm.fpioa.GPIOHS2)

# 初始化引脚
pin = GPIO(GPIO.GPIOHS0, GPIO.OUT)
button1 = GPIO(GPIO.GPIOHS1, GPIO.IN)
button2 = GPIO(GPIO.GPIOHS2, GPIO.IN)

mode = 0  # 初始模式为mode0
debounce_time = 0.1  # 防抖动时间，单位为秒

PreAngle = 0


def Servo(servo, angle):
    # 将角度限制在-90°到90°范围内
    angle = max(min(angle, 90), -90)
    # 计算对应的占空比（范围为5到10）
    S1.duty((angle + 90) / 180 * 10 + 2.5)

def get_mic_dir():
    global PreAngle, previous_angle, Angle
    AngleX = 0
    AngleY = 0
    AngleAddPi = 0

    mic_list = []
    image = mic.get_map()  # 获取声音源分布图像(返回声源黑白位图)
    b = mic.get_dir(image)  # 计算、获取声源方向(从声源位图计算声源方向 返回12个强度值 对应12个LED灯)

    for i in range(len(b)):
        if b[i] >= 2:
            AngleX += b[i] * math.sin(i * math.pi / 6)
            AngleY += b[i] * math.cos(i * math.pi / 6)
    AngleX = round(AngleX, 6)  # 计算坐标转换值
    AngleY = round(AngleY, 6)
    if AngleY < 0:
        AngleAddPi = 180
    if AngleX < 0 and AngleY > 0:
        AngleAddPi = 360
    if AngleX != 0 or AngleY != 0:  # 参数修正
        if AngleY == 0:
            Angle = 90 if AngleX > 0 else 270  # 填补X轴角度
        else:
            Angle = AngleAddPi + round(math.degrees(math.atan(AngleX / AngleY)), 4)  # 计算角度

        if (0 < Angle < 35) or (325 < Angle < 360):
            if 325 < Angle < 360:  # 处理315-360的角度值
                Angle = Angle - 360

            # 低通滤波
            Angle = low_pass_filter(Angle, previous_angle, alpha)
            previous_angle = Angle

            Servo(S1, -Angle)  # 控制舵机

            PreAngle = Angle

            lcd.draw_string(60, 75, "a(angle): " + str(-Angle), lcd.RED, lcd.BLACK)
            lcd.draw_string(60, 150, "L(distance): " + str(275 / math.cos(-Angle * math.pi / 180)), lcd.RED, lcd.BLACK)
            lcd.fill_rectangle(281, 10, 25, 225, (0, 0, 0))  # 清空右边的区域
            try:
                lcd.fill_rectangle(281, int(112 + math.tan(Angle * math.pi / 180) * 192), 15, 15, (0, 255, 200))  # 角度位置实时
            except:
                pass
        AngleR = round(math.sqrt(AngleY * AngleY + AngleX * AngleX), 4)  # 计算强度
        mic_list.append(AngleX)  # X坐标
        mic_list.append(AngleY)  # Y坐标
        mic_list.append(AngleR)  # 强度
        mic_list.append(Angle)  # 角度

    mic.set_led(b, (10, 10, 0))  # 配置 RGB LED 颜色值   （从计算的声源方向设置点亮对应的LED灯）

    return mic_list  # 返回列表，X坐标，Y坐标，强度，角度

lcd.fill_rectangle(46, 5, 260, 4, (255, 0, 255))  # 上边线
lcd.fill_rectangle(46, 5, 4, 230, (255, 0, 255))  # 左边线
lcd.fill_rectangle(46, 235, 260, 4, (255, 0, 255))  # 下边线
lcd.fill_rectangle(306, 5, 4, 234, (255, 0, 255))  # 右边线
lcd.fill_rectangle(267, 5, 4, 234, (255, 0, 255))  # 右边线2
lcd.fill_rectangle(40, 115, 10, 10, (255, 0, 255))  # 起点坐标框
lcd.fill_rectangle(0, 77, 47, 4, (255, 0, 255))  # 左区域上边线
lcd.fill_rectangle(0, 154, 47, 4, (255, 0, 255))  # 左区域下边线
lcd.fill_rectangle(0, 77, 4, 77, (255, 0, 255))  # 左区域左边线

while True:
    get_mic_dir()
    if not button1.value():  # 如果按钮1被按下
        time.sleep(debounce_time)  # 防抖动
        if not button1.value():  # 再次检测按钮1的状态
            mode = 1
            print("进入定位模式")
            pin.value(0)
            start_time = time.time()  # 获取当前时间
            while time.time() - start_time < 6:  # 循环运行6秒
                get_mic_dir()  # 不间断运行 get_mic_dir()
                time.sleep_ms(20)  # 添加一个小的延时，防止CPU占用过高
            while True:
                if 6 <= time.time() - start_time < 8.5:
                    if abs(Angle - previous_angle) <= 1:
                        pin.value(1)
                        print('提前定位')
                        time.sleep(5)
                        break
                elif time.time() - start_time > 9:
                    pin.value(1)
                    print('强制定位')
                    time.sleep(5)
                    break  # 如果需要在执行后退出循环，加上这一行

            while not button1.value():  # 等待按钮1释放
                pass
    elif not button2.value():  # 如果按钮2被按下
        time.sleep(debounce_time)  # 防抖动
        if not button2.value():  # 再次检测按钮2的状态
            mode = 2
            print("进入追踪模式")
            pin.value(1)
            while not button2.value():  # 等待按钮2释放
                pass
    else:
        mode = 0
    time.sleep_ms(20)
