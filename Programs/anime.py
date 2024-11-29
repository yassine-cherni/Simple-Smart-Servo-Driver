import servo
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# PID Gain
KP = 80
KI = 0.1
KD = 600

# Servo offset
adcMAX = 2900.0
adcMIN = 300.0
angleMAX = 90
angleMIN = -90

# Maximum servo output (0~999)
MaxOutput = 999

# Low-pass filter coefficient for sensor (0~1)
alpha = 1

# Low-pass filter coefficient for servo output (0~1)
beta = 1

# Instantiate the servo
Servo = servo.ServoP(0x10)

# Configure the servo
Servo.set_PID(KP, KI, KD)

# Calibrate the servo
Servo.CalibrateADCMotorAngle(angleMIN, angleMAX, adcMIN, adcMAX)

# Set the maximum output for the servo
Servo.set_MaxOutput(MaxOutput)

# Set the low-pass filter coefficient for sensor values of each servo
Servo.set_lowpass_filter_alpha(alpha)

# Set the low-pass filter coefficient for the sensor values of the servo
Servo.set_lowpass_filter_beta(beta)

# Set the current angle of the servo to the initial target angle
Servo.set_Init()



# リアルタイムプロット用の設定
# プロットするデータの保持数
max_points = 1000
angles = deque([0]*max_points, maxlen=max_points)
timestamps = deque([0]*max_points, maxlen=max_points)

# プロットの初期設定
fig, ax = plt.subplots()
line, = ax.plot(timestamps, angles, lw=2)
ax.set_ylim(angleMIN - 10, angleMAX + 10)
ax.set_xlim(0, max_points)
ax.set_xlabel('Time (arbitrary units)')
ax.set_ylabel('Angle (degrees)')
ax.set_title('Real-Time Servo Angle')

# 更新関数
def update(frame):
    # 現在の角度を取得
    NowAngle = Servo.get_Angle()
    # 時間（ポイント数）を追加
    if len(timestamps) == 0:
        current_time = 0
    else:
        current_time = timestamps[-1] + 1
    timestamps.append(current_time)
    angles.append(NowAngle)
    
    # データを更新
    line.set_data(timestamps, angles)
    ax.set_xlim(max(0, current_time - max_points), current_time)
    return line,

# アニメーションの設定
ani = animation.FuncAnimation(fig, update, blit=True, interval=10)

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    # プログラム終了時にサーボを停止
    Servo.motor_off()
    Servo.close_servo()
