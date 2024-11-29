import smbus2
import struct
import time

#ポテンショメータのサーボ
class ServoP:
    def __init__(self, slave_addr):
        self.slave_addr = slave_addr
        self.bus = smbus2.SMBus(1) #ラズパイのI2Cバス(通常は1)
        self.data_length = 5
        self.delay_time = 0.01
        self.accelerationTimeRatio = 0.2
        
    #データをスレーブに送る
    def I2C_send(self, data_addr, data):
        data_bytes = struct.pack('<f', data)
        data_to_send = list(data_bytes)
        self.bus.write_i2c_block_data(self.slave_addr, data_addr, data_to_send)

    #データをスレーブから受け取る
    def I2C_get(self, data_addr):
        received_data = self.bus.read_i2c_block_data(self.slave_addr, data_addr, self.data_length)
        get_bytes = bytes(received_data[1:])
        get_data = struct.unpack('<f', get_bytes)[0]
        return get_data
    
    #モータをオンにする
    def motor_on(self):
        self.I2C_send(0x01, 0)

    #モータをオフにする
    def motor_off(self):
        self.I2C_send(0x02, 0)

    #PIDゲインを設定
    def set_PID(self, Kp, Ki, Kd):
        self.I2C_send(0x03, Kp)
        time.sleep(self.delay_time)
        self.I2C_send(0x04, Kd)
        time.sleep(self.delay_time)
        self.I2C_send(0x05, Ki)
        time.sleep(self.delay_time)

    #ADCと機械角のキャリブレーション 
    def CalibrateADCMotorAngle(self, angleMin, angleMax, adcMin, adcMax):
        self.I2C_send(0x06, angleMax)
        time.sleep(self.delay_time)
        self.I2C_send(0x07, angleMin)
        time.sleep(self.delay_time)
        self.I2C_send(0x08, adcMin)
        time.sleep(self.delay_time)
        self.I2C_send(0x09, adcMax)
        time.sleep(self.delay_time)
    
    #最大出力の設定(0~1000)
    def set_MaxOutput(self, MaxOutput):
        self.I2C_send(0x0a, MaxOutput)
        time.sleep(self.delay_time)

    #ローパスフィルタの係数を設定(0~1)(センサ)
    def set_lowpass_filter_alpha(self, alpha):
        self.I2C_send(0x0b, alpha)
        time.sleep(self.delay_time)

    #ローパスフィルタの係数を設定(0~1)(出力)
    def set_lowpass_filter_beta(self, beta):
        self.I2C_send(0x0c, beta)
        time.sleep(self.delay_time)

    #現在角度の取得
    def get_Angle(self):
        self.I2C_send(0x11, 0)
        time.sleep(self.delay_time)
        angle = self.I2C_get(0x11)
        return angle
    
    #現在角速度の取得
    def get_Velocity(self):
        self.I2C_send(0x12, 0)
        time.sleep(self.delay_time)
        velocity = self.I2C_get(0x12)
        return velocity

    #現在出力の取得
    def get_Output(self):
        self.I2C_send(0x13, 0)
        time.sleep(self.delay_time)
        output = self.I2C_get(0x13)
        return output

    #ADC取得
    def get_ADC(self):
        self.I2C_send(0x14, 0)
        time.sleep(self.delay_time)
        value = self.I2C_get(0x14)
        return value

    #目標角を指定
    def set_GoalAngle(self, goal_angle):
        self.I2C_send(0x20, goal_angle)
    
    #速度台形則に基づ得くパラメータ設計
    def calc_s_LSPB(self, movement_time):
        time_steps = [i * self.delay_time for i in range(int(movement_time / self.delay_time) + 1)]
        parameter_s_steps = []

        tb = movement_time*self.accelerationTimeRatio
        VM = 1/(movement_time - tb)
        a = VM/tb

        #区間に応じて
        for t in time_steps:

            if 0 <= t < tb:
                s = (a*(t**2))/2
                parameter_s_steps.append(s)

            elif tb <= t < movement_time - tb:
                s = VM*(t - (tb/2))
                parameter_s_steps.append(s)

            elif movement_time - tb <= t <= movement_time:
                s = 1 - (a*((movement_time - t)**2))/2
                parameter_s_steps.append(s)

        return parameter_s_steps

    #普通の軌道設計
    def calc_s_normal(self, movement_time):
        time_steps = [i * self.commandCycle for i in range(int(movement_time / self.commandCycle) + 1)]
        parameter_s_steps = []
        for t in time_steps:
            s = t/movement_time
            parameter_s_steps.append(s)

        return parameter_s_steps

#サーボを速度制御用の角度軌道を生成
    def createAngleTrajectory(self, goal_angle_deg, movement_time, LSPB_ON):
        angle_steps = []

        start_angle_deg = self.get_Angle()

        #速度台形則による移動
        if(LSPB_ON == 1):
            parameter_s_steps_theta = self.calc_s_LSPB(movement_time)
            for s in parameter_s_steps_theta:
                angle_deg = start_angle_deg*(1 - s) + goal_angle_deg*s
                angle_steps.append(angle_deg)
        
        #普通の移動
        else:
            parameter_s_steps_theta = self.calc_s_normal(movement_time)
            for s in parameter_s_steps_theta:
                angle_deg = start_angle_deg*(1 - s) + goal_angle_deg*s
                angle_steps.append(angle_deg)
        
        return angle_steps

    #サーボを速度制御
    def move_to_Target_angle(self, goal_angle, movement_time):
        angle_steps = self.createAngleTrajectory(goal_angle, movement_time, 1)
        for angle in angle_steps:
            self.set_GoalAngle(angle)
            time.sleep(self.delay_time)


  #サーボの角度を初期化
    def set_Init(self):
        init_angle = self.get_Angle()
        time.sleep(self.delay_time)
        self.set_GoalAngle(init_angle)
        time.sleep(self.delay_time)

    #サーボを終了する
    def close_servo(self):
        self.motor_off()
        self.bus.close()



