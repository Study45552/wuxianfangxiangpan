#!/usr/bin/python
#-*- coding:UTF-8 -*-
import math

# Chassis类（底盘类）控制车辆底盘的运动和设备状态
class Chassis(object):
    # 初始化函数
    def __init__(self, gear, velocity, steering, Brake, enable, headlamp, lamp, speaker):
        self.gear = gear              # 目标挡位
        self.velocity = velocity      # 目标车体速度
        self.steering = steering      # 目标车体转向角
        self.Brake = Brake            # 目标车辆制动 

        self.enable = enable          # I/O控制使能
        self.headlamp = headlamp      # 远光灯开关
        self.lamp = lamp              # 转向灯开关
        self.speaker = speaker        # 喇叭开关
        self.speedlimit = 0           # 速度挡位 0~3
        self.flag = 0


    # 底盘控制方法
    def ctrl_cmd(self, ctrl_cmd_data):
        """将底盘参数写入ctrl_cmd_data控制对象数据结构中"""
        ctrl_cmd_data.ctrl_cmd_gear = self.gear
        ctrl_cmd_data.ctrl_cmd_velocity = self.velocity
        ctrl_cmd_data.ctrl_cmd_steering = self.steering
        ctrl_cmd_data.ctrl_cmd_Brake = self.Brake
        return ctrl_cmd_data

    # io控制
    def io_cmd(self, io_cmd_data):
        """将设备状态(灯光、喇叭等)填充到io_cmd_data对象中,用于生成 IO 控制命令"""
        io_cmd_data.io_cmd_enable = self.enable
        io_cmd_data.io_cmd_upper_beam_headlamp = self.headlamp
        io_cmd_data.io_cmd_turn_lamp = self.lamp
        io_cmd_data.io_cmd_speaker = self.speaker
        return io_cmd_data

    # 处理速度数据(输入的油门值需根据限速规则转换为实际车速)
    def handle_veplocity(self, velocity):
        return float(velocity)*((self.speedlimit +1)/3.0)*1.1
        
    # 处理转向数据（转换为实际转向角度）
    def handle_steering(self, steering):
        # return (math.sin(float(steering)-1))*24.75
        return (float(steering)-1)*24.75
    
    # 处理转向灯逻辑（左右转向互斥不能同时开启），并管理转向灯的开关状态。
    def handle_lamp(self, left, right, turn_lamp):
        lamp = self.lamp
        if(turn_lamp == 0 and left == 1):
            turn_lamp = 1
            if(self.lamp == 0 or self.lamp == 2):
                lamp = 1
            elif(self.lamp == 1):
                lamp = 0
        elif(turn_lamp == 1 and left == 0 and right == 0):
            turn_lamp = 0
        elif(turn_lamp == 0 and right == 1):
            turn_lamp = 1
            if(self.lamp == 0 or self.lamp == 1):
                lamp = 2
            elif(self.lamp == 2):
                lamp = 0
        return lamp, turn_lamp
    
    # 处理喇叭逻辑
    def handle_speaker(self, speaker):
        return speaker
    
    # 处理大灯
    def handle_headlamp(self, headlamp, turn_headlamp):
        headlamp = self.headlamp
        if(turn_headlamp == 0 and headlamp == 1):
            turn_headlamp = 1
            if(self.headlamp == 0):
                headlamp = 1
            else:
                headlamp = 0
        elif(turn_headlamp == 1 and headlamp == 0):
            turn_headlamp = 0
        return headlamp, turn_headlamp

    # 处理刹车（将连续的制动输入值转换为离散的制动状态（0 = 松开，1 = 制动））
    def handle_Brake(self, Brake):
        return 1 if(float(Brake) > 0.4) else 0


    # ******************************************ROS节点话题，重要*******************************************************
    # 该方法用于更新底盘的控制状态，并根据挡位、速度、转向和制动等参数生成控制命令。
    # /cmd_ctrl话题发布     data_pub_ctrl
    def data_pub_ctrl(self, gear, velocity, steering, Brake):
        if(gear != 5): # 当没有按键按下时，gear为5， 当gear不等于5时，挡位才会变化
            self.gear = gear

        # 基于挡位的速度和转向控制
        if(self.gear == 3 or self.gear == 1 ): # 当驻车的时候速度为0,转向角为0，当空挡的时候只能控制方向盘
            self.velocity = 0.0
            if(self.gear == 1):
                self.steering = 0.0
            else:
                self.steering = steering
        else:
            if(Brake):
                self.velocity = 0.0
            else:
                self.velocity = velocity
            self.steering = steering
            
        self.Brake = Brake    # 将输入的制动状态（Brake）直接更新到底盘属性
        
    # 该方法用于更新底盘的 IO 设备状态，包括控制使能、大灯、转向灯和喇叭。
    # /io_cmd话题发布   data_pub_io
    def data_pub_io(self, enable, headlamp, lamp, speaker):
        self.enable = enable
        self.headlamp = headlamp   # 大灯
        self.lamp = lamp    # 转向灯
        self.speaker = speaker   # 喇叭

    # 判断是否驻车,去驻车闪烁    判断控制命令是否需要更新
    def is_change_cmd(self, gear, velocity, steering, Brake):
        return True if(self.gear == 1) else False

    def is_change_io(self, enable, headlamp, lamp, speaker):
        return True if (self.enable == enable and self.headlamp==headlamp and self.lamp==lamp and self.speaker==speaker) else False

    