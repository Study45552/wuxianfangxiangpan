#!/usr/bin/python
#-*- coding:UTF-8 -*-

import rospy
import socket
import time
import json
import chassis
from date_handle.msg import ctrl_cmd
from date_handle.msg import io_cmd

"""
基于 ROS的车辆控制程序, 主要功能是接收组播数据并将其转换为车辆控制命令
1. 初始化模块：创建 ROS 节点、底盘对象和网络连接
2. 数据接收模块：通过 UDP 组播接收控制器数据
3. 数据处理模块：解析并处理控制器输入，转换为车辆控制命令
4. 命令发布模块：将控制命令发布到 ROS 话题
"""
#  ****************************************  重要文件  *************************************************************


# 初始化 ROS 节点和通信接口，创建底盘控制对象
def init():
    '''
    返回值：
    chassis_: 底盘对象，里面封装了对底盘对象的操作
    cmd_data / io_data: ROS消息发布对象
    cmd_ctrl_data / io_ctrl_data: 控制数据对象, 通过调用chassis_对象的ctrl_cmd方法将chassis_对象的属性赋值给他
    '''
    chassis_  =  chassis.Chassis(3, 0.0, 0.0, 0, 0, 0, 0, 0) # 初始化chassis底盘对象, 对各个参数进行默认赋值  底盘默认状态：空挡（3）、速度 0、转向 0

    rospy.init_node('data_handle') # 初始化node节点
    # 创建话题发布   (参数1：话题名称；  参数2：消息类型；  参数3：消息队列的大小) 
    # ctrl_cmd消息包含运动控制字段（挡位、速度、转向、制动）     io_cmd消息包含 IO 控制字段（使能、灯光、喇叭）
    cmd_data = rospy.Publisher('ctrl_cmd', ctrl_cmd, queue_size = 1)   # 初始化话题发布
    io_data = rospy.Publisher('io_cmd', io_cmd, queue_size = 1)    # 初始化话题发布
    rate = rospy.Rate(500) # 调整接收速率
    
    cmd_ctrl_data = ctrl_cmd()   # 创建运动控制命令消息实例
    io_ctrl_data = io_cmd()      # 创建IO控制命令消息实例

    # 发布消息到 ROS 话题
    cmd_data.publish(chassis_.ctrl_cmd(cmd_ctrl_data))   # 发布运动控制命令
    io_data.publish(chassis_.io_cmd(io_ctrl_data))       # 发布IO控制命令
    return chassis_, cmd_data, cmd_ctrl_data, io_data, io_ctrl_data        


# 创建和配置一个 UDP 组播套接字，绑定到指定端口，用于接收网络中的发送组播数据。
# 创建UDP套接字 → 允许端口重用 → 绑定本地端口 → 加入组播组 → 设置阻塞模式 → 返回套接字
def init_socket():
    '''组播socket
    '''
    multicast_group = '224.0.0.1'   # 组播组地址
    server_address = ('', 40102)     # 本地接收地址（所有接口，端口40102）
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建 UDP 套接字     参数1：使用 IPv4 地址族。   参数2：使用 UDP 协议（无连接、适合实时数据传输）
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)   # 允许同一端口被多个进程绑定（调试时避免端口占用问题）。   参数1 ：SOL_SOCKET：套接字层选项。  参数2：SO_REUSEADDR：启用端口重用。
    sock.bind(server_address)    # 将套接字绑定到本地地址中所有接口的40102端口。套接字将接收发送到40102端口的所有数据。
    status = sock.setsockopt(
        socket.IPPROTO_IP,           # IP协议层选项
        socket.IP_ADD_MEMBERSHIP,    # 将套接字加入组播组
        socket.inet_aton('224.0.0.1') + socket.inet_aton('192.168.3.15')       # 组播地址和本地IP地址的二进制表示
        )   # 将套接字加入指定的组播组，使其能接收该组播组的数据。
    
    sock.setblocking(1)   # 设置套接字为阻塞模式。   当调用sock.recvfrom()时，若无数据则阻塞等待，直到收到数据。
    return sock          # 返回套接组


# 将接收到的控制器原始数据转换为车辆可识别的控制命令，更新底盘对象的状态，并将控制命令发布到 ROS 话题。
"""
接收到的控制器数据（列表） → 各数据字段解析 → 
↓(调用Chassis处理方法解析)
转向角、速度、灯光等控制参数 → 计算挡位 → 
↓（更新底盘对象）
chassis_对象状态更新 → 
↓(发布到ROS话题)
ctrl_cmd话题(运动控制)、io_cmd话题(IO控制)
"""
def publish(data, chassis_, cmd_data, cmd_ctrl_data, io_data, io_ctrl_data):
    '''消息发布函数
    data[0]: 转向
    data[1]: 速度
    data[2]: 刹车
    data[3]: 左转灯
    data[4]: 右转灯
    data[5]: 喇叭
    data[6]: 大灯
    data[7]: 驻车
    data[8]: 后退
    data[9]: 前进
    data[10]: 空挡
    data[11]: 加速挡位
    data[12]: 减速挡位
    '''
    # 定义全局变量
    global increase_speedlimit; global decrease_speedlimit     # 速度挡位按钮状态
    global turn_lamp; global turn_headlamp                     # 转向灯和大灯状态

    # 加速和减速挡位按钮处理  仅在 “按下” 的瞬间触发挡位增加 / 降低，避免持续按下时重复触发
    if(increase_speedlimit == 0 and data[11] == 1):
        if(chassis_.speedlimit < 3):
            chassis_.speedlimit += 1
        increase_speedlimit = 1
    elif(increase_speedlimit == 1 and data[11] == 0):
        increase_speedlimit = 0
    elif(decrease_speedlimit == 0 and data[12] == 1):
        if(chassis_.speedlimit > 0):
            chassis_.speedlimit -= 1
        decrease_speedlimit = 1
    elif(decrease_speedlimit == 1 and data[12] == 0):
        decrease_speedlimit = 0

    steering = chassis_.handle_steering(data[0])          # 转向处理：将[0,2]转为角度
    velocity = chassis_.handle_veplocity(data[1])         # 速度处理：考虑限速挡位
    lamp, turn_lamp = chassis_.handle_lamp(data[3], data[4], turn_lamp)  # 转向灯处理：左右互斥，返回新状态和标志
    speaker = chassis_.handle_speaker(data[5])            # 喇叭处理
    headlamp, turn_headlamp = chassis_.handle_headlamp(data[6], turn_headlamp)    # 灯处理
    Brake = chassis_.handle_Brake(data[2])      # 刹车处理：将[0,1]转为0或1

    # 挡位计算逻辑，当按钮被按下，该位为1会改变gear值，当gear = 5时，gear值将与上次一样
    # 优先级：驻车 1 > 后退 2 > 前进 3 > 空挡 4  > 保持原挡位 5
    gear = 1 if(data[7]) else (2 if(data[8]) else (4 if(data[9]) else (3 if(data[10]) else 5)))
    
    # 更新底盘对象状态
    enable = 1; 
    chassis_.data_pub_ctrl(gear, velocity, steering, Brake)             # 更新底盘运动状态（挡位、速度、转向、制动）
    chassis_.data_pub_io(enable, headlamp, lamp, speaker)               # 更新底盘IO状态（使能、灯光、喇叭）
    if(not chassis_.is_change_cmd(gear, velocity, steering, Brake)):    # 有条件发布运动控制命令（仅当状态变化时）
        cmd_data.publish(chassis_.ctrl_cmd(cmd_ctrl_data))
    # if(not chassis_.is_change_io(enable, headlamp, lamp, speaker)):
    io_data.publish(chassis_.io_cmd(io_ctrl_data))              # # 无条件发布IO命令
    

# main入口  初始化系统组件并启动数据接收 - 处理 - 发布的主循环。
if __name__ == '__main__':
    # 初始化chassis对象，初始化 ROS 节点data_handle，创建ctrl_cmd和io_cmd话题的发布者，建立ros连接
    chassis_, cmd_data, cmd_ctrl_data, io_data, io_ctrl_data = init()   
    s = init_socket()             # 调用init_socket()方法，建立socket连接
    # 全局状态初始化
    increase_speedlimit = 0
    decrease_speedlimit = 0
    turn_lamp = 0 # 转向信号， 0为关， 1左 2右
    turn_headlamp = 0 # 0为关，1为开

    while(True):
        data, add = s.recvfrom(106)    # 接收控制器程序发送的 UDP 组播数据，  data：接收到的二进制数据     106：接收缓冲区大小（需与发送方数据长度匹配）
        
        data = data.decode('UTF-8')   # 解析 JSON 数据，  二进制数据 → UTF-8字符串 → JSON对象 → Python列表
        data = json.loads(data)

        publish(data, chassis_, cmd_data, cmd_ctrl_data, io_data, io_ctrl_data)   # 调用publish()处理并发布数据
    
