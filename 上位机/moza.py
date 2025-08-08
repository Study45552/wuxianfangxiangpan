# coding:utf-8
#!/usr/bin/env python3
"""
数据流程:
(1)程序启动后初始化所有控制器设备
(2)在主循环中，持续读取方向盘和踏板的数据
(3)将读取的数据存储到 Information 对象中
(4)通过 TextPrint 类将数据显示在屏幕上
(5)将 Information 对象中的数据整理成列表并序列化为 JSON 格式
(6)通过 UDP 组播将 JSON 数据发送到指定网络地址。程序中的网络配置参数(IP 地址、端口号)可以根据实际使用环境进行调整，以适应不同的网络设置和应用需求。
(7)循环此过程，直到用户关闭程序
"""

import pygame
import sys
import json
from socket import *
import struct

#####  1. 常量定义部分
# 定义了颜色常量用于界面显示 白底黑字
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
# 设置了网络通信参数（IP 地址、端口号、组播组等）
SENDERIP = '192.168.3.11'  # 本地ip
SENDERPORT = 40101  # 本地接口
MYPORT = 40102  # 发送数据到该端口
MYGROUP = '224.0.0.1'  # 组播组
MYTTL = 255  # 发送数据的TTL值


#####  2. Information类
# 用于存储所有控制器的状态信息，包含方向盘角度、油门刹车值、各种灯光和档位状态，并提供方法将所有数据整理成列表格式
class Information:
    # 定义类的构造函数，用于初始化对象的属性。
    def __init__(self):
        self.steer = ""
        self.accelerate = ""
        self.decelerate = ""
        self.left_lighting = ""
        self.right_lighting = ""
        self.trumpet = ""
        self.headlight = ""
        self.P = ""
        self.R = ""
        self.D = ""
        self.N = ""
        self.increase_speedlimit = ""
        self.decrease_speedlimit = ""
        self.list = []
    # 该方法的作用是整理属性到列表
    def update(self):
        self.list = [self.steer,self.accelerate,self.decelerate,self.left_lighting,self.right_lighting,self.trumpet,self.headlight,self.P,self.R,self.D,self.N,self.increase_speedlimit,self.decrease_speedlimit]
    # 该方法的作用是提供数据接口
    def data(self):
        return self.list

moza_information = Information()   # 创建一个information类的实例对象并赋值

# 3、 TextPrint 类    （不重要）
# 辅助类，用于在屏幕上格式化显示文本信息， 提供缩进、重置等功能，使信息显示更有条理
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def defprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


# 4、 JoystickSteer 方向盘类
# 处理方向盘控制器的数据读取和显示：
# 通过 Pygame 的 Joystick 模块初始化方向盘设备并读取其数据，包括方向盘角度和多个按钮状态（转向灯、喇叭、档位等），将数据更新到全局数据对象moza_information中
class joystick_steer:
    # 初始化控制器
    def __init__(self,textPrint):
        self.joystick = pygame.joystick.Joystick(1)
        self.textPrint = textPrint
    # 初始化控制器并显示信息(不重要)
    def joystick_init(self):
        self.joystick.init()  # 初始化 joystick 模块

        self.textPrint.defprint(screen, "Joystick steer")
        self.textPrint.indent()  # 初始化 joystick 模块
        name = self.joystick.get_name()  # 获得 Joystick 系统名称
        self.textPrint.defprint(screen, "Joystick name: {}".format(name))

    # 读取控制器的数据，实际上读取到方向盘的按钮数据和转轴数据，并将数据更新到moza_information中
    def get_data(self):
        axes = self.joystick.get_numaxes()  # 获得 Joystick 操纵轴的数量
        self.textPrint.defprint(screen, "Number of axes: {}".format(axes))
        self.textPrint.indent()
        # 读取方向盘转向轴数据
        axis = self.joystick.get_axis(0)  # 获得操纵轴的当前坐标
        self.textPrint.defprint(screen, "Axis steer value: {:>6.3f}".format(axis))  # :>6.3f ：总长度为6位,精确到小数点后三位的浮点类型
        self.textPrint.unindent()  # 不缩进
        #  处理转向数据：转换为0-2范围并保存到全局对象
        axis = float(axis) + 1
        axis = '%.3f' % axis
        moza_information.steer = axis
        #  获取按钮数量并显示
        buttons = self.joystick.get_numbuttons()  # 获得 Joystick 上追踪球的数量
        self.textPrint.defprint(screen, "Number of buttons: {}".format(buttons))

        # 读取各个按钮的状态，并更新到全局对象，并将按钮的ID与功能进行映射张其中0010606
        #速度上限提高
        button = self.joystick.get_button(13)  # 获得当前按钮的状态。
        moza_information.increase_speedlimit = button
        self.textPrint.defprint(screen, " increase_speedlimit value: {}".format( button))
        #速度上限降低
        button = self.joystick.get_button(12)  # 获得当前按钮的状态。
        moza_information.decrease_speedlimit = button
        self.textPrint.defprint(screen, "decrease_speedlimit value: {}".format( button))
        #左转
        button = self.joystick.get_button(18)  # 获得当前按钮的状态。
        moza_information.left_lighting = button
        self.textPrint.defprint(screen, "left_lighting value: {}".format( button))
        #右转
        button = self.joystick.get_button(31)  # 获得当前按钮的状态。
        moza_information.right_lighting = button
        self.textPrint.defprint(screen, "right_lighting value: {}".format(button))
        #喇叭
        button = self.joystick.get_button(20)  # 获得当前按钮的状态。
        moza_information.trumpet = button
        self.textPrint.defprint(screen, "trumpet value: {}".format(button))
        #示廓灯
        button = self.joystick.get_button(33)  # 获得当前按钮的状态。
        moza_information.headlight = button
        self.textPrint.defprint(screen, "headlight value: {}".format(button))
        #P档
        button = self.joystick.get_button(21)  # 获得当前按钮的状态。
        moza_information.P = button
        self.textPrint.defprint(screen, "P value: {}".format(button))
        #R档
        button = self.joystick.get_button(22)  # 获得当前按钮的状态。
        moza_information.R = button
        self.textPrint.defprint(screen, "R value: {}".format(button))
        #D档
        button = self.joystick.get_button(34)  # 获得当前按钮的状态。
        moza_information.D = button
        self.textPrint.defprint(screen, "D value: {}".format(button))
        #N档
        button = self.joystick.get_button(35)  # 获得当前按钮的状态。
        moza_information.N = button
        self.textPrint.defprint(screen, "N value: {}".format(button))
        self.textPrint.unindent()


# 5、 JoystickPedal 油门刹车类
# 通过Joystick 模块初始化并连接踏板控制器，读取踏板的油门和刹车输入数据（操纵轴数据），并将数据更新到全局数据对象moza_information中
class joystick_pedal:
    def __init__(self,textPrint):
        self.joystick = pygame.joystick.Joystick(0)  # 连接控制器
        self.textPrint = textPrint
    # 初始化控制器并显示信息
    def joystick_init(self):
        self.joystick.init()  # 初始化 joystick 模块

        self.textPrint.defprint(screen, "Joystick pedal")
        self.textPrint.indent()  # 初始化 joystick 模块

        name = self.joystick.get_name()  # 获得 Joystick 系统名称
        self.textPrint.defprint(screen, "Joystick name: {}".format(name))

    # 读取控制器数据
    def get_data(self):
        axes = self.joystick.get_numaxes()  # 获得 Joystick 操纵轴的数量
        self.textPrint.defprint(screen, "Number of axes: {}".format(axes))
        self.textPrint.indent()

        # 读取油门数据
        axis = self.joystick.get_axis(0)  # 获得操纵轴的当前坐标
        axis = float(axis) + 1
        axis = '%.3f' % axis 
        moza_information.accelerate = axis  # 保存油门值
        self.textPrint.defprint(screen, "accelerate value: {}".format(axis))

        # 读取刹车刹车数据
        axis = self.joystick.get_axis(1)  # 获得操纵轴的当前坐标
        axis = float(axis) + 1
        axis = '%.3f' % axis
        moza_information.decelerate = axis # 保存刹车值
        self.textPrint.defprint(screen, "decelerate value: {}".format(axis))
        self.textPrint.unindent()


# ******************************************  网络功能 （十分重要）*******************************************************
#  创建和配置一个 UDP 套接字并配置组播参数（TTL 和加入组播组），将控制器数据序列化为 JSON 格式发送到组播地址
def create_socket():

    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)   # 创建 UDP 套接字  参数：(1)AF_INET：使用 IPv4 地址族；(2)SOCK_DGRAM：使用 UDP 协议； (3)IPPROTO_UDP：显式指定 UDP 协议
    s.bind((SENDERIP,SENDERPORT))   # 绑定本地 IP 地址和端口, 绑定后，套接字将从该地址和端口发送数据
    ttl_bin = struct.pack('@i', MYTTL)  # 设置组播 TTL(控制组播数据包可以经过的路由器跳数)  
                                        # 参数：MYTTL = 255：表示数据包可以在本地网络自由传播;   若MYTTL=1：数据包只能在同一局域网内传播，无法通过路由器转发。
                                        # struct.pack('@i', MYTTL)：将整数转换为网络字节序的二进制数据
    s.setsockopt(IPPROTO_IP, IP_MULTICAST_TTL, ttl_bin)  # 参数：（1）IPPROTO_IP：指定 IP 协议层  （2） IP_MULTICAST_TTL：组播 TTL 选项  （3）ttl_bin：TTL 值的二进制表示
    
    # 将套接字加入指定的组播组，目的允许套接字接收该组播组的数据（虽然当前程序仅用于发送）
    status = s.setsockopt(
        IPPROTO_IP,          
        IP_ADD_MEMBERSHIP,   # # 加入组播组的选项
        inet_aton(MYGROUP) + inet_aton(SENDERIP)
    )  
    # inet_aton（）将点分十进制 IP 地址转换为 32 位二进制格式   这里将组播地址（如224.0.0.1）和本地 IP 地址转换为二进制格式， 指定从哪个接口加入组播组

    print("链接成功")

    return s   # 将创建并配置好的 UDP 组播套接字对象 返回给调用者



# 代码的主要逻辑部分，协调各个模块工作，实现 “读取控制器数据→更新状态→发送网络数据” 的循环。
pygame.init()
done = False
clock = pygame.time.Clock()  # 时钟对象，控制循环帧率
pygame.joystick.init()

# 创建方向盘和油门刹车控制器对象并初始化
joystick_0 = joystick_steer(textPrint)
joystick_0.joystick_init()
joystick_1 = joystick_pedal(textPrint)
joystick_1.joystick_init()
# 调用 create_socket() 获取组播套接字，用于发送数据。
send_data = create_socket()  
# -------- 主循环程序-----------
# 不断读取控制器数据，更新显示信息，定时将数据通过网络发送
while done == False:
    # 1. 处理用户事件：
    for event in pygame.event.get():  # 用户操作事件获取
        if event.type == pygame.QUIT:  # 如果用户点击关闭
            done = True  # 停止循环
        # 可能的操纵杆动作:   轴   球  按钮  帽子
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")  # 操纵杆按钮按下
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")  # 操纵杆按钮解除

    joystick_count = pygame.joystick.get_count()  # 2. 获取控制器数量

    # 3. 读取控制器数据并更新全局状态
    joystick_0.get_data()
    joystick_1.get_data()
    moza_information.update()

    # 4. 序列化数据为JSON格式并通过UDP组播发送到网络
    DATA = moza_information.data()  # data()方法返回参数列表信息
    data = json.dumps(DATA)   # 数据序列化为 JSON 字符串
    send_data.sendto(bytes(data.encode('utf-8')), (MYGROUP, MYPORT)) 
    # 参数一：bytes(data.encode('utf-8'))：将 JSON 字符串转换为 UTF-8 编码的二进制数据（bytes类型）。  参数二：(MYGROUP, MYPORT)：组播目标地址和端口

    # 5. 打印调试信息并控制帧率 每秒执行 20 次
    print(data,sys.getsizeof(data))
    clock.tick(20)  

# 资源释放 
send_data.close() # 关闭网络套接字
pygame.quit()  # 卸载 joystick 模块



