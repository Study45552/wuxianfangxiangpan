#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import json
import threading
import time
import flask
from flask import request
from date_handle.msg import io_fb
from date_handle.msg import odo_fb
from date_handle.msg import lr_wheel_fb
from date_handle.msg import rr_wheel_fb
from date_handle.msg import bms_Infor_fb
from date_handle.msg import bms_flag_Infor_fb
from date_handle.msg import Veh_Diag_fb
from date_handle.msg import ctrl_fb

""" 
实现了一个将 ROS话题数据转换为 JSON 格式，并通过 Flask Web 接口提供访问的系统。 
1. 订阅 ROS 话题，接收车辆各个模块的反馈数据。
2. 将接收到的 ROS 消息转换为 JSON 格式, 存储在全局变量中
3. 提供 Flask Web 接口，使用 Flask 提供 HTTP 接口，允许外部系统获取 JSON 格式的车辆数据
4. 支持多线程处理，确保数据接收和 Web 服务的并发运行
"""


# 以下几个函数的功能均为： 当 ROS 话题接收到新消息时，将消息字段转换为 JSON 格式并存储在全局变量中  （不太重要）
# 数据映射：每个回调函数对应一个 ROS 话题，将消息字段映射为 JSON 字典
# 示例：io_fb话题的io_fb_enable字段映射为 JSON 中的"io_fb_enable"键
def io_fb_callback(io_fb):
    '''(1) io反馈的回调函数
    作用: json化
    '''
    global io_fb_data
    io_fb_data =  {
            "io_fb_enable": io_fb.io_fb_enable,
            "io_fb_upper_beam_headlamp": io_fb.io_fb_upper_beam_headlamp,
            "io_fb_turn_lamp": io_fb.io_fb_turn_lamp,
            "io_fb_braking_lamp": io_fb.io_fb_braking_lamp,
            "io_fb_speaker": io_fb.io_fb_speaker,
            "io_fb_fm_impact_sensor": io_fb.io_fb_fm_impact_sensor,
            "io_fb_rm_impact_sensor": io_fb.io_fb_rm_impact_sensor
    }

def odo_fb_callback(odo_fb):
    '''(2) odo反馈的回调函数
    作用: json化
    '''
    global odo_fb_data
    odo_fb_data = {
        "odo_fb_accumulative_mileage": odo_fb.odo_fb_accumulative_mileage,
        "odo_fb_accumulative_angular": odo_fb.odo_fb_accumulative_angular
    }

def lr_wheel_fb_callback(lr_wheel_fb):
    '''(3) lr_wheel_fb反馈的回调函数
    作用: json化
    '''
    global lr_wheel_fb_data
    lr_wheel_fb_data = {
        "lr_wheel_fb_velocity": lr_wheel_fb.lr_wheel_fb_velocity,
        "lr_wheel_fb_pulse": lr_wheel_fb.lr_wheel_fb_pulse
    }

def rr_wheel_fb_callback(rr_wheel_fb):
    '''(4) rr_wheel_fb反馈的回调函数
    作用: json化
    '''
    global rr_wheel_fb_data
    rr_wheel_fb_data = {
        "rr_wheel_fb_velocity": rr_wheel_fb.rr_wheel_fb_velocity,
        "rr_wheel_fb_pulse": rr_wheel_fb.rr_wheel_fb_pulse
    }

def bms_Infor_fb_callback(bms_Infor_fb):
    '''(5) bms_Info_callback反馈的回调函数
    作用: json化
    '''
    global bms_Infor_fb_data
    bms_Infor_fb_data = {
        "bms_Infor_voltage": bms_Infor_fb.bms_Infor_voltage,
        "bms_Infor_current": bms_Infor_fb.bms_Infor_current,
        "bms_Infor_remaining_capacity": bms_Infor_fb.bms_Infor_remaining_capacity
    }

def bms_flag_Infor_fb_callback(bms_flag_Infor_fb):
    ''' (6) bms_flag_Info_callback反馈的回调函数
    作用:json化
    '''
    global bms_flag_Infor_fb_data
    bms_flag_Infor_fb_data = {
        "bms_flag_Infor_soc": bms_flag_Infor_fb.bms_flag_Infor_soc
    }

def Veh_Diag_fb_callback(Veh_Diag_fb):
    ''' (7) Veh_fb_Diag_callback反馈的回调函数
    作用: json化
    '''
    global Veh_Diag_fb_data
    Veh_Diag_fb_data = {
        "Veh_fb_FaultLevel": Veh_Diag_fb.Veh_fb_FaultLevel,
        "Veh_fb_AutoCANCtrlCmd": Veh_Diag_fb.Veh_fb_AutoCANCtrlCmd,
        "Veh_fb_AutolOCANCmd": Veh_Diag_fb.Veh_fb_AutolOCANCmd
    }

def ctrl_fb_callback(ctrl_fb):
    '''(8) ctrl_fb_callback反馈的回调函数
    作用: json化
    '''
    global ctrl_fb_data
    # mutext.acquire()
    ctrl_fb_data = {
        "ctrl_fb_gear": ctrl_fb.ctrl_fb_gear,
        "ctrl_fb_velocity": ctrl_fb.ctrl_fb_velocity,
        "ctrl_fb_steering": ctrl_fb.ctrl_fb_steering,
        "ctrl_fb_Brake": ctrl_fb.ctrl_fb_Brake,
        "ctrl_fb_mode": ctrl_fb.ctrl_fb_mode,
    }
    # mutext.release()



# 定时发送数据   每隔 10 秒将所有全局数据合并为一个 JSON 对象并打印
def send():
    global io_fb_data; global odo_fb_data; global lr_wheel_fb_data; global rr_wheel_fb_data
    while(True):
        time.sleep(10)
        res = {
            "io_fb": io_fb_data,
            "odo_fb": odo_fb_data,
            "lr_wheel_fb": lr_wheel_fb_data,
            "rr_wheel_fb": rr_wheel_fb_data,
            "bms_Infor_fb": bms_flag_Infor_fb_data,
            "bms_flag_Infor_fb": bms_flag_Infor_fb_data,
            "Veh_Diag_fb": Veh_Diag_fb_data,
            "ctrl_fb": ctrl_fb_data
        }
        print(json.dumps(res))  # 将合并后的 JSON 数据打印到控制台



# 初始化函数，创建 ROS 节点并订阅多个车辆状态话题。
# 当指定话题有新消息发布时，自动调用对应的回调函数，回调函数负责将 ROS 消息转换为 JSON 格式并存储在全局变量中，供 Web 接口和定时发送函数使用
def init():
    rospy.init_node('listener', anonymous=True, disable_signals = True)   # 初始化 ROS 节点，建立与 ROS 系统的连接

    # ROS的话题订阅    （参数1：话题名称   参数2：消息类型   参数3：回调函数）
    rospy.Subscriber('io_fb', io_fb, io_fb_callback)   
    rospy.Subscriber('odo_fb', odo_fb, odo_fb_callback)
    rospy.Subscriber('lr_wheel_fb', lr_wheel_fb, lr_wheel_fb_callback)
    rospy.Subscriber('rr_wheel_fb', rr_wheel_fb, rr_wheel_fb_callback)
    rospy.Subscriber('bms_Infor_fb', bms_Infor_fb, bms_Infor_fb_callback)
    rospy.Subscriber('bms_flag_Infor_fb', bms_flag_Infor_fb, bms_flag_Infor_fb_callback)
    rospy.Subscriber('Veh_fb_Diag', Veh_Diag_fb, Veh_Diag_fb_callback)
    rospy.Subscriber('ctrl_fb', ctrl_fb, ctrl_fb_callback)
    rospy.spin()  # ROS 消息循环，阻塞当前线程，持续处理 ROS 消息，确保订阅者能够实时接收并处理新消息


# 使用 Flask 框架创建了多个 Web 接口  
# 基于 HTTP 协议提供多个车辆状态数据的访问接口，将 ROS 系统中的车辆状态数据转换为 JSON 格式返回
server = flask.Flask(__name__)


#  控制反馈接口 /get_ctrl_fb ， 返回车辆控制反馈数据（挡位、速度、转向等）  
#  收到对/get_ctrl_fb的请求时，调用get_ctrl_fb函数处理
"""
客户端请求(GET/POST /get_ctrl_fb) → Flask路由匹配 → 调用get_ctrl_fb函数 →
↓
获取全局数据ctrl_fb_data → 组装响应字典 → 转换为JSON → 返回给客户端
"""
@server.route('/get_ctrl_fb', methods=['get', 'post'])
def get_ctrl_fb():
    global ctrl_fb_data
    # mutext.acquire()    # 线程锁获取操作，防止多线程并发访问时出现数据不一致
    res = {
        "code": 200,    # HTTP 状态码（200 表示请求成功）
        "data":ctrl_fb_data     # 实际返回的车辆控制反馈数据
    }
    # mutext.release()   # 线程锁释放操作
    return json.dumps(res, ensure_ascii = False)  # 将响应字典转换为 JSON 字符串并返回


#  IO 设备反馈接口  /get_io_fb    返回车辆 IO 设备状态（灯光、喇叭、传感器等）
@server.route('/get_io_fb', methods=['get', 'post'])
def get_io_fb():
    '''网络接口
    '''
    global io_fb_data
    # mutext.acquire()
    res = {
        "code": 200,
        "data":io_fb_data
    }
    # mutext.release()
    return json.dumps(res, ensure_ascii = False)


# 里程计反馈接口  ：返回里程计数据（行驶里程、角度等）
@server.route('/get_odo_fb', methods=['get', 'post'])
def get_odo_fb():
    '''网络接口
    '''
    global odo_fb_data
    # mutext.acquire()
    res = {
        "code": 200,
        "data":odo_fb_data
    }
    # mutext.release()
    return json.dumps(res, ensure_ascii = False)


#  电池信息接口
@server.route('/get_bms_Info_fb', methods=['get', 'post'])
def get_bms_Info_fb():
    '''网络接口
    '''
    global bms_Infor_fb_data
    # mutext.acquire()
    res = {
        "code": 200,
        "data":bms_Infor_fb_data
    }
    # mutext.release()
    return json.dumps(res, ensure_ascii = False)


# 电池信息接口 /get_bms_Info_fb
@server.route('/get_bms_flag_Info_fb', methods=['get', 'post'])
def get_bms_flag_Info_fb():
    '''网络接口
    '''
    global bms_flag_Infor_fb_data
    # mutext.acquire()
    res = {
        "code": 200,
        "data": bms_flag_Infor_fb_data
    }
    # mutext.release()
    return json.dumps(res, ensure_ascii = False)


# 主函数  负责初始化系统组件、启动**多线程**，并运行 Flask Web 服务器。
if __name__ == '__main__':

    # 初始化存储车辆状态数据的全局字典, 初始均为空，等待 ROS 回调函数填充
    io_fb_data = {}
    odo_fb_data = {}
    lr_wheel_fb_data = {}
    rr_wheel_fb_data = {}
    bms_Infor_fb_data = {}; 
    bms_flag_Infor_fb_data = {}
    Veh_Diag_fb_data = {}
    ctrl_fb_data ={}
    res = {}

    # ********************************************************************************************
    mutext = threading.Lock()   # 创建线程锁，用于保护多线程环境下的全局数据访问    锁机制避免并发访问导致的数据不一致
    # ********************************************************************************************

    # thread = threading.Thread(target = send)   # 启动一个定时发送数据的线程（send函数每隔 10 秒打印所有数据）
    # thread.setDaemon(True)    # 设置为守护线程，主线程退出时自动终止
    # thread.start()

    # 启动 ROS 订阅线程   在新线程中运行init()函数
    threading.Thread(target = init).start()    # 初始化 ROS 节点, 订阅多个车辆状态话题, 注册回调函数处理数据, 进入 ROS 消息循环
    # 多线程意义：ROS 消息处理与 Web 服务并行运行，互不阻塞

    server.run(debug = True, port = 8093, host = '0.0.0.0', threaded = True)   # 启动 Flask 服务器，提供 Web 接口访问车辆状态数据
    # init()
 