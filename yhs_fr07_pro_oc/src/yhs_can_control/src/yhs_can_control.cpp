#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "yhs_can_control.h"

/*
实现了一个基于 ROS 的 CAN 总线控制节点，
主要功能是将 ROS 话题消息转换为 CAN 帧发送至车辆，并将接收到的 CAN 数据解析为 ROS 消息发布。
*/

namespace yhs_tool 
{

/*
	CanControl类实现了以下核心功能:
	1. ROS-CAN 数据转换：将 ROS 的ctrl_cmd和io_cmd话题消息转换为 CAN 帧发送
	2. CAN-ROS 数据解析：将接收到的 CAN 帧解析为 ROS 话题消息(如ctrl_fb、io_fb等)
	3. 多线程设计：使用独立线程接收 CAN 数据，避免阻塞主线程  ************************重要	
		（1）主线程：处理 ROS 回调和用户输入，主线程是ROS的主循环
		（2）接收线程：专注 CAN 数据读取与解析
		线程安全：
			通过互斥锁（cmd_mutex_）保护共享数据，避免多线程同时访问 CAN 发送缓冲区
	4. 设备交互：通过 socket 接口操作 CAN 总线设备
*/

CanControl::CanControl()
{
	ros::NodeHandle private_node("~");
	
}


CanControl::~CanControl()
{

}

// ROS 节点中处理 IO 控制命令的回调函数，负责将接收到的 ROS 消息转换为 CAN 总线帧并发送。
/*
（1）接收ROS话题io_cmd消息 ->  
（2）按照特定协议格式将消息内容编码到 CAN 帧数据中 -> 
（3）添加帧计数和 CRC 校验 -> 
（4）发送至 CAN 总线 -> 
（5）通过 CAN 设备发送数据  -> 
（6）处理发送结果并释放锁
*/ 
void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
{
	static unsigned char count_1 = 0;   // 生成帧计数（每 16 帧循环一次）

	/*
		cmd_mutex_.lock();
		// ... 数据处理 ...
		cmd_mutex_.unlock();
	*/
	cmd_mutex_.lock();     // 线程锁保护，防止多线程并发访问共享缓冲区。获取互斥锁，确保在发送数据时不会被其他线程打断

	// 数据初始化与赋值
	memset(sendData_u_io_, 0, 8);  // 初始化8字节缓冲区
	sendData_u_io_[0] = msg.io_cmd_enable;   // sendData_u_io_是 8 字节数组，对应 CAN 帧的数据部分

	// 灯光控制位操作
	sendData_u_io_[1] = 0xff;
	if(msg.io_cmd_upper_beam_headlamp)
		sendData_u_io_[1] &= 0xff;  // // 远光灯开启
	else sendData_u_io_[1] &= 0xfd;  // 远光灯关闭

	if(msg.io_cmd_turn_lamp == 0)
		sendData_u_io_[1] &= 0xf3;   // 转向灯关闭
	if(msg.io_cmd_turn_lamp == 1)
		sendData_u_io_[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendData_u_io_[1] &= 0xfb;

	// 喇叭控制
	sendData_u_io_[2] = msg.io_cmd_speaker;   

	sendData_u_io_[3] = 0;
	sendData_u_io_[4] = 0;
	sendData_u_io_[5] = 0;


	// *************************************  帧计数与 CRC 校验，以及CAN帧的构造 （很重要）*****************************************************************************
	// 以下代码实现了 CAN 总线通信中的 IO 控制帧发送功能，主要包括帧计数、CRC 校验、CAN 帧构建和发送操作。
	// 生成帧计数，循环计数0-15，每发送16帧数据，count_1重置为0。
	// 帧计数作用：接收方通过比较计数判断消息是否丢失或重复，每 16 帧循环一次，适用于短时间内的消息排序
	count_1 ++;
	if(count_1 == 16)	count_1 = 0;    // 循环计数

	sendData_u_io_[6] =  count_1 << 4;   // 高4位存储计数器值

	// CRC 校验：将前7字节数据异或得到第8字节
	// 这里的CRC校验对前 7 字节数据进行异或运算，结果存入第 8 字节作为校验和，用于检测数据传输错误，接收方需执行相同计算并比较结果
	sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

	// 构建 CAN 帧 ID 和数据长度， 并发送
	send_frames_[0].can_id = 0x98C4D7D0;    // CAN帧ID，唯一标识消息类型（IO控制专用）
    send_frames_[0].can_dlc = 8;            // 数据长度为8字节
	memcpy(send_frames_[0].data, sendData_u_io_, 8);    // 将之前组装好的 sendData_u_io_ 的 IO 控制数据（灯光、喇叭等状态）复制到 CAN 帧的数据部分，准备发送。     data：实际负载数据
														// 复制长度：8 字节（CAN 数据帧的标准长度）

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));   // 通过write函数向 CAN 设备文件描述符发送数据
    if (ret <= 0)    // 发送失败
	{
      ROS_ERROR("send message failed, error code: %d",ret);   // 获取错误码
    }	

	cmd_mutex_.unlock();
}
// ************************************************************* 重要  底盘控制回调函数 ***********************************************************************************

//速度控制回调函数
// 负责将 ROS 节点消息中的速度、转向、挡位等控制指令编码为 CAN 帧格式并发送。  主要逻辑与上面io的类似
// 两个确保数据完整性保障的点：（1）帧计数检测丢帧  （2）CRC 校验检测传输错误（这里的CRC 校验算法是较为简单异或操作，需要考虑下为什么用这个，为什么不用更有效的更好的）
void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg)
{
	unsigned short vel = 0;       // 转换后的速度
	static unsigned char count = 0;    // 静态帧计数器（用于标识消息顺序）
	short angular = msg.ctrl_cmd_steering * 100;   // 存储转换后的转向角度，转向角转换：弧度 → 整数（乘以100）
	// 速度转换：m/s → 整数（乘以1000）
	if(msg.ctrl_cmd_velocity < 0) vel = 0;
	else
	{
		vel = msg.ctrl_cmd_velocity * 1000;
	}

	cmd_mutex_.lock();             // 启动线程锁保护，防止多线程并发访问共享缓冲区。获取互斥锁，确保在发送数据时不会被其他线程打断
	memset(sendData_u_vel_, 0, 8);   // 初始化8字节发送缓冲区，清空发送缓冲区

	// 字节 0 编码：挡位与速度低 4 位， 按位或合并两个字段
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & msg.ctrl_cmd_gear);
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((vel & 0x0f) << 4));
	// 字节 1 编码：速度中间 8 位
	sendData_u_vel_[1] = (vel >> 4) & 0xff;
	// 字节 2 编码：速度高 4 位与转向角低 4 位
	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (vel >> 12));
	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));
	// 字节 3 编码：转向角中间 8 位
	sendData_u_vel_[3] = (angular >> 4) & 0xff;
	// 字节 4 编码：转向角高 4 位与制动状态
	sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((msg.ctrl_cmd_Brake & 0x0f) << 4));
	sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));
	
	// 字节 5-6 编码：保留位与帧计数
	sendData_u_vel_[5] = 0;   // 保留字节，置零

	// 帧计数作用：(1)接收方通过比较计数判断消息顺序 (2)每 16 帧循环一次，适用于短时间通信
	count ++; 						    // 帧计数器自增
	if(count == 16)	count = 0;          // 循环计数，确保计数在 0-15 之间,   
	sendData_u_vel_[6] =  count << 4;   // 高4位存储计数器，低4位置零
	
	// 字节 7 编码：CRC 校验用于检测数据传输错误，
	// CRC 校验原理：将前7字节数据异或得到的结果存入第8字节，接收方需执行相同计算并比较结果
	sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

	// 构建 CAN 帧 ID 和数据长度
	send_frames_[0].can_id = 0x98C4D2D0;     				// can_id设置CAN帧ID（唯一标识消息类型
    send_frames_[0].can_dlc = 8;             				// can_dlc设置数据长度为8字节
	memcpy(send_frames_[0].data, sendData_u_vel_, 8);       // 复制数据到CAN帧中

	// write函数将send_frames_[0]中的数据发送到CAN硬件设备，返回值ret表示发送结果
	// (参数1：dev_handler_是CAN设备的文件描述符（CAN接口），参数2：指向要发送的CAN帧数据的指针，参数3：要发送的数据长度)
	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));     // 向底盘写入CAN帧数据
	
    if (ret <= 0)      // 若读取失败
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	

	cmd_mutex_.unlock();       // 释放线程锁，允许其他线程访问共享资源，确保临界区代码的原子性
}


// ******************************************************* 重要  ************************************************************
// recvData函数是 CAN 总线数据接收与解析的核心线程，负责从 CAN 总线读取数据帧，根据 CAN ID 分类解析，并转换为 ROS 消息发布。
// 流程：循环（ROS节点运行时） → 读取CAN帧 → 根据CAN ID解析数据 → 校验CRC → 发布ROS消息
void CanControl::recvData()
{

	while(ros::ok())    // // 当ROS节点未关闭时持续运行
	{
		// read函数参数    参数1：dev_handler_是CAN设备的文件描述符，参数2：指向接收缓冲区的指针，参数3：要读取的数据长度  CAN 帧大小（通常 16 字节） 
		if(read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)   // 从 CAN 设备读取一帧数据  
		{
			// 读取成功，解析接收到的 CAN 帧数据
			// recv_frames_[0] 是接收到的 CAN 帧数据，包含 can_id、can_dlc 和 data 字段      CAN ID 通常由厂商自定义，高字节标识设备类型，低字节标识消息类型
			// can_id：CAN 帧的标识符，用于区分不同类型的消息
			// can_dlc：数据长度，表示数据部分的字节数
			// data：实际负载数据，包含具体的控制或反馈信息


			for(int j = 0;j < 1;j++)
			{
				// 根据 can_id 进行不同类型的消息处理
				switch (recv_frames_[0].can_id)
				{
					// 速度控制反馈
					// 功能： CAN 总线接收线程中处理速度控制反馈帧的核心逻辑，负责将接收到的原始 CAN 数据解析为 ROS 消息格式。
					case 0x98C4D2EF:
					{
						yhs_can_msgs::ctrl_fb msg;

						// 提取挡位（低4位）
						msg.ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];   
						
						// 组合速度值（3字节）
						msg.ctrl_fb_velocity = (float)((unsigned short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
						
						// 组合转向角（3字节)
						msg.ctrl_fb_steering = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;
						
						// 制动状态解析
						msg.ctrl_fb_Brake = (recv_frames_[0].data[4] & 0x30) >> 4;
						// 控制模式解析
						msg.ctrl_fb_mode = (recv_frames_[0].data[4] & 0xc0) >> 6;

						// 接收方CRC校验     
						// 发送方CRC校验时已经计算前 7 字节异或值存入data[7]，这里是接收方重复计算并比较
						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];
						if(crc == recv_frames_[0].data[7])   // 如果发送方和接收方的CRC校验能匹配的上，那么就发布消息
						{
								
							ctrl_fb_pub_.publish(msg);   // 将解析后的控制反馈数据发送到 ROS 话题系统。
						}

						break;
					}

					// 解析 CAN 总线上的左轮反馈消息，将原始字节数据转换为 ROS 消息格式。
					case 0x98C4D7EF:    // CAN ID
					{
						yhs_can_msgs::lr_wheel_fb msg;
						msg.lr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.lr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])    // CRC校验
						{
								
							lr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					// 解析 CAN 总线上的右轮反馈消息，将原始字节数据转换为 ROS 消息格式。
					case 0x98C4D8EF:    // CAN ID
					{
						yhs_can_msgs::rr_wheel_fb msg;
						msg.rr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.rr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])    // CRC校验
						{
								
							rr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					// 解析 CAN 总线上的 IO 设备反馈消息（CAN ID: 0x98C4DAEF），将原始字节数据转换为 ROS 消息格式。
					case 0x98C4DAEF:     // CAN ID
					{
						yhs_can_msgs::io_fb msg;
						if(0x01 & recv_frames_[0].data[0]) msg.io_fb_enable = true;	else msg.io_fb_enable = false;   //  系统使能标志解析
	
						if(0x02 & recv_frames_[0].data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;   // 远光灯状态解析

						msg.io_fb_turn_lamp = (0x0c & recv_frames_[0].data[1]) >> 2;

						if(0x10 & recv_frames_[0].data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;   // 转向灯状态解析

						if(0x01 & recv_frames_[0].data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;  // 喇叭状态解析

						if(0x02 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;    // 前向碰撞传感器状态解析

						if(0x10 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;    // 后向碰撞传感器状态解析

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])    // CRC 校验
						{
								
							io_fb_pub_.publish(msg);
						}

						break;
					}

					// 里程计反馈（不重要）
					case 0x98C4DEEF:
					{
						yhs_can_msgs::odo_fb msg;
						msg.odo_fb_accumulative_mileage = (float)((int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

						msg.odo_fb_accumulative_angular = (float)((int)(recv_frames_[0].data[7] << 24 | recv_frames_[0].data[6] << 16 | recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 1000;

						odo_fb_pub_.publish(msg);

						break;
					}

					// 解析 CAN 总线上的电池管理系统（BMS）信息反馈消息 （不重要）
					case 0x98C4E1EF:
					{
						yhs_can_msgs::bms_Infor_fb msg;
						msg.bms_Infor_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

						msg.bms_Infor_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

						msg.bms_Infor_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					// 解析 CAN 总线上的电池管理系统（BMS）状态标志反馈消息 （不重要）
					case 0x98C4E2EF:
					{
						yhs_can_msgs::bms_flag_Infor_fb msg;
						msg.bms_flag_Infor_soc = recv_frames_[0].data[0];

						if(0x01 & recv_frames_[0].data[1]) msg.bms_flag_Infor_single_ov = true;	else msg.bms_flag_Infor_single_ov = false;

						if(0x02 & recv_frames_[0].data[1]) msg.bms_flag_Infor_single_uv = true;	else msg.bms_flag_Infor_single_uv = false;

						if(0x04 & recv_frames_[0].data[1]) msg.bms_flag_Infor_ov = true;	else msg.bms_flag_Infor_ov = false;

						if(0x08 & recv_frames_[0].data[1]) msg.bms_flag_Infor_uv = true;	else msg.bms_flag_Infor_uv = false;

						if(0x10 & recv_frames_[0].data[1]) msg.bms_flag_Infor_charge_ot = true;	else msg.bms_flag_Infor_charge_ot = false;

						if(0x20 & recv_frames_[0].data[1]) msg.bms_flag_Infor_charge_ut = true;	else msg.bms_flag_Infor_charge_ut = false;

						if(0x40 & recv_frames_[0].data[1]) msg.bms_flag_Infor_discharge_ot = true;	else msg.bms_flag_Infor_discharge_ot = false;

						if(0x80 & recv_frames_[0].data[1]) msg.bms_flag_Infor_discharge_ut = true;	else msg.bms_flag_Infor_discharge_ut = false;

						if(0x01 & recv_frames_[0].data[2]) msg.bms_flag_Infor_charge_oc = true;	else msg.bms_flag_Infor_charge_oc = false;

						if(0x02 & recv_frames_[0].data[2]) msg.bms_flag_Infor_discharge_oc = true;	else msg.bms_flag_Infor_discharge_oc = false;

						if(0x04 & recv_frames_[0].data[2]) msg.bms_flag_Infor_short = true;	else msg.bms_flag_Infor_short = false;

						if(0x08 & recv_frames_[0].data[2]) msg.bms_flag_Infor_ic_error = true;	else msg.bms_flag_Infor_ic_error = false;

						if(0x10 & recv_frames_[0].data[2]) msg.bms_flag_Infor_lock_mos = true;	else msg.bms_flag_Infor_lock_mos = false;

						if(0x20 & recv_frames_[0].data[2]) msg.bms_flag_Infor_charge_flag = true;	else msg.bms_flag_Infor_charge_flag = false;

						msg.bms_flag_Infor_hight_temperature = (float)((short)(recv_frames_[0].data[4] << 4 | recv_frames_[0].data[3] >> 4)) / 10;

						msg.bms_flag_Infor_low_temperature = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 8 | recv_frames_[0].data[5])) / 10;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_flag_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					// 负责解析 CAN 总线上的电机编码器反馈消息  （不重要）
					case 0x98C4DCEF:
					{
						yhs_can_msgs::Drive_MCUEcoder_fb msg;
						msg.Drive_fb_MCUEcoder = (int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0]); 

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							Drive_MCUEcoder_fb_pub_.publish(msg);
						}

						break;
					}

					// CAN 总线上的车辆诊断反馈消息   （不重要）
					case 0x98C4EAEF:
					{
						yhs_can_msgs::Veh_Diag_fb msg;
						msg.Veh_fb_FaultLevel = 0x0f & recv_frames_[0].data[0];

						if(0x10 & recv_frames_[0].data[0]) msg.Veh_fb_AutoCANCtrlCmd = true;	else msg.Veh_fb_AutoCANCtrlCmd = false;

						if(0x20 & recv_frames_[0].data[0]) msg.Veh_fb_AutoIOCANCmd = true;	else msg.Veh_fb_AutoIOCANCmd = false;

						if(0x01 & recv_frames_[0].data[1]) msg.Veh_fb_EPSDisOnline = true;	else msg.Veh_fb_EPSDisOnline = false;

						if(0x02 & recv_frames_[0].data[1]) msg.Veh_fb_EPSfault = true;	else msg.Veh_fb_EPSfault = false;

						if(0x04 & recv_frames_[0].data[1]) msg.Veh_fb_EPSMosfetOT = true;	else msg.Veh_fb_EPSMosfetOT = false;

						if(0x08 & recv_frames_[0].data[1]) msg.Veh_fb_EPSWarning = true;	else msg.Veh_fb_EPSWarning = false;

						if(0x10 & recv_frames_[0].data[1]) msg.Veh_fb_EPSDisWork = true;	else msg.Veh_fb_EPSDisWork = false;

						if(0x20 & recv_frames_[0].data[1]) msg.Veh_fb_EPSOverCurrent = true;	else msg.Veh_fb_EPSOverCurrent = false;

						
						
						if(0x10 & recv_frames_[0].data[2]) msg.Veh_fb_EHBecuFault = true;	else msg.Veh_fb_EHBecuFault = false;

						if(0x20 & recv_frames_[0].data[2]) msg.Veh_fb_EHBDisOnline = true;	else msg.Veh_fb_EHBDisOnline = false;

						if(0x40 & recv_frames_[0].data[2]) msg.Veh_fb_EHBWorkModelFault = true;	else msg.Veh_fb_EHBWorkModelFault = false;

						if(0x80 & recv_frames_[0].data[2]) msg.Veh_fb_EHBDisEn = true;	else msg.Veh_fb_EHBDisEn = false;


						if(0x01 & recv_frames_[0].data[3]) msg.Veh_fb_EHBAnguleFault = true;	else msg.Veh_fb_EHBAnguleFault = false;

						if(0x02 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOT = true;	else msg.Veh_fb_EHBOT = false;

						if(0x04 & recv_frames_[0].data[3]) msg.Veh_fb_EHBPowerFault = true;	else msg.Veh_fb_EHBPowerFault = false;

						if(0x08 & recv_frames_[0].data[3]) msg.Veh_fb_EHBsensorAbnomal = true;	else msg.Veh_fb_EHBsensorAbnomal = false;

						if(0x10 & recv_frames_[0].data[3]) msg.Veh_fb_EHBMotorFault = true;	else msg.Veh_fb_EHBMotorFault = false;

						if(0x20 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOilPressSensorFault = true;	else msg.Veh_fb_EHBOilPressSensorFault = false;

						if(0x40 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOilFault = true;	else msg.Veh_fb_EHBOilFault = false;




						if(0x01 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUDisOnline = true;	else msg.Veh_fb_DrvMCUDisOnline = false;

						if(0x02 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUOT = true;	else msg.Veh_fb_DrvMCUOT = false;

						if(0x04 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUOV = true;	else msg.Veh_fb_DrvMCUOV = false;

						if(0x08 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUUV = true;	else msg.Veh_fb_DrvMCUUV = false;

						if(0x10 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUShort = true;	else msg.Veh_fb_DrvMCUShort = false;

						if(0x20 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUScram = true;	else msg.Veh_fb_DrvMCUScram = false;

						if(0x40 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUHall = true;	else msg.Veh_fb_DrvMCUHall = false;

						if(0x80 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUMOSFEF = true;	else msg.Veh_fb_DrvMCUMOSFEF = false;


						if(0x10 & recv_frames_[0].data[5]) msg.Veh_fb_AUXBMSDisOnline = true;	else msg.Veh_fb_AUXBMSDisOnline = false;

						if(0x20 & recv_frames_[0].data[5]) msg.Veh_fb_AuxScram = true;	else msg.Veh_fb_AuxScram = false;

						if(0x40 & recv_frames_[0].data[5]) msg.Veh_fb_AuxRemoteClose = true;	else msg.Veh_fb_AuxRemoteClose = false;

						if(0x80 & recv_frames_[0].data[5]) msg.Veh_fb_AuxRemoteDisOnline = true;	else msg.Veh_fb_AuxRemoteDisOnline = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							Veh_Diag_fb_pub_.publish(msg);
						}


						break;
					}

					default:
						break;
				}

			}

					
		}
	}
}

// CAN 控制节点中的数据发送线程   维持一个定时循环。
void CanControl::sendData()
{
	ros::Rate loop(100);   // 以 100Hz 的频率执行循环


	while(ros::ok())   // 循环体内仅包含休眠，未实现数据发送功能
	{

		loop.sleep();
	}

}

// ************************************************************  很重要  *****************************************************************
// run() 函数是 CAN 控制节点的核心运行函数，负责初始化 ROS 通信、CAN 设备，并启动数据接收线程。
/*
	处理流程：
		1. 初始化 ROS 订阅器和发布器
		2. 打开并配置 CAN 设备
		3. 创建数据接收线程
		4. 启动 ROS 主循环
		5. 节点退出时释放资源
*/
void CanControl::run()
{

	//  1. ROS 订阅器初始化, 初始化 ROS 节点句柄  
	// ctrl_cmd_sub_：订阅控制命令话题，接收速度、转向等控制指令
	// io_cmd_sub_：订阅 IO 控制命令话题，接收灯光、喇叭等控制指令
	// 参数：（1）"ctrl_cmd"、"io_cmd"为话题名称 （2）队列大小：5（最多缓存 5 条未处理消息）  （3）回调函数：ctrl_cmdCallBack、io_cmdCallBack  （4）this：传递类实例指针，使回调函数能访问类成员
	ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);   
	io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);

	// 2. 初始化 ROS 发布器   nh_通常代表 "NodeHandle"，是 ROS 节点的句柄对象
	// 目的是将解析后的 CAN 数据发布到对应话题   参数就是 话题名称和队列大小
	ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb",5);   //  发布控制反馈
	lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb",5);   // 发布 IO 设备反馈
	rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb",5);
	io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb",5);
	odo_fb_pub_ = nh_.advertise<yhs_can_msgs::odo_fb>("odo_fb",5);
	bms_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_Infor_fb>("bms_Infor_fb",5);
	bms_flag_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_Infor_fb>("bms_flag_Infor_fb",5);
	Drive_MCUEcoder_fb_pub_ = nh_.advertise<yhs_can_msgs::Drive_MCUEcoder_fb>("Drive_MCUEcoder_fb",5);
	Veh_Diag_fb_pub_ = nh_.advertise<yhs_can_msgs::Veh_Diag_fb>("Veh_Diag_fb",5);

	// 3. CAN 设备打开与配置
	// 打开CAN设备
	dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);   // 参数1：PF_CAN表示使用CAN协议族，参数2：SOCK_RAW表示原始套接字，直接访问链路层，参数3：CAN_RAW表示使用CAN原始套接字协议
	if (dev_handler_ < 0)   // 发生了错误
	{
		ROS_ERROR(">>open can deivce error!");
		return;
	}
    else
	{
		ROS_INFO(">>open can deivce success!");
	}

	// 获取网络接口索引（重要）
	struct ifreq ifr;   			// ifreq结构体用于存储网络接口的相关信息 ，向内核请求网络接口信息
	std::string can_name("can0");   // 这里假设使用的CAN设备名称为"can0"，可以根据实际情况修改
	strcpy(ifr.ifr_name, can_name.c_str());   // 将can_name转换为C风格字符串并复制到ifr.ifr_name中, 确保内核知道要操作的具体接口
	ioctl(dev_handler_, SIOCGIFINDEX, &ifr);  // 使用ioctl函数获取指定网络接口的索引，将结果存储在ifr.ifr_ifindex中。  
	// （参数1：dev_handler_：CAN socket 句柄， SIOCGIFINDEX：获取接口索引的命令， &ifr：传入 / 传出参数结构）


    // 配置 socket 地址结构， 绑定socket到网络接口（重要）
	struct sockaddr_can addr;               // sockaddr_can 结构体用于存储 CAN 套接字的地址信息， 用于 socket 绑定
	memset(&addr, 0, sizeof(addr));         // 内存初始化，确保结构干净，避免未初始化数据
	addr.can_family = AF_CAN;         		// can_family：地址族，固定为AF_CAN
	addr.can_ifindex = ifr.ifr_ifindex;     // can_ifindex：接口索引（通过ioctl获取）
	int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),sizeof(addr));   // bind函数 将 socket 与特定网络接口关联
	// （参数1：dev_handler_是CAN socket 句柄，参数2：指向sockaddr_can结构体的指针，参数3：地址结构体的大小）
	if (ret < 0)    // 如果绑定失败
	{
		ROS_ERROR(">>bind dev_handler error!\r\n");
		return;
	}

	// 4. 创建接收数据线程 ****************************************************重要，多线程***********************************************************
	boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));      // 使用boost::thread创建独立线程执行recvData()函数，该函数负责循环读取CAN数据并解析
	//	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));

	// 5. 启动 ROS 主循环
	ros::spin();    // 进入ROS主循环，处理回调函数，ros::spin()会阻塞当前线程，处理 ROS 回调函数（如订阅器回调）
	
	close(dev_handler_);   // 节点退出时关闭CAN设备
}

}



// 主函数    初始化 ROS 系统、创建 CAN 控制实例并启动节点运行。
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "yhs_can_control_node");     // 初始化ROS节点

	yhs_tool::CanControl cancontrol;   // 创建CAN控制实例
	cancontrol.run();     // 运行CAN控制节点

	return 0;
}
