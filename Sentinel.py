"""
* @file    Sentinel.py
* @author  Junbin Gao
* @data    2019.8
* @notes   2019neuq电赛四旋翼-定点题
"""

import sensor, image, time ,math,cpufreq,pyb,random,utime,lcd
from pyb import UART,Timer,LED,Pin,SPI
CompetitionScene=0    #比赛现场标志位,用于控制是否关闭可视化
work_mode=0xff        #相机工作模式

#Global variables Start
red_thresholds=[(40, 70, 50, 80, 10, 70)]   #红色圆心的阈值,由于干扰项少,可以将范围尽可能给大,提高识别率
pack_data=bytearray([0xAA,0xAF,0xF2,0x00,   #串口发送数据包
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x01,0x01,
	0x01,0x00])
class Dot(object):         #点模式类
	x = 0
	y = 0
	last_dot=0
class receive(object):     #串口接收类
	uart_buf = []
	_data_len = 0
	_data_cnt = 0
	state = 0
#创建对象
Receive=receive()
clock = time.clock()
dot  = Dot()
uart = UART(3,115200)
uart.init(115200,timeout_char=1000)
#Global variables End

# Init Function Start
# @param
# is_debug:  选则debug or release模式
# pixformat: 图像模式选则
# delay_time:初始化延时时间
# @note      相机初始化,设定一些参数,注意加上延时,否则会影响自动曝光
def init(is_debug,pixformat,delay_time):
	uart.deinit()
	sensor.reset()
	if pixformat=="GRAY":
		sensor.set_pixformat(sensor.GRAYSCALE)
	elif pixformat=="RGB":
		sensor.set_pixformat(sensor.RGB565)
	sensor.set_framesize(sensor.QQVGA)
	sensor.set_gainceiling(128)
	sensor.set_contrast(3)
	sensor.set_brightness(0)
	sensor.set_saturation(3)
	sensor.set_auto_exposure(True)
	sensor.skip_frames(time = delay_time)
	sensor.set_auto_gain(False)
	sensor.set_auto_whitebal(False)
	uart.init(115200,timeout_char=1000)
	global CompetitionScene
	if is_debug==True:
		CompetitionScene=0
	else:
		CompetitionScene=1
# 和校验
# @param  None
# @note   校验串口接收信息,这里没用到
def sum_check():
	global pack_data
	lens = len(pack_data)
	if(lens>=4):
		pack_data[3] = lens-5;
	else:
		return 0
	return 1
# 点数据打包
# @param  None
def pack_block_data():
	global pack_data,CompetitionScene
	pack_data=bytearray([0xAA,0xAF,0xF2,0x00,
		dot.x>>8,dot.x,
		dot.y>>8,dot.y,
		0x00,0x00,
		0x01,0x01,0x01,0x00])
	if CompetitionScene==0:
		print("point,x:%d,y:%d"%(dot.x,dot.x))
	dot.x = 160
	dot.y = 120
	if(sum_check()==0):
		print("the length of receive message is wrong!")
		return 0
	else:
		uart.write(pack_data)
# 十六进制打印
# @param
# bytes:  十六进制数组or列表
def print_hex(bytes):
	l = [hex(int(i)) for i in bytes]
	print(" ".join(l))
# 串口接收
# @param  记不大清了~
def Receive_Anl(data_buf,num):
	global work_mode,CompetitionScene
	'''
	sum = 0
	i = 0
	while i<(num-1):
		sum = sum + data_buf[i]
		i = i + 1
	sum = sum%256
	if sum != data_buf[num-1]:
		return
	'''
	'''
	if data_buf[2]==0x01:
		print("dot okzhi
	if data_buf[2]==0x02:
		print("line ok!")
	'''
	if data_buf[2]==0xFC:
		print("单片机发送了信息!")
		work_mode = data_buf[4]
		if CompetitionScene==0:
			print("Set work mode success! now_work_mode:0x%x"%work_mode)
	if data_buf[2]==0x5F:
		print("条形码拍摄完毕")
		work_mode=0x02
		uart.write(bytearray(data_buf))
	'''
	if data_buf[2]==0x6F:
		print("出现二维码")
		uart.write(bytearray(data_buf))
	'''
	if data_buf[2]==0x7F:
		print("二维码拍摄完毕")
		work_mode = 0x03
		uart.write(bytearray(data_buf))
# 串口数据接收
# @param
# data:  串口收到的字节信息
def Receive_Prepare(data):
	if Receive.state==0:
		Receive.uart_buf=[]
		if data == 0xAA:
			Receive.state = 1
			Receive.uart_buf.append(data)
		else:
			Receive.state = 0
	elif Receive.state==1:
		if data == 0xAF:
			Receive.state = 2
			Receive.uart_buf.append(data)
		else:
			Receive.state = 0
	elif Receive.state==2:
		if data <= 0xFF:
			Receive.state = 3
			Receive.uart_buf.append(data)
		else:
			Receive.state = 0
	elif Receive.state==3:
		if data <= 33:
			Receive.state = 4
			Receive.uart_buf.append(data)
			Receive._data_len = data
			Receive._data_cnt = 0
		else:
			Receive.state = 0
	elif Receive.state==4:
		if Receive._data_len > 0:
			Receive. _data_len = Receive._data_len - 1
			Receive.uart_buf.append(data)
			if Receive._data_len == 0:
				Receive.state = 5
		else:
			Receive.state = 0
	elif Receive.state==5:
		Receive.state = 0
		Receive.uart_buf.append(data)
		Receive_Anl(Receive.uart_buf,Receive.uart_buf[3]+5)
		Receive.uart_buf=[]
	else:
		Receive.state = 0
def uart_read_buf():
	global CompetitionScene
	i = 0
	buf_size = uart.any()
	while i<buf_size:
		char =uart.readchar()
		Receive_Prepare(char)
		if CompetitionScene==0:
			print("char:0x%x"%(char))
		i = i + 1
	if CompetitionScene==0 and buf_size!=0:
		print("buf_size:%d"%(buf_size))
		buf_size=0
# 找色块
# @param
# img:    输入的图像
# @return 返回点坐标
def find_blobs(img):
	global CompetitionScene
	is_find_blob=False
	blobs=img.find_blobs(red_thresholds, pixels_threshold=20, area_threshold=20, merge=True)
	for blob in blobs:
		if(is_find_blob==False):
			dot.x = blob.cx()
			dot.y = blob.cy()
			is_find_blob = True
			pack_block_data()
			if(CompetitionScene==0):
				img.draw_cross(blob.cx(), blob.cy())
#Main Funtion Start				
while(True):
	clock.tick()
	if work_mode==0x00:
		init(is_debug=True,pixformat="RGB",delay_time=700)
		work_mode=0x01
	if work_mode!=0xff:
		img = sensor.snapshot()
	if work_mode==0x01:
		find_blobs(img)
	uart_read_buf()
#Main Funtion End		