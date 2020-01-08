"""
* @file    PatrolLine.py
* @author  Junbin Gao
* @data    2019.8
* @notes   2019neuq电赛四旋翼-巡线拍照题
"""

import sensor, image, time ,math,cpufreq,pyb,random,utime
from pyb import UART,Timer,LED,Pin,SPI

#Global variables Start
#阈值
yellow_thresholds=[(60, 110, -30, 20, 70, 100)]  #黄色异物的阈值
blob_range=[100,60,120,120]                      #条形码拍摄的范围阈值   x,y,w,h

#串口发送数据打包,给单片机
pack_data=bytearray([0xAA,0xAF,0xF2,0x00,
    0x00,0x00,
    0x00,0x00,
    0x00,0x00,
    0x01,0x01,
    0x01,0x00])

#串口发送数据打包,给2号摄像头
pack_data2=bytearray([0xAA,0xAF,0xFC,0x09,
    0x00,0x00,
    0x00,0x00,
    0x00,0x00,
    0x01,0x01,
    0x01,0x00])

#工作模式，超级重要标志位
work_mode = 0xff
#是否转发2号摄像头点数据
send2rd = False


#指示灯
red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)
led_mode  = None    #led指示模式选择
paizhao_led=0       #拍照指示灯标志位，置1闪蓝灯
paizhao_star=0      #拍照闪烁控制位，中间标志位，不用管

#是否需要debug标志位，初始化函数里进行修改，不用管
CompetitionScene=0

#点
class Dot(object):     #点的坐标（x,y）
    x = 0
    y = 0
    last_dot=0    #线点切换时，用来强化判断，防止点线跳变

#线
class singleline_check():     #线的反馈量（左右偏差，角度偏差）
    rho_err   = 40
    theta_err = 0
    line_mode = None

#串口接收
class receive(object):      #串口接收类
    uart_buf = []      #接收存放数组
    _data_len = 0
    _data_cnt = 0
    state = 0

#实例化
Receive=receive()     #串口接收，单片机
Receive2=receive()     #串口接收，摄像头2
dot  = Dot()         #点
singleline_check = singleline_check()  #线
clock = time.clock()                # to process a frame sometimes.

#串口初始化，波特率 115200
uart = UART(3,115200)
uart2= UART(1,115200)
uart.init(115200,timeout_char=1000)
uart2.init(115200,timeout_char=1000)

#Global variables End


def init(is_debug,pixformat,delay_time):
    #关闭串口，防止初始化过程溢出
    uart.deinit()
    uart2.deinit()


    sensor.reset()


    sensor.set_pixformat(sensor.RGB565)    #RGB565

    sensor.set_framesize(sensor.QVGA)      #320*240

    sensor.set_gainceiling(128)            #增益上限 2,4,8,16,32,64,128
    sensor.set_contrast(3)                 #对比度 -3至3
    sensor.set_brightness(0)               #亮度。-3至+3
    sensor.set_saturation(3)               #饱和度。-3至+3
    sensor.set_auto_exposure(True)         #自动曝光

    sensor.skip_frames(time = delay_time)
    sensor.set_auto_gain(False) # 在进行颜色追踪时，必须关闭
    sensor.set_auto_whitebal(False) # 在进行颜色追踪时，必须关闭

    #重新打开串口
    uart.init(115200,timeout_char=1000)
    uart2.init(115200,timeout_char=1000)
    #判断是否debug模式
    global CompetitionScene
    if is_debug==True:
        CompetitionScene=0
    else:
        CompetitionScene=1

#线模式数据转化
def data_translate(data):
    if data>90:
        data=data-180
    return -data

#有效数据和校验，数据个数不够，返回0，正确返回1
def sum_check():
    global pack_data
    lens = len(pack_data)#数据包大小
    if(lens>=4):
        pack_data[3] = lens-5;#有效数据个数
    else:
        return 0
    return 1

#设置其他摄像头模式
def set_other_mode(address,mode):
    global CompetitionScene
    if address==1:
        pack_data2=bytearray([0xAA,0xAF,0xFC,0x09,mode,
        0x00,0x00,0x00,0x03,0x00,0X00,0X01,0x01,0x00])
        uart2.write(pack_data2)
        if CompetitionScene==0:
            if mode == 0x00:
                print("准备拍条形码")
    '''
    #调试用，调节2号摄像头阈值
    if address==2:
        pack_data2=bytearray([0xAA,0xAF,0xAA,0x09,0x01,
        0x01,0x00,0x00,0x03,0x00,0X00,0X01,0x01,0x00])
        uart2.write(pack_data2)

    if address==3:
        pack_data2=bytearray([0xAA,0xAF,0xBB,0x09,0x00,
        0xff,0x00,0x00,0x03,0x00,0X00,0X01,0x01,0x00])
        uart2.write(pack_data2)
    '''

#线检测发送数据打包
def pack_linetrack_data():
    global pack_data,CompetitionScene
    singleline_check.theta_err=data_translate(singleline_check.theta_err)
    #确保偏差范围为0-120
    singleline_check.rho_err = int(singleline_check.rho_err/2)
    pack_data=bytearray([0xAA,0xAF,0xF3,0x00,
        singleline_check.rho_err>>8,singleline_check.rho_err,
        0x00,0x00,
        0x03,singleline_check.line_mode,0X00,0X01,0x01,0x00])
    if CompetitionScene==0:
        print("line,x:%d,theta_err:%d"%(singleline_check.rho_err,singleline_check.theta_err))
    #清零线检测偏移数据和倾角数据，使得在没有检测到线时，输出为零
    singleline_check.rho_err = 80
    singleline_check.theta_err = 0
    if(sum_check()==0):
        print("接收到的数组长度不够哦~")
        return 0
    else:
        uart.write(pack_data)


#状态发送函数
def pack_state(state):
    global pack_data,CompetitionScene
    # 具体情况待定
    pack_data=bytearray([0xAA,0xAF,state,0x00,
    0x00,0x00,
    0x00,0x00,
    0x00,0x00,
    0x01,0x01,
    0x00,0x00])
    if CompetitionScene==0:
        if state==0x02:
            print("发现了黄色色块，飞机声光指示")

    if(sum_check()==0):
        print("接收到的数组长度不够")
        return 0
    else:
        uart.write(pack_data)   #串口输出



#二进制打印函数
def print_hex(bytes):
    l = [hex(int(i)) for i in bytes]
    print(" ".join(l))



sure_receive_pole = 0    #强化判断，确保找到了杆，防止误判提前转弯或降落
#串口接收数据解析
def Receive_Anl(data_buf,num):
    global work_mode,CompetitionScene,sure_receive_pole

    if data_buf[2]==0xFC:
        #设置模块工作模式
        work_mode = data_buf[4]
        if CompetitionScene==0:
            print("单片机来信，切换模式成功，work_mode:0x%x"%work_mode)

    if data_buf[2]==0xAA:
        #调整阈值
        uart2.write(bytearray(data_buf))
        if CompetitionScene==0:
            print("调整大杆阈值,转发成功")
    if data_buf[2]==0xBB:
        #调整阈值
        uart2.write(bytearray(data_buf))
        if CompetitionScene==0:
            print("调整小杆阈值,转发成功")

    if data_buf[2]==0xF2:
        print("2号摄像头:找到杆")
        #大摄像头
        if(work_mode==0x03):
            uart.write(bytearray(data_buf))
            if CompetitionScene==0:
                print("点坐标发送给单片机")
        if(work_mode==0x02):
            sure_receive_pole+=1
            if(sure_receive_pole>=3):
                sure_receive_pole=0
                work_mode = 0x03
        if work_mode==0x06:
            uart.write(bytearray(data_buf))
            if CompetitionScene==0:
                print("点坐标发送给单片机")
        if work_mode==0x05:
            sure_receive_pole+=1
            if(sure_receive_pole>=3):
                sure_receive_pole=0
                work_mode = 0x06
        if work_mode==0x04:
            uart.write(bytearray(data_buf))
            if CompetitionScene==0:
                print("点坐标发送给单片机")
        if(work_mode==0x09):
            uart.write(bytearray(data_buf))
            if CompetitionScene==0:
                print("点坐标发送给单片机")
    if data_buf[2]==0x6F:
        #拍照
        img.save("erweima.jpg")  #自己拍照-二维码
        print("拍二维码ok")
        paizhao_show()           #拍摄指示灯



#串口接收通信协议--单片机
def Receive_Prepare(data):
    if Receive.state==0:
        Receive.uart_buf=[]
        if data == 0xAA:#帧头
            Receive.state = 1
            Receive.uart_buf.append(data) #将数据保存到数组里面
        else:
            Receive.state = 0
    elif Receive.state==1:
        if data == 0xAF:#帧头
            Receive.state = 2
            Receive.uart_buf.append(data) #将数据保存到数组里面
        else:
            Receive.state = 0
    elif Receive.state==2:
        if data <= 0xFF:#模式选择
            Receive.state = 3
            Receive.uart_buf.append(data) #将数据保存到数组里面
        else:
            Receive.state = 0
    elif Receive.state==3:
        if data <= 33:  #数据个数
            Receive.state = 4
            Receive.uart_buf.append(data) #将数据保存到数组里面
            Receive._data_len = data
            Receive._data_cnt = 0
        else:
            Receive.state = 0
    elif Receive.state==4:
        if Receive._data_len > 0:
            Receive. _data_len = Receive._data_len - 1
            Receive.uart_buf.append(data) #将数据保存到数组里面
            if Receive._data_len == 0:
                Receive.state = 5
        else:
            Receive.state = 0
    elif Receive.state==5:
        Receive.state = 0
        Receive.uart_buf.append(data) #将数据保存到数组里面
        Receive_Anl(Receive.uart_buf,Receive.uart_buf[3]+5) #还原数据个数，数据的总个数为6
        Receive.uart_buf=[]#清空缓冲区，准备下次接收数据
    else:
        Receive.state = 0

#串口接收通信协议--摄像头2
def Receive_Prepare2(data2):
    if Receive2.state==0:
        Receive2.uart_buf=[]
        if data2 == 0xAA:#帧头
            Receive2.state = 1
            Receive2.uart_buf.append(data2) #将数据保存到数组里面
        else:
            Receive2.state = 0
    elif Receive2.state==1:
        if data2 == 0xAF:#帧头
            Receive2.state = 2
            Receive2.uart_buf.append(data2) #将数据保存到数组里面
        else:
            Receive2.state = 0
    elif Receive2.state==2:
        if data2 <= 0xFF:#模式选择
            Receive2.state = 3
            Receive2.uart_buf.append(data2) #将数据保存到数组里面
        else:
            Receive2.state = 0
    elif Receive2.state==3:
        if data2 <= 33:  #数据个数
            Receive2.state = 4
            Receive2.uart_buf.append(data2) #将数据保存到数组里面
            Receive2._data_len = data2
            Receive2._data_cnt = 0
        else:
            Receive2.state = 0
    elif Receive2.state==4:
        if Receive2._data_len > 0:
            Receive2. _data_len = Receive2._data_len - 1
            Receive2.uart_buf.append(data2) #将数据保存到数组里面
            if Receive2._data_len == 0:
                Receive2.state = 5
        else:
            Receive2.state = 0
    elif Receive2.state==5:
        Receive2.state = 0
        Receive2.uart_buf.append(data2) #将数据保存到数组里面
        Receive_Anl(Receive2.uart_buf,Receive2.uart_buf[3]+5) #还原数据个数，数据的总个数为6
        Receive2.uart_buf=[]#清空缓冲区，准备下次接收数据
    else:
        Receive2.state = 0

#读取串口缓存--单片机&摄像头2
def uart_read_buf():
    global CompetitionScene,send2rd
    i = 0
    buf_size = uart.any() #判断是否有串口数据--单片机
    while i<buf_size:
        char =uart.readchar()
        Receive_Prepare(char) #读取串口数据--单片机
        i = i + 1
    if CompetitionScene==0 and buf_size!=0:
        #print("收到单片机串口数据，数据包大小为：:%d"%(buf_size))    #通信异常时才考虑打开这句话，通信正常时注释掉
        buf_size=0

    i2 = 0
    buf_size2 = uart2.any() #判断是否有串口数据--摄像头2
    while i2<buf_size2:
        char2 =uart2.readchar()
        if send2rd:
            Receive_Prepare2(char2) #读取串口数据，判断完再根据情况转发
        i2 = i2 + 1
    if CompetitionScene==0 and buf_size2!=0:
        #print("收到摄像头2串口数据，数据包大小为：:%d"%(buf_size2))    #通信异常时才考虑打开这句话，通信正常时注释掉
        buf_size2=0


#角度弧度转化
def angle_to_radian(angle):
    return ((math.pi*angle)/180)



# tick
def tick(timer):            # 回调函数
    global CompetitionScene,led_mode,paizhao_led,paizhao_star
    if led_mode == "line":
        red_led.off()
        green_led.off()
        blue_led.off()
        green_led.on()
    if led_mode == None:
        red_led.off()
        green_led.off()
        blue_led.off()

    if paizhao_led==1:
        paizhao_star+=1
        if paizhao_star<10:
            red_led.off()
            green_led.off()
            blue_led.on()
        elif paizhao_star<20:
            red_led.off()
            green_led.off()
            blue_led.off()
        elif paizhao_star<30:
            red_led.off()
            green_led.off()
            blue_led.on()
        elif paizhao_star<40:
            red_led.off()
            green_led.off()
            blue_led.off()
            paizhao_led=0
            paizhao_star=0


tim = Timer(4, freq=20)      # 定时器4，频率20次/秒

tim.callback(tick)          # 设置回调函数


#拍照闪灯函数
def paizhao_show():
    global paizhao_led,paizhao_star
    paizhao_led=1
    paizhao_star=0




#识别线、十字（直线）
#is_need_line:是否需要线
#if_need_shu:如果需要线，找竖线还是找横线
#if_need_dot：是否需要找十字（point）
#ten_angle:十字的夹角要求，如果为None,就是寻找任意角度，无要求，如果有角度，就是找这个角度上下的角度，上下范围在函数内部可调
#is_shizi :True 要出头的   False要不出头的   None都行   注意，如果这个参数不是None，会大大降低帧率
#返回值“line”or“point“or None
def find_lines_dot(img,is_need_line,if_shu,if_need_dot,ten_angle=None,is_shizi=None):
    global CompetitionScene
    lines=img.find_lines(threshold = 6000, theta_margin = 20, rho_margin = 20)#霍夫变换找直线
    shu_flag=0
    line_count=0
    shu_k=0
    shu_b=0
    hen_k=0
    hen_b=0
    shu_rho=0
    shu_theta=0
    line=[[0,0],[0,0],[0,0],[0,0]]
    for l in lines:
        if if_need_dot==False:      #如果不需要找点
            if if_shu==True:
                heng_shu_flag=abs(90-l.theta())>30
            if if_shu==False:
                heng_shu_flag=abs(90-l.theta())<45
            if (shu_flag==0) and (heng_shu_flag):   #判断是竖线，且只要一条，得到竖线的斜率和截距
                shu_flag= 1
                shu_rho = l.rho()
                shu_theta = l.theta()
                if(l.rho()==0):
                    shu_b=0
                    if(l.theta()!=0):
                        shu_k=math.tan(angle_to_radian(l.theta()-90))
                else:
                    if(l.theta()<90) and (l.theta()>0):
                        shu_k = -1.0/math.tan(angle_to_radian(l.theta()))
                        shu_b = l.rho()/math.sin(angle_to_radian(l.theta()))
                    elif(l.theta()==90):
                        shu_k = 0
                        shu_b = l.rho()
                    elif(l.theta()>90):
                        shu_k = math.tan(angle_to_radian(l.theta()-90))
                        shu_b = l.rho()/math.cos(angle_to_radian(l.theta()-90))
                if CompetitionScene==0:
                    img.draw_line(l.line(), color = (255,255,255))

    if if_need_dot==False and shu_flag==1:    #如果不需要找点，并且找到了一条竖线
        if(shu_theta==0):                     #与y轴平行线
            if if_shu==True:                  #找的是竖线
                singleline_check.rho_err=abs(shu_rho)
                singleline_check.theta_err=0
                singleline_check.line_mode=0x01

        elif(shu_theta==90):                  #与x轴平行
            if if_shu==False:                 #找的是横线
                singleline_check.line_mode=0x02
                singleline_check.rho_err=abs(shu_rho)
                singleline_check.theta_err=0

        else:
            if shu_k!=0:
                if if_shu==False:
                    if shu_theta<=90:
                        shu_theta=shu_theta+90
                    else:
                        shu_theta=shu_theta-90
                    singleline_check.line_mode = 0x02   #横线
                    singleline_check.rho_err=int(160*shu_k+shu_b)
                    singleline_check.theta_err=shu_theta

        #竖线的位置串口发送
        if is_need_line==True:
            pack_linetrack_data()
            return "line"


'''
------以下：查找黄色异物
is_find_yellow  是否发现黄色异物，用于给飞机声光指示
is_find_code    是否该拍摄条形码，用于自身拍照
'''
always_find_blobs=0
is_find_yellow=False
is_find_code=False
def find_blobs(img):
    global CompetitionScene,always_find_blobs,is_find_code,is_find_yellow,yellow_thresholds
    is_find_blob=False
    blobs=img.find_blobs(yellow_thresholds, pixels_threshold=20, area_threshold=20, merge=True)
    for blob in blobs:
        if(is_find_blob==False):
            if CompetitionScene==0:
                img.draw_rectangle(blob_range[0],blob_range[1],blob_range[2],blob_range[3],color = (255,255,255), thickness = 2, fill = False)
                img.draw_cross(blob.cx(), blob.cy())
            is_find_blob=True
            if is_find_yellow==False:
                always_find_blobs+=1
            if is_find_yellow==True:
                if(blob.cx()>blob_range[0] and blob.cx()<blob_range[0]+blob_range[2] and blob.cy()>blob_range[1] and blob.cy()<blob_range[1]+blob_range[3]):
                    is_find_code = True   #二维码在合适的视野
    if blobs==None:
        always_find_blobs=0
    if always_find_blobs>5:
        is_find_yellow=True  #找到了黄色
        always_find_blobs=0


# Main Funtion Start
work_mode=0x00   #上电自动初始化

while(True):

    clock.tick()

    if work_mode==0x00:
        init(True,"RGB",1000)  #初始化
        work_mode=0xfe         #待机


    if work_mode!=0xff:
        img = sensor.snapshot()
        img = img.lens_corr(1.0)  #校准畸变


    if work_mode==0x01:    #找线，找黄色色块
        send2rd=False
        #找电线
        if find_lines_dot(img=img,is_need_line=True,if_shu=False,if_need_dot=False,ten_angle=None,is_shizi=None)=="line":
            led_mode = "line"
        else:
            led_mode = None
        if(is_find_yellow==False):
            find_blobs(img)
        else:
            pack_state(0x02)         #让飞机声光指示
            pack_state(0x02)         #让飞机声光指示
            work_mode=0x11

    if work_mode==0x11:    #找线，准备拍照-条形码
        send2rd=False
        #找电线
        if find_lines_dot(img=img,is_need_line=True,if_shu=False,if_need_dot=False,ten_angle=None,is_shizi=None)=="line":
            led_mode = "line"
        else:
            led_mode = None
        if(is_find_code==False):
            find_blobs(img)
        else:
            set_other_mode(1,0x02)      #告诉摄像头2，重新初始化，避免误判出发杆为目标杆
            img.save("tiaoxingma.jpg")  #自己拍照-条形码
            print("条形码拍摄ok")
            paizhao_show()              #拍照指示灯
            work_mode=0x02


    if work_mode==0x02:       #找完条形码，找电线
        send2rd=True       #开启搜索杆，但是不发给单片机，强化判断后进入模式0x03
        if find_lines_dot(img=img,is_need_line=True,if_shu=False,if_need_dot=False,ten_angle=None,is_shizi=None)=="line":  #找电线
            led_mode = "line"
        else:
            led_mode = None

    if work_mode==0x03:       #下面摄像头找到了电线杆，转发数据即可,绕杆旋转
        led_mode = None
        send2rd=True


    if work_mode==0x04:       #转完飞机给标志，关闭转发，只找线
         #找电线
        if find_lines_dot(img=img,is_need_line=True,if_shu=False,if_need_dot=False,ten_angle=None,is_shizi=None)=="line":
            send2rd=False
            work_mode = 0x07


    if work_mode==0x07:
        if find_lines_dot(img=img,is_need_line=True,if_shu=False,if_need_dot=False,ten_angle=None,is_shizi=None)=="line":
            led_mode = "line"
        else:
            led_mode = None


    if work_mode==0x05:      #飞机飞行一段路程，给切换模式，开启搜索杆
        send2rd=True		#开启搜索杆，但是不发给单片机，强化判断后进入模式0x06，飞机则地降落
        if find_lines_dot(img=img,is_need_line=True,if_shu=False,if_need_dot=False,ten_angle=None,is_shizi=None)=="line":  #找电线
            led_mode = "line"
        else:
            led_mode = None


    if work_mode==0x06:    #只转发杆模式
        send2rd=True

    if work_mode==0x08:   #调阈值时找线
        send2rd = False
        if find_lines_dot(img=img,is_need_line=True,if_shu=False,if_need_dot=False,ten_angle=None,is_shizi=None)=="line":
           led_mode = "line"
        else:
           led_mode = None


    if work_mode==0x09:   #调阈值时找杆
        send2rd = True


    if work_mode==0xfe:   #待机
        send2rd = False


    if CompetitionScene==0 and work_mode!=0xff:
        #img.draw_string(0, 0, "FPS:%d"%(clock.fps()),color=0)
        img.draw_string(0, 0, "mode:0x%x"%(work_mode),color=0,scale=1.0,mono_space=False,x_spacing=3)
        #lcd.display(img,roi=[0,0,128,160])

    uart_read_buf()  #接收串口数据
# Main Funtion End