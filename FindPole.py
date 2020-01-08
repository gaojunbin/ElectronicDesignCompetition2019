"""
* @file    FindPole.py
* @author  Junbin Gao
* @data    2019.8
* @notes   2019neuq电赛四旋翼-标杆题
"""
import sensor, image, time ,math,cpufreq,pyb,random,utime,lcd
from pyb import UART,Timer,LED,Pin,SPI
CompetitionScene=0     #比赛现场标志位,用于控制是否关闭可视化
work_mode=0xff         #相机工作模式

#Global variables Start
#条形码及二维码数量
my_barcode_count=0       
my_qrcode_count=0

pole_high=60        #大杆阈值  大的阈值可以稍稍宽泛一些
small_pole_high=65  #小杆阈值，最终决定了返回的杆的状况，这个阈值很关键

pack_data=bytearray([0xAA,0xAF,0xF2,0x00,
    0x00,0x00,
    0x00,0x00,
    0x00,0x00,
    0x01,0x00,
    0x00,0x00])


#指示灯
red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)
led_mode  = None


class Dot(object):
    x = 0
    y = 0
    last_dot=0
class receive(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0
Receive=receive()
clock = time.clock()
dot  = Dot()
uart = UART(3,115200)
uart.init(115200,timeout_char=1000)

#Global variables End


def init(is_debug,delay_time):
    uart.deinit()
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_contrast(3)
    sensor.set_brightness(-3)
    sensor.set_auto_exposure(True)
    sensor.skip_frames(time = delay_time)
    sensor.set_auto_whitebal(False)
    uart.init(115200,timeout_char=1000)
    lcd.init()
    global CompetitionScene
    if is_debug==True:
        CompetitionScene=0
    else:
        CompetitionScene=1

def sum_check():
    global pack_data
    lens = len(pack_data)
    if(lens>=4):
        pack_data[3] = lens-5;
    else:
        return 0
    return 1

def pack_state(state):
    global pack_data,CompetitionScene
    pack_data=bytearray([0xAA,0xAF,state,0x00,
    0x00,0x00,
    0x00,0x00,
    0x00,0x00,
    0x01,0x01,
    0x00,0x00])
    if CompetitionScene==0:
        if state==0x6F:
            print("可以拍二维码了")

    if(sum_check()==0):
        print("接收到的数组长度不够")
        return 0
    else:
        uart.write(pack_data)
def pack_block_data():
    global pack_data,CompetitionScene
    pack_data=bytearray([0xAA,0xAF,0xF2,0x00,
        dot.x>>8,dot.x,
        dot.y>>8,dot.y,
        0x00,0x00,
        0x01,0x01,0x01,0x00])
    if CompetitionScene==0:
        print("左右位移:%d,像素点大小:%d"%(dot.y,dot.x))
    if(sum_check()==0):
        print("接收到的数组长度不够")
        return 0
    else:
        uart.write(pack_data)
def print_hex(bytes):
    l = [hex(int(i)) for i in bytes]
    print(" ".join(l))


def Receive_Anl(data_buf,num):
    global work_mode,CompetitionScene,pole_high,small_pole_high
    if data_buf[2]==0xFC:
        work_mode = data_buf[4]
        if CompetitionScene==0:
            print("切换模式成功，现在的work_mode:0x%x"%work_mode)

    if data_buf[2]==0xAA:
        pole_high = (data_buf[4]<<8)+data_buf[5]
        if pole_high>255:
            pole_high=255
        if CompetitionScene==0:
            print("杆阈值:%d"%pole_high)
    if data_buf[2]==0xBB:
        small_pole_high = (data_buf[4]<<8)+data_buf[5]
        if small_pole_high>255:
            small_pole_high=255
        if CompetitionScene==0:
            print("小杆阈值:%d"%small_pole_high)

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
    i = 0
    buf_size = uart.any()
    while i<buf_size:
        char =uart.readchar()
        Receive_Prepare(char)
        if CompetitionScene==0:
            #print("char:%d,buf_size:%d"%(char,buf_size))
            pass
        i = i + 1

# 角度弧度转化
def angle_to_radian(angle):
    return ((math.pi*angle)/180)
def radian_to_angle(radian):
    return (180*radian/math.pi)
# 判断是否在范围里     
def is_range(x,x1,x2):
    if x1>=x2:
        x_temp = x2
        x2 = x1
        x1 = x_temp
    if x>=x1 and x<=x2:
        return True
    else:
        return False


# 定时器部分
def tick(timer):            # we will receive the timer object when being called
    global CompetitionScene,led_mode
    if led_mode == "gan":
        red_led.off()
        green_led.off()
        blue_led.off()
        green_led.on()
    if led_mode == None:
        red_led.off()
        green_led.off()
        blue_led.off()


tim = Timer(4, freq=20)      # create a timer object using timer 4 - trigger at 1Hz

tim.callback(tick)          # set the callback to our tick function


# find pole
is_really_find_pole = False
continue_find_pole_time = 0
need_pole=None
small_blobs = None
need_small_blob = None
def find_pole(img):
    global CompetitionScene,is_really_find_pole,continue_find_pole_time,need_pole,small_blobs,need_small_blob
    gan_thresholds=[(0,pole_high)]    #大杆阈值
    small_pole_thresholds=[(0,small_pole_high)]    #小杆阈值
    longest=0
    need_pole = None
    is_find_pole=False
    blobs=img.find_blobs(gan_thresholds, area_threshold=3500,pixel_threshold=10, y_stride=10,merge=False)   #找大杆
    for blob in blobs:             #找到大杆里竖直方向最长的
        if blob.h()>=longest:
            longest = blob.h()
            need_pole = blob
    if(need_pole!=None):      #至少找到了一根大杆
        if(need_pole.h()>=120 and need_pole.w()<=320 and need_pole.w()>=12):   #对大杆的长宽限制，排除干扰
            if need_pole.cx()-70>0:
                my_roi1 = need_pole.cx()-70
            else:
                my_roi1 = 0
            if need_pole.cx()+70<320:
                my_roi2 = 140
            else:
                my_roi2 = (320-need_pole.x()-1)
            my_roi=[my_roi1,0,my_roi2,20]    #对找到的大杆区域，上部进行搜索小杆
            small_blobs = img.find_blobs(small_pole_thresholds, roi= my_roi,area_threshold=25, y_stride=1,x_stride=1,merge=False)   #找小杆
            small_longest=0
            for small_blob in small_blobs:   #找到小杆里竖直方向最长的
                if small_blob.h()>=small_longest:
                    small_longest = small_blob.h()
                    need_small_blob = small_blob
            if need_small_blob!=None:       #至少找到了一根小杆
                if need_small_blob.w()>12 and need_small_blob.w()<140 and need_small_blob.h()>=10:   #对小杆进行限制
                    if need_small_blob.x()>20 and need_small_blob.x()+need_small_blob.w()<300:  #去除边缘
                        is_find_pole = True
                        if CompetitionScene==0:
                            #print(need_small_blob.cx(),need_small_blob.w())
                            img.draw_rectangle(need_small_blob.rect(),color = (255,255,255), thickness = 2, fill = False)
                            img.draw_cross(need_small_blob.cx(), need_small_blob.cy())
    return is_find_pole

# Main Function Start
work_mode=0x00
while(True):
    clock.tick()
    if work_mode==0x00:
        init(is_debug=True,delay_time=1000)
        work_mode=0x01
    if work_mode==0x10:
        init(is_debug=True,delay_time=1000)
        work_mode=0x11
    if work_mode!=0xff:
        img = sensor.snapshot().lens_corr(strength = 1.5, zoom = 1.15)
    if work_mode==0x01:
        if find_pole(img)==True:
            led_mode="gan"
            dot.x = need_small_blob.w()
            dot.y = 320-need_small_blob.cx()
            pack_block_data()
        else:
            led_mode=None
    if work_mode==0x02:
        if my_barcode_count<1:
            my_barcode_count+=1
        else:
            work_mode=0x10
    if work_mode==0x11:
        if find_pole(img)==True:
            led_mode="gan"
            dot.x = need_small_blob.w()
            dot.y = 320-need_small_blob.cx()
            pack_block_data()
            if(dot.y>120 and dot.y<220 and my_qrcode_count<1):
                my_qrcode_count+=1
                pack_state(0x6F)   #可以拍二维码了
        else:
            led_mode=None
    if CompetitionScene==0 and work_mode!=0xff:
        img.draw_string(100, 0, "mode:0x%x"%(work_mode),color=0,scale=1.0,mono_space=False,x_spacing=1)
        img.draw_string(100, 20, "pole_high:%d"%(pole_high),color=0,scale=1.0,mono_space=False,x_spacing=1)
        img.draw_string(100, 40, "small_pole_high:%d"%(small_pole_high),color=0,scale=1.0,mono_space=False,x_spacing=1)
        #img_display = img.copy()
        #img_display.rotation_corr(z_rotation=90)
        lcd.display(img,roi=[90,0,128,160])
        #lcd.display(img_display,roi=[90,0,128,160])
    uart_read_buf()
# Main Function End