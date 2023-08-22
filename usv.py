from Expansion_Board import DFRobot_Expansion_Board_IIC as Board
import serial
import time
import re
import threading
import socket
import threading
import time
import socket
import codecs
import threading, time
from sys import exit
import RPi.GPIO as GPIO


# for extension board
board = Board(1, 0x10)    # Select i2c bus 1, set address to 0x10
def board_detect():
    l = board.detecte()
    print("Board list conform:")
    print(l)

''' print last operate status, users can use this variable to determine the result of a function call. '''
def print_board_status():
    if board.last_operate_status == board.STA_OK:
        print("board status: everything ok")
    elif board.last_operate_status == board.STA_ERR:
        print("board status: unexpected error")
    elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
        print("board status: device not detected")
    elif board.last_operate_status == board.STA_ERR_PARAMETER:
        print("board status: parameter error")
    elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
        print("board status: unsupport board framware version")

board_detect()    # If you forget address you had set, use this to detected them, must have class instance
board.set_addr(0x10)
  # Set board controler address, use it carefully, reboot module to make it effective

while board.begin() != board.STA_OK:    # Board begin and check board status
    print_board_status()
    print("board begin faild")
    time.sleep(2)
    print("board begin success")



# set up for pwm and adc
board.set_pwm_enable()                # Pwm channel need external power
board.set_adc_enable()
  # board.set_pwm_disable()
# board.set_pwm_frequency(1000)         
# Set frequency to 1000HZ, Attention: PWM voltage depends on independent power supply
board.set_pwm_frequency(50)    # set up pwd period to 20ms

# pwm number
propeller_pwm_num = 0
rudder_pwm_num = 1

# for decoding ah200 and gps
def bcd2float(string):
    if int(string[:2])==10:
        value = -(int(string[2:4])+int(string[4:])*0.01)
    else:
        value = int(string[2:4])+int(string[4:])*0.01
    return value

def decodeah200(ah200string):
    if len(ah200string)==56:
        ah200string_01 = ah200string[:28]
        ah200string_02 = ah200string[28:]
        ah200string_list = [ah200string_01,ah200string_02]
        for string in ah200string_list:
            if string.startswith("770d0054"):
                acc_x = 0.01*bcd2float(string[8:14])
                acc_y = 0.01*bcd2float(string[14:20])
                acc_z = 0.01*bcd2float(string[20:-2])
            if string.startswith("770d0084"):
                pitch = bcd2float(string[8:14])
                roll = bcd2float(string[14:20])
                heading = int(string[20:22])*100 + bcd2float(string[20:-2])
                print('heading:',heading)
    return dict(acc_x=acc_x,acc_y=acc_y,acc_z=acc_z,pitch=pitch,roll=roll,heading=heading)




compass_info = ''
class CompassSerialPort:
    def __init__(self,port,buand):
        super(CompassSerialPort, self).__init__()
        self.port=serial.Serial(port,buand)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()
    def port_open(self):
        if not self.port.isOpen():
            self.port.open()
    
    def port_close(self):
        self.port.close()
    def read_data(self):
        global compass_info
        while True:
            ah200string = ''
            command_direction = b'\x77\x04\x00\x04\x08'
            command_accelerate = b'\x77\x04\x00\x54\x58'
            # command_anglespeed = b'\x77\x04\x00\x50\x54'
            command_list = [command_direction, command_accelerate]
#             for decode angle and accelerate
            
            for command in command_list:
                self.port.write(command)
                num = self.port.inWaiting()
                if num >1:
                    serialdata = self.port.read(num)
                            # serialdata = serial_port.read()
                    ah200string += serialdata.hex()
                    time.sleep(.05)
            try:
                compass_info = decodeah200(ah200string)
                # print(compass_info,430098)
            except:
                pass
            time.sleep(.1)
            


# for decoding gps            
def rmc2gps(rmc_string):
    rmc = rmc_string.split(",")
    lat = float(rmc[3])
    lat = int(lat/100) + (lat -int(lat/100)*100)/60
    lon = float(rmc[5])
    lon = int(lon/100) + (lon -int(lon/100)*100)/60
    try:
        speed = float(rmc[7])
    except:
        speed = 0
    try:
        course = float(rmc[8])
    except:
        course = 0
    time = "20"+rmc[9][-2:]+'-'+rmc[9][-4:-2]+'-'+rmc[9][:2]+ " " +rmc[1][:2]+":"+rmc[1][2:4]+":"+rmc[1][4:]
    gps_info = dict(lon=lon, lat=lat, time=time, speed=speed, course=course)
    return gps_info            


# gpsserial
gpsstring = b''
gps_info = ''
gpsstates = []
class GPSSerialPort:
    message='' 
    def __init__(self,port,buand):
        super(GPSSerialPort, self).__init__()
        self.port=serial.Serial(port,buand)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()
    def port_open(self):
        if not self.port.isOpen():
            self.port.open()
    
    def port_close(self):
        self.port.close()
    
    def read_data(self):
        global gpsstring
        global gps_info
        self.port.flushInput()
        while True:
            gpsstring += self.port.read()
            #print(gpsstring)
            #print(str(gpsstring),430099)
            try:
                gpsstring = gpsstring.decode('utf-8')
                # delete the first part no used the data
                gpsstring = gpsstring.replace("\r","").replace("\n","")
                gpsstring = r'$'+r'$'.join(gpsstring.split(r"$")[1:])
                
                # find rmc and gga string
                rmc_list = [''.join(a) for a in re.findall(r'(\$GNRMC)(.*?)(\*)([A-Za-z0-9]{2})',gpsstring)]
                gga_list = [''.join(a) for a in re.findall(r'(\$GNGGA)(.*?)(\*)([A-Za-z0-9]{2})',gpsstring)]
                # print(rmc_list,430099)
                for rmc_string in rmc_list:
                    gps_info = rmc2gps(rmc_string)
                    # print(gps_info)
                    
                #     delete decoded data
                    gpsstring = ''.join(gpsstring.split(rmc_string)[1:])
                    # gpsstring=gpsstring.replace(rmc_string,"")
#                     delete gga string
                for gga_string in gga_list:
                    gpsstring = ''.join(gpsstring.split(gga_string)[1:])
                    # gpsstring=gpsstring.replace(gga_string,"")
#                 change gps string to bytes~!!
                gpsstring = bytes(gpsstring,'utf-8')  
                # time.sleep(.5)
            except:
                gpsstring = bytes(gpsstring,'utf-8') 
                pass
            
            
            
def checksum(checkData):
    dataList = [int(x, 16) for x in checkData]
    sumnum = sum(dataList)
    if sumnum > 0xff:
        sumnum = sumnum & 0xff
    sumnum = str(hex(sumnum))[2:].zfill(2)
    return sumnum

class ControlMode:
    NO_CONTROL = 0
    MANUEL_CONTROL = 1
    AUTO_CONTROL = 2
    REMOTE_CONTROL = 3
    E_CONTROL = 5

rudder=0
rotation_speed = 0
now_control_mode = 0
def make6845(state):
    global now_control_mode
    hexlist = "68 44 00 44 00 68 03 C8 45 01 00 00 00 21 11 11 14 46 24 45 4A E9 D0 06 4E 43 10 D3 01 3E 00 00 00 00 00 64 00 00 00 00 00 63 00 00 00 00 00 62 00 EF 09 00 00 00 00 32 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 88 04 50 00 ED 00 E0 16".split(" ")
    
    hextime = state['time'].replace(".",":").replace(" ",":").replace("-",":").split(":")[1:]
    
    lon = state['lon']
    hexlon = hex(int(round(lon,7)*10000000))[2:].zfill(8)
    hexlon = [hexlon[-2:],hexlon[-4:-2],hexlon[-6:-4],hexlon[-8:-6]]

    lat = state['lat']
    hexlat = hex(int(round(lat,7)*10000000))[2:].zfill(8)
    hexlat = [hexlat[-2:],hexlat[-4:-2],hexlat[-6:-4],hexlat[-8:-6]]

    speed = state['speed']
   # print("speed:",speed)
    hexspeed =  hex(int(round(speed,2)*100))[2:].zfill(4)
   # print("hexspeed_1:",hexspeed)
    hexspeed = [hexspeed[-2:],hexspeed[-4:-2]]
    #print("hexspeed_2:",hexspeed)

    course = state['course']
    hexcourse =  hex(int(round(course,1)))[2:].zfill(4)
    hexcourse = [hexcourse[-2:],hexcourse[-4:-2]]
    
    
    
    if rudder>=0:
        hexrudder = hex(int(round(rudder,1)))[2:].zfill(4)
        hexrudder = [hexrudder[-2:],hexrudder[-4:-2]]
    else:

        hexrudder = hex(int(round(rudder,1)))[3:].zfill(4)
        hexrudder = [hexrudder[-2:],hexrudder[-4:-2]]
    


    if rotation_speed>=0:
        hexrotation = hex(int(round(rotation_speed,1)))[2:].zfill(4)
        hexrotation = [hexrotation[-2:],hexrotation[-4:-2]]
    else:
        hexrotation = hex(int(round(rotation_speed,1)))[3:].zfill(4)
        hexrotation = [hexrotation[-2:],hexrotation[-4:-2]]
    
    heading = state['heading']
    hexheading =  hex(int(round(heading,1)))[2:].zfill(4)
    
    hexheading = [hexheading[-2:],hexheading[-4:-2]]

    acc_x = state['acc_x']
    if acc_x>=0:
        hexacc_x =  hex(int(round(acc_x,1)*10))[2:].zfill(4)
        hexacc_x = [hexacc_x[-2:],hexacc_x[-4:-2]]
    else:

        hexacc_x =  hex(int(round(acc_x,1)*10))[3:].zfill(4)
        hexacc_x = [hexacc_x[-2:],hexacc_x[-4:-2]]
        

    acc_y = state['acc_y']
    if acc_y>=0:
        
        hexacc_y =  hex(int(round(acc_y,1)*10))[2:].zfill(4)
        hexacc_y = [hexacc_y[-2:],hexacc_y[-4:-2]]
    else:
        hexacc_y =  hex(int(round(acc_y,1)*10))[3:].zfill(4)
        hexacc_y = [hexacc_y[-2:],hexacc_y[-4:-2]]

    acc_z = state['acc_z']
    if acc_z >=0:
        
        hexacc_z =  hex(int(round(acc_z,1)*10))[2:].zfill(4)
        hexacc_z = [hexacc_z[-2:],hexacc_z[-4:-2]]
    else:
        hexacc_z =  hex(int(round(acc_z,1)*10))[3:].zfill(4)
        hexacc_z = [hexacc_z[-2:],hexacc_z[-4:-2]]
    
    print("time:",hextime)
    print("lat:",hexlat)
    print("lon:",hexlon)
    print("heading:",hexheading)
    print("course:",hexcourse)
    print("speed:",hexspeed)
    print("acc_x:",hexacc_x)
    print("acc_y:",hexacc_y)
    print("acc_z:",hexacc_z)
    
    
    
    hexmode = hex(int(round(now_control_mode,1)))[2:].zfill(2)
   # print("hexmode_1:",hexmode)
    hexmode = [hexmode]
   # print("hexmode_2:",hexmode)
    
#     read io for rpm 
#     read adc for rudder angle and current
    try:
        hex_rudder_angle = hex(rudder_angle)
    except:
        hex_rudder_angle = hex(0)
    
    hexlist[13:19] = hextime
    hexlist[20:24] = hexlon
    hexlist[25:29] = hexlat
    hexlist[29:31] = hexspeed
    hexlist[31:33] = hexacc_x
    hexlist[33:35] = hexacc_y
    #print("hexacc_y: ", hexacc_y)
    hexlist[35:37] = hexacc_z
    #print("heading_2:",hexlist[55:57])
    hexlist[55:57] = hexheading 
    hexlist[69:71] = hexrotation
    hexlist[71:73] = hexrotation
    hexlist[73:75] = hexrudder
    hexlist[75:77] = hexcourse
    #print("heading:",hexheading)
    
    print("hexrudder=",hexrudder)
    print("control_mode_2",hexlist[57:58])  
    
    # use static checksum 
    # hexlist[-2] = checksum(hexlist[9:-2])
    hexline = ' '.join(hexlist)
    #print(hexlist[57],430098)
    return hexline


# for udp sending and recieving
udpclient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# server = ("192.168.1.211",8922)
server = ("10.49.7.255",9001)
# server = ("<broadcast>",9001)
#server = ("0.0.0.0",9001)
# server = ("",9001)
state_last = ''
state_present = ''
udp_recieve = ''
# add this for broadcast
udpclient.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
udpclient.sendto("test".encode('utf-8'), server)
def SEND():
    global state_last
    global state_present
    while True:
        # print("enter here: ", state_last)
        # print("enter here: ", state_present)
#         send ship status
        try:
            gpsinfo = {**gps_info}
        except:
            gpsinfo = dict(lon=114,lat=30,speed=1,course=1,time="1970-01-01 00:00:00.10")
        #print("compassinfo",**compassinfo)
        try:
            compassinfo = { **compass_info}

        except:
            compassinfo = dict(acc_x=0,acc_y=0,acc_z=0,pitch=0,heading=0,roll=0)
        state_present = {**gpsinfo, **compassinfo}

#         if (state_last != state_present):
        data = make6845(state_present)
# data = str(state_present)
        #print("data:",data)
        #print("state_present:",state_present)
#         trans string data to hex
        datalist = data.split(" ")
        
        
        datastr = ''.join(datalist)
        

        print("datastr:",datastr)
        datastr = codecs.decode(datastr, "hex")
        udpclient.sendto(datastr, server)
        #print("send data: "+str(time.time())+","+str(datastr))

        state_last = state_present
            # print(data)
            # del(statedict)
        time.sleep(.1)


        
udpserver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpserver.bind(("0.0.0.0", 9002))
udpserver.settimeout(3.0)
udpserver.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,100)

# def RECV():
#     global udp_recieve
#     while True:
# #         receive command from server and trans to command to ship
#         udp_recieve,__ = udpserver.recvfrom(1024)
#         command = udp_recieve.decode('utf-8')
#         commandlist = command.split(" ")
#         if checksum(commandlist[9:-2]) == commandlist[-2]:
# #             set up pwm for rudder and rpm
#             rpm1 = int(commandlist[10],16)
#             rpm2 = int(commandlist[11],16)

#             rudder_angle = int(commandlist[12],16)

#             rpm1dir = int(commandlist[13],16)
#             rpm2dir = int(commandlist[14],16)
#             propeller_pwm_num = 0
#             rudder_pwm_num = 1
#             board.set_pwm_duty(propeller_pwm_num,8.2+(rpm1)*(10-8.2)/100) 
#             board.set_pwm_duty(rudder_pwm_num,7.5+(rudder_angle-40)/18) 
#         print(rpm1)
    



def RECV():
    global udp_recieve,rudder_angle,now_control_mode
    # set up for pwm and adc
    board.set_pwm_enable()                # Pwm channel need external power
    board.set_adc_enable()
  # board.set_pwm_disable()
# board.set_pwm_frequency(1000)         
# Set frequency to 1000HZ, Attention: PWM voltage depends on independent power supply
    board.set_pwm_frequency(50)    # set up pwd period to 20ms 
    #udpserver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #udpserver.bind(("0.0.0.0", 9001))

# pwm number
    propeller_pwm_num = 0
    rudder_pwm_num = 1
    manual_command_time = 0
    while True: 

        try:
            udpserver.setblocking(False)  
            udp_recieve,__ = udpserver.recvfrom(1024)
            # udpserver.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,100)
            # receive command from server and trans to command to ship
            command_time = time.time()
            #         if send hex data
            present_command = ''.join(['%02X ' % b for b in udp_recieve]).strip(" ")
            #     if send assic
            #present_command = udp_recieve.decode('utf-8') 
            print("present_command",present_command)
            print('430097')
            beforecommand = present_command.split(" ")
            udpserver.setblocking(True) 
            print(beforecommand)
            #print('430098')
            if len(beforecommand)==17:
                #print('430099')
                #     bytes --> string
                #         fun 02H
                # 68 06 00 06 00 68 0C 0F 02 01 00 00 28 00 00 29 16
                print(beforecommand)
                if int(beforecommand[7],16)==4:
                    manual_command_time = time.time()
                    manual_commandlist = beforecommand
                if int(beforecommand[7],16)==1:
                    auto_command_time = time.time()
                    auto_commandlist = beforecommand
                if manual_command_time >= command_time - 3:
                    commandlist = manual_commandlist
                    print("manual control")
                else:
                    commandlist = auto_commandlist  
                    print("auto control")
                now_control_mode = int(commandlist[9],16)
                print("commandlist: ",commandlist)
                if (int(commandlist[9],16) == 34 or int(commandlist[9],16) == 5):
                    now_control_mode = 2
                    
                # print(commandlist,"hh")
                # print(now_control_mode)
                if not present_command:
                    board.set_pwm_duty(propeller_pwm_num,7.1) 
                    board.set_pwm_duty(rudder_pwm_num,7.5)
                    print("stop control1")
                #if (checksum(commandlist[9:-2]) == commandlist[-2])&(present_command):
                if present_command:             
                    del present_command  
        #             set up pwm for rudder and rpm
        #    control Priority 
        #             if command from pad
        #                 manual control
        #             if command from pc
        #                 auto control
                    rpm1 = int(commandlist[10],16)
                    rpm2 = int(commandlist[11],16)

                    rudder_angle = int(commandlist[12],16)

                    rpm1dir = int(commandlist[13],16)
                    rpm2dir = int(commandlist[14],16)

                    propeller_pwm_num = 0
                    rudder_pwm_num = 1
                    board.set_pwm_duty(rudder_pwm_num,7.5+(rudder_angle-40)/18)
                    if rpm1+rpm2==0:
                        board.set_pwm_duty(propeller_pwm_num,7.1)
                    else:
                        if rpm1dir==1:
                            board.set_pwm_duty(propeller_pwm_num,7.6+((rpm1+rpm2)/2)*(9.3-7.6)/100) 
                        if rpm1dir==2:
                            board.set_pwm_duty(propeller_pwm_num,6.5-((rpm1+rpm2)/2)*(6.5-5)/100) 
                        if rpm1dir==0:
                            board.set_pwm_duty(propeller_pwm_num,7.1)                         
                    print(commandlist)
                else:
                    print("stop control2")
                    #now_control_mode = 0
        except:
            print("stop control3")
            # board.set_pwm_duty(propeller_pwm_num,8.3)
            board.set_pwm_duty(propeller_pwm_num,7.1) 
            board.set_pwm_duty(rudder_pwm_num,7.5)
        time.sleep(1.2)
def RAFB():
    while True:
        global rudder
        val0 = board.get_adc_value(board.A0) # channel A0 is readed
        print('val0=',val0)
        if val0>2000 and val0<2940:    
            rudder=40-(2940-val0)/8.675
        else:
            if val0<2000:
                rudder=(val0+90)/8.675+40
            else:
                rudder=(val0-2940)/8.675+40
        print("channel:A0, rudder:%5.1f" %rudder)
        time.sleep(2)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(27,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)  

def VE():
    #while True:
        n=1
        t=0
        while True:
            global rotation_speed
            channel = GPIO.wait_for_edge(27, GPIO.RISING,timeout=10000)
            if channel:
                dt =time.time()-t
                t = time.time()
                rotation_speed = int(1/dt/16*60)
                # print(dt)
                print('Velocity:',rotation_speed)         
            n+=1     
        

# for using threading compass
            
threads = []
Compass_Port="/dev/ttyUSB0"   #串口
Compass_baudRate=115200       #波特率
Compass_Serial=CompassSerialPort(Compass_Port,Compass_baudRate)
t0=threading.Thread(target=Compass_Serial.read_data)
t0.start()
threads.append(t0)

time.sleep(5)
GPS_Port="/dev/ttyUSB1"   #串口
GPS_baudRate=38400       #波特率
# threads = []
gps_Serial=GPSSerialPort(GPS_Port,GPS_baudRate)

t1=threading.Thread(target=gps_Serial.read_data)
t1.start()
threads.append(t1)


time.sleep(5)
t2 = threading.Thread(target=SEND)
t2.start()
threads.append(t2)
time.sleep(3)

t3 = threading.Thread(target=RECV)
t3.start()
threads.append(t3)

t4 = threading.Thread(target=RAFB)
t4.start()
threads.append(t4)                      
        
t5 = threading.Thread(target=VE)
t5.start()
threads.append(t5)

print("success")