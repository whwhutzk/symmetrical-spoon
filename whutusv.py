from Expansion_Board import DFRobot_Expansion_Board_IIC as Board
import serial
import time
import re
import pandas as pd
import threading
import socket
import threading
import time
import socket
import threading, time
from sys import exit

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
#                 print(compass_info)
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
        
        while True:
            gpsstring += self.port.read()
            # print(gpsstring)
#             print(str(gpsstring))
            try:
                gpsstring = gpsstring.decode('utf-8')
                # delete the first part no used the data
                gpsstring = gpsstring.replace("\r","").replace("\n","")
                gpsstring = r'$'+r'$'.join(gpsstring.split(r"$")[1:])

                # find rmc and gga string
                rmc_list = [''.join(a) for a in re.findall(r'(\$GNRMC)(.*?)(\*)([A-Za-z0-9]{2})',gpsstring)]
                gga_list = [''.join(a) for a in re.findall(r'(\$GNGGA)(.*?)(\*)([A-Za-z0-9]{2})',gpsstring)]
                for rmc_string in rmc_list:
                    gps_info = rmc2gps(rmc_string)
#                     print(gps_info)
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
                pass
            
            
            
def checksum(checkData):
    dataList = [int(x, 16) for x in checkData]
    sumnum = sum(dataList)
    if sumnum > 0xff:
        sumnum = sumnum & 0xff
    sumnum = str(hex(sumnum))[2:].zfill(2)
    return sumnum


def make6845(state):
    hexlist = "68 44 00 44 00 68 01 0C 45 01 00 00 00 21 11 11 14 46 24 45 4A E9 D0 06 4E 43 10 D3 01 3E 00 00 00 00 00 64 00 00 00 00 00 63 00 00 00 00 00 62 00 EF 09 00 00 00 00 32 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 88 04 50 00 ED 00 E0 16".split(" ")
    
    hextime = state['time'].replace(".",":").replace(" ",":").replace("-",":").split(":")[1:]
    
    lon = state['lon']
    hexlon = hex(int(round(lon,7)*10000000))[2:]
    hexlon = [hexlon[-2:],hexlon[-4:-2],hexlon[-6:-4],hexlon[-8:-6]]

    lat = state['lat']
    hexlat = hex(int(round(lat,7)*10000000))[2:]
    hexlat = [hexlat[-2:],hexlat[-4:-2],hexlat[-6:-4],hexlat[-8:-6]]

    speed = state['speed']
    hexspeed =  hex(int(round(speed,2)*10))[2:].zfill(2)

    course = state['course']
    hexcourse =  hex(int(round(course,1)*10))[2:].zfill(4)
    hexcourse = [hexcourse[-2:],hexcourse[-4:-2]]

    heading = state['heading']
    hexheading =  hex(int(round(heading,1)*10))[2:].zfill(4)
    hexheading = [hexheading[-2:],hexheading[-4:-2]]

    acc_x = state['acc_x']
    hexacc_x =  hex(int(round(acc_x,1)*10))[2:].zfill(4)
    hexacc_x = [hexacc_x[-2:],hexacc_x[-4:-2]]

    acc_y = state['acc_y']
    hexacc_y =  hex(int(round(acc_y,1)*10))[2:].zfill(4)
    hexacc_y = [hexacc_y[-2:],hexacc_y[-4:-2]]

    acc_z = state['acc_z']
    hexacc_z =  hex(int(round(acc_z,1)*10))[2:].zfill(4)
    hexacc_z = [hexacc_z[-2:],hexacc_z[-4:-2]]
    
#     read io for rpm 
#     read adc for rudder angle and current

    hexlist[13:19] = hextime
    hexlist[20:24] = hexlon
    hexlist[25:29] = hexlat
    hexlist[29:31] = hexspeed
    hexlist[33:35] = hexacc_x
    hexlist[35:37] = hexacc_y
    hexlist[37:39] = hexacc_z
    hexlist[57:59] = hexheading
    hexlist[75:77] = hexcourse
    hexlist[-2] = checksum(hexlist[9:-2])
    hexline = ' '.join(hexlist)
    return hexline


# for udp sending and recieving
udpclient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# server = ("192.168.1.211",8922)
server = ("<broadcast>",6004)
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

        state_present = {**gps_info, **compass_info}
#         if (state_last != state_present):
        data = make6845(state_present)
        # data = str(state_present)
            # print("data:",data)
        udpclient.sendto(data.encode('utf-8'), server)
        state_last = state_present
            # print(data)
            # del(statedict)
        time.sleep(.1)



        
udpserver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpserver.bind(("0.0.0.0", 8001))

def RECV():
    global udp_recieve
    while True:
#         receive command from server and trans to command to ship
        udp_recieve,__ = udpserver.recvfrom(1024)
        command = udp_recieve.decode('utf-8')
        commandlist = command.split(" ")
        if checksum(commandlist[9:-2]) == commandlist[-2]:
#             set up pwm for rudder and rpm
            rpm1 = int(commandlist[10],16)
            rpm2 = int(commandlist[11],16)

            rudder_angle = int(commandlist[12],16)

            rpm1dir = int(commandlist[13],16)
            rpm2dir = int(commandlist[14],16)
            propeller_pwm_num = 0
            rudder_pwm_num = 1
            board.set_pwm_duty(propeller_pwm_num,8.2+((rpm1+rpm1)/2)*(9.2-8.2)/100) 
            board.set_pwm_duty(rudder_pwm_num,7.5+rudder_angle*10/80) 
            print(commandlist)
        
        
    

    
    
    
# for using threading compass
            
threads = []
Compass_Port="/dev/ttyUSB0"   #串口
Compass_baudRate=115200       #波特率
Compass_Serial=CompassSerialPort(Compass_Port,Compass_baudRate)
t0=threading.Thread(target=Compass_Serial.read_data)
t0.start()
threads.append(t0)

GPS_Port="/dev/ttyUSB1"   #串口
GPS_baudRate=38400       #波特率
# threads = []
gps_Serial=GPSSerialPort(GPS_Port,GPS_baudRate)

t1=threading.Thread(target=gps_Serial.read_data)
t1.start()
threads.append(t1)

t2 = threading.Thread(target=SEND)
t2.start()
threads.append(t2)


t3 = threading.Thread(target=RECV)
t3.start()
threads.append(t3)