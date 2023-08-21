import os
import time
import socket
import serial
import codecs
import pigpio
import threading
from datetime import datetime
from multiprocessing import Process
class ShipMode:
    NO_CONTROL = 0
    MANUEL_CONTROL = 1
    AUTO_CONTROL = 2
    REMOTE_CONTROL = 3
    E_CONTROL = 5
class USVData:
    def __init__(self):
        self.utctime = 0.0
        self.lon = 0
        self.lat = 0.0
        self.course = 0.0
        self.speed = 0.0
        self.diffstatus = 0.0
        self.heading = 0.0
        self.roll = 0.0
        self.pitch=0.0
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
#         actual rudder angle
        self.rudder_pos = 0.0
#         set rudder angle
        self.aim_rudder = 0.0
        self.mode = ShipMode.NO_CONTROL
        self.voltage = 0.0
        self.current = 0
#         use pwm value as the rpm here
        self.propellerpwm = 0.0
        self.control_time = 0.0
Rudder Control and Read State
def s16_int(s16:list):
    s16_list = ("0x"+" 0x".join(s16)).split(" ")
    num=0
    for i in range(len(s16_list)):
        num+= int(s16_list[i], 16)*pow(256, i)
    return num

def string2list(string):
    return [string[i:i+2] for i in range(0, len(string), 2)] 

def decoderudder(statestring):
    state = string2list(statestring)
    pos = int(s16_int(state[5:7])*0.0879) - 180
#     need to decode to angle
    speed = state[7:9]
#     need to decode to speed
    load = state[9:11]
    vol = state[11]
    temp = state[12]
#     just return actual rudder angle
    return pos

def checksum_rudder(checkData_rudder):
    dataList = [int(x, 16) for x in checkData_rudder]
    sumnum = sum(dataList)
    while sumnum > 0xff:
        sumnum = sumnum - 0xff -0x01
    sumnum = ~sumnum & 0xff
    sumnum = str(hex(sumnum))[2:].zfill(2)
    return sumnum


def rudder_control():
    while True:
        rudder_command_list = ['ff','ff','fe','12','83','2a','06','01','00','54','09','00','e8','03','02','00','54','09','00','e8','03','a9']
        rudderpos = rudder2hex(usvdata.aim_rudder)
#         trans rudder value to hex number
        rudderpos = 180 + rudderpos
        aim_step = hex(int(rudderpos/0.0879))[2:].zfill(4)
        hex_rudder = [aim_step[-2:],aim_step[-4:-2]]
        
        rudder_command_list[8:10] = hex_rudder
        rudder_command_list[15:17] = hex_rudder
        rudder_command_list[-1] = checksum_rudder(rudder_command_list[2:-1])
        rudder_command_list = ''.join(rudder_command_list)
        rudder_command = codecs.decode(rudder_command_list, "hex")
        sevro_port.write(rudder_command)
        time.sleep(.1)
def readrudder():
    while True:
        try:            
            statecommand = b'\xFF\xFF\xFE\x06\x82\x38\x08\x01\x02\x36'
            sevro_port.write(statecommand)
            num = sevro_port.inWaiting()
            if num >1:
                rudder_string = sevro_port.read(num).hex()
            if len(rudder_string)>=28:
                rudderstate = rudder_string[:28]
                usvdata.rudder_pos = decoderudder(rudderstate)
        except:
            pass
        time.sleep(.1)
compass
def bcd2float(string):
    if int(string[:2])==10:
        value = -(int(string[2:4])+int(string[4:])*0.01)
    else:
        value = int(string[2:4])+int(string[4:])*0.01
    return value

def decodecompass(string):
    if len(string)==56:
        string1 = string[:28]
        string2 = string[28:]
        stringlist = [string1,string2]
        for string in stringlist:
            if string.startswith("680d0084"):
                usvdata.pitch = bcd2float(string[8:14])
                usvdata.roll = bcd2float(string[14:20])
                usvdata.heading = int(string[20:22])*100 + bcd2float(string[20:-2])
#decodes compass
def readcompass():
    while True:
        try:
            compass_string = compass_port.read(size = 28).hex()
            compass_state = decodeah200(compass_string)
        except:
            pass
        time.sleep(.1)
GPS
def ReadGps():
    global gpsstring
    gpsstring = b''
    gps_port.flushInput()
    while True:
        count = gps_port.inWaiting()
        if count != 0:
            gpsstring += gps_port.read(count)
        #print(gpsstring)
        try:
            gpsstring = gpsstring.decode('utf-8')
            # delete the first part no used the data
            gpsstring = gpsstring.replace("\r","").replace("\n","")
            gpsstring = r'$'+r'$'.join(gpsstring.split(r"$")[1:])
            # find rmc and gga string
            rmc_list = [''.join(a) for a in re.findall(r'(\$GNRMC)(.*?)(\*)([A-Za-z0-9]{2})',gpsstring)]
            gga_list = [''.join(a) for a in re.findall(r'(\$GNGGA)(.*?)(\*)([A-Za-z0-9]{2})',gpsstring)]
            if len(rmc_list)>=1:
                rmc_string = rmc_list[-1]
                try:
                    gps_info = rmc2gps(rmc_string)
                    usvdata.utctime  = gps_info['time']
                    usvdata.lon = gps_info['lon']
                    usvdata.lat = gps_info['lat']
                    usvdata.speed = gps_info['speed']
                    usvdata.course = gps_info['course']
                except:
                    pass
                gpsstring = ''.join(gpsstring.split(rmc_string)[1:])
        except:
            pass
        gpsstring = bytes(gpsstring,'utf-8') 

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
Trans usvdata to hex
def make6845():
    global now_control_mode
    hexlist = "68 44 00 44 00 68 03 C8 45 01 00 00 00 21 11 11 14 46 24 45 4A E9 D0 06 4E 43 10 D3 01 3E 00 00 00 00 00 64 00 00 00 00 00 63 00 00 00 00 00 62 00 EF 09 00 00 00 00 32 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 88 04 50 00 ED 00 E0 16".split(" ")
    
    hextime = usvdata.utctime.replace(".",":").replace(" ",":").replace("-",":").split(":")[1:]

    hexlon = hex(int(round(usvdata.lon,7)*10000000))[2:].zfill(8)
    hexlon = [hexlon[-2:],hexlon[-4:-2],hexlon[-6:-4],hexlon[-8:-6]]


    hexlat = hex(int(round(usvdata.lat,7)*10000000))[2:].zfill(8)
    hexlat = [hexlat[-2:],hexlat[-4:-2],hexlat[-6:-4],hexlat[-8:-6]]

    hexspeed =  hex(int(round(usvdata.speed,2)*100))[2:].zfill(4)
    hexspeed = [hexspeed[-2:],hexspeed[-4:-2]]

    hexcourse =  hex(int(round(usvdata.course,1)))[2:].zfill(4)
    hexcourse = [hexcourse[-2:],hexcourse[-4:-2]]
    
    if rudder>=0:
        hexrudder = hex(int(round(usvdata.rudder_pos,1)))[2:].zfill(4)
        hexrudder = [hexrudder[-2:],hexrudder[-4:-2]]
    else:

        hexrudder = hex(int(round(usvdata.rudder_pos,1)))[3:].zfill(4)
        hexrudder = [hexrudder[-2:],hexrudder[-4:-2]]

    hexrpm = hex(int(round(usvdata.propellerpwm,1)))[2:].zfill(4)
    hexrpm = [hexrpm[-2:],hexrpm[-4:-2]]
    
    hexheading =  hex(int(round(usvdata.heading,1)))[2:].zfill(4) 
    hexheading = [hexheading[-2:],hexheading[-4:-2]]

    if usvdata.acc_x>=0:
        hexacc_x =  hex(int(round(usvdata.acc_x,1)*10))[2:].zfill(4)
        hexacc_x = [hexacc_x[-2:],hexacc_x[-4:-2]]
    else:

        hexacc_x =  hex(int(round(usvdata.acc_x,1)*10))[3:].zfill(4)
        hexacc_x = [hexacc_x[-2:],hexacc_x[-4:-2]]

    if usvdata.acc_y>=0:
        
        hexacc_y =  hex(int(round(usvdata.acc_y,1)*10))[2:].zfill(4)
        hexacc_y = [hexacc_y[-2:],hexacc_y[-4:-2]]
    else:
        hexacc_y =  hex(int(round(usvdata.acc_y,1)*10))[3:].zfill(4)
        hexacc_y = [hexacc_y[-2:],hexacc_y[-4:-2]]

    if usvdata.acc_z >=0:
        
        hexacc_z =  hex(int(round(usvdata.acc_z,1)*10))[2:].zfill(4)
        hexacc_z = [hexacc_z[-2:],hexacc_z[-4:-2]]
    else:
        hexacc_z =  hex(int(round(usvdata.acc_z,1)*10))[3:].zfill(4)
        hexacc_z = [hexacc_z[-2:],hexacc_z[-4:-2]]
    
    
    hexmode = hex(int(round(usvdata.mode,1)))[2:].zfill(2)
    hexmode = [hexmode]

    hex_rudder_angle = hex(usvdata.aim_rudder)
    
    hexlist[13:19] = hextime
    hexlist[20:24] = hexlon
    hexlist[25:29] = hexlat
    hexlist[29:31] = hexspeed
    hexlist[31:33] = hexacc_x
    hexlist[33:35] = hexacc_y
    hexlist[35:37] = hexacc_z
    hexlist[55:57] = hexheading 
#     use pwm value
    hexlist[69:71] = hexrpm
    hexlist[71:73] = hexrpm
    hexlist[73:75] = hexrudder
    hexlist[75:77] = hexcourse

    hexlist[-2] = checksum(hexlist[9:-2])
    hexline = ' '.join(hexlist)
    #print(hexlist[57],430098)
    return hexline
# Function for Propeller Controlling
def propeller_control():
    while True:
        try:
            if usvdata.pwm > 2000:
                pi.set_servo_pulsewidth(ESC_GPIO_PIN, 2000) 
            elif usvdata.pwm < 1000:
                pi.set_servo_pulsewidth(ESC_GPIO_PIN, 1000) 
            else:
                pi.set_servo_pulsewidth(ESC_GPIO_PIN, usvdata.propellerpwm) 
        except:
            pass  
# Function for Broadcastting State
def BroadcastState():
    while True:
        try:
            
            data = make6845()
            datastr = ''.join(data.split(" "))
            datastr = codecs.decode(datastr, "hex")
            udpclient.sendto(datastr, server)
        except:
            pass
        try:
            usvstate = {'type':'point','state':'test','shipid':1,'shipname':"yellowusv",'lon': usvdata.lon, 'lat': usvdata.lat, 'time':usvdata.utctime,'speed':usvdata.speed,'heading':usvdata.heading, 'course':usvdata.course,'roll':usvdata.roll,'pitch':usvdata.pitch,'voltage':usvdata.voltage, 'current':usvdata.current, 'rpm':usvdata.propellerpwm, 'rudder':usvdata.rudder_pos}
            udpclient.sendto(str(StateDict).encode('utf-8'), webserver)
        except:
            pass
        time.sleep(.1)
#  Funciton for writing to local log file
def write2log():
    while True:
        filename = "yellowusv"+datetime.utcnow().strftime('%Y%m%d %H:%M:%S.%f')[:-16]+".txt"
        if not os.path.exists(filename):
            usvlog = open(filename, "a+")
            usvlog.write(','.join(list(usvdata.__dict__.keys()))+"\n")
        else:
            usvlog = open(filename, "a+")
            valuelists = list(usvdata.__dict__.values())
            valuelists = [str(a) for a in valuelists]
            usvlog.write(','.join(valuelists)+"\n")
        time.sleep(.1)
def CommandProcessing():
    global udp_recieve,rudder_angle,now_control_mode
    propeller_pwm_num = 0
    rudder_pwm_num = 1
    manual_command_time = 0
    while True: 
        try:
            udpserver.setblocking(False)  
            udp_recieve,__ = udpserver.recvfrom(1024)
            command_time = time.time()
            #         if send hex data
            present_command = ''.join(['%02X ' % b for b in udp_recieve]).strip(" ")
            beforecommand = present_command.split(" ")
            udpserver.setblocking(True) 
            if len(beforecommand)==17:
                usv.control_time = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-5]
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
                    usvdata.mode=2

                if not present_command:
                    usvdata.propellerpwm = 1000
                    usvdata.aim_rudder = 0
                    print("stop control1")
                #if (checksum(commandlist[9:-2]) == commandlist[-2])&(present_command):
                if present_command:             
                    del present_command  
                    rpm1 = int(commandlist[10],16)
                    rpm2 = int(commandlist[11],16)

                    rpm1dir = int(commandlist[13],16)
                    rpm2dir = int(commandlist[14],16)

                    propeller_pwm_num = 0
                    rudder_pwm_num = 1
                    
                    rudder_angle = int(commandlist[12],16)
                    usvdata.aim_rudder = rudder_angle-40
                    
                    board.set_pwm_duty(rudder_pwm_num,7.5+(rudder_angle-40)/18)
                    if rpm1+rpm2==0:
                        usvdata.propellerpwm = 1000
                    else:
                        if rpm1dir==1:
                            usvdata.propellerpwm = (rpm1+rpm2)*5
#                             can not be reverse rotation temporarily
                        if rpm1dir==0:
                            usvdata.propellerpwm = 1000                       
                else:
                    print("stop control")
                    usvdata.propellerpwm = 1000
                    usvdata.aim_rudder = 0
                    #now_control_mode = 0
        except:
            print("stop control3")
            usvdata.propellerpwm = 1000
            usvdata.aim_rudder = 0
        time.sleep(.1)
# gpio init
pi = pigpio.pi() 
ESC_GPIO_PIN = 27  
PWM_FREQUENCY = 66  
MIN_PWM = 1000
MAX_PWM = 2000

pi.set_servo_pulsewidth(ESC_GPIO_PIN, MIN_PWM)   
time.sleep(5)  
# initial usvdata
usvdata = USVData()
# configure for the serials
sevro_port = serial.Serial(
    # port="/dev/ttyS0",
    port="/dev/ttyUSB0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    )

compass_port = serial.Serial(
    port="/dev/ttyUSB2",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout = 0.2
    )

gps_port = serial.Serial(
    port="/dev/ttyAMA0",
    baudrate=38400,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    )

# create udp client for broadcast usv statue
udpclient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpclient.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
# broadcast through local network
server = ("10.49.7.255",9001)
# boradcast to webserver
webserver = ("43.143.137.184",10000)


# create Server for Recieve Control Command
udpserver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpserver.bind(("0.0.0.0", 9002))
udpserver.settimeout(3.0)
udpserver.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,100)
# use multiporcess for the import sensors
process_list = []
pgps = Process(target=ReadGps) #use multiprocessing for gps serial
pgps.start()
process_list.append(pgps)

pcom = Process(target=readcompass) #use multiprocessing for compass serial
pcom.start()
process_list.append(pcom)

pcommand = Process(target=CommandProcessing) #use multiprocessing for command processing
pcommand.start()
process_list.append(pcommand)
threads = []

t0 = threading.Thread(target=propeller_control)
t0.start()
threads.append(t0)

t1 = threading.Thread(target=rudder_control)
t1.start()
threads.append(t1)

t2 = threading.Thread(target=BroadcastState)
t2.start()
threads.append(t2)                      
        
t3 = threading.Thread(target=write2log)
t3.start()
threads.append(t3)
