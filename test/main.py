import time
import subprocess
import threading
import SocketServer 
import RPi.GPIO as GPIO
from SocketServer import StreamRequestHandler as SRH
from time import ctime  
from PCA9685 import PCA9685
import os

offset = 0
count = 0
flag = 1

pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

#Set the Horizontal servo parameters
HPulse = 800  #Sets the initial Pulse
HStep = 0      #Sets the initial step length
pwm.setServoPulse(0,HPulse)
# pwm.setRotationAngle(0, 40)
#Set the vertical servo parameters
VPulse = 1500  #Sets the initial Pulse
VStep = 0      #Sets the initial step length
pwm.setServoPulse(1,VPulse)
# pwm.setRotationAngle(1, 100)
    
time.sleep(5)
cmd = "hostname -I | cut -d\' \' -f1"
host = subprocess.check_output(cmd,shell = True )
print(host)
#host = '192.168.6.107'
port = 8000
addr = (host,port)  

class Servers(SRH): 
    def handle(self): 
        global HStep,VStep,flag 
        print("got connection from ",self.client_address)
        self.wfile.write('connection %s:%s at %s succeed!' % (host,port,ctime()))  
        while True:  
            data = self.request.recv(1024)  
            if not data:   
                break  
            if data == "Stop":
                HStep = 0
                VStep = 0
                print("Stop")
            elif data == "Forward":
                print("Forward")
            elif data == "Backward":
                print("Backward")
            elif data == "TurnLeft":
                print("TurnLeft")
            elif data == "TurnRight":
                print("TurnRight")
            elif data == "Up":
                HStep = -5
                print("HStep = ", HStep)
            elif data == "Down":
                HStep = 5
                print("HStep = ", HStep)
            elif data == "Left":
                VStep = 5
                print("VStep = ", VStep)
            elif data == "Right":
                VStep = -5
                print("VStep = ", VStep)
            elif data == "BuzzerOn":
                print("BuzzerOn")
            elif data == "BuzzerOff":
                print("BuzzerOff")
            else:
                value = 0
                try:
                    value = int(data)
                    if(value >= 0 and value <= 100):
                        print(value)
                        # Ab.setPWMA(value);
                        # Ab.setPWMB(value);
                except:
                    print("Command error")
            print(data)   
            #print "recv from ", self.client_address[0]  
            self.request.send(data)  

def timerfunc():
	global HPulse,VPulse,HStep,VStep,pwm
	
	if(HStep != 0):
		HPulse += HStep
		if(HPulse >= 2500): 
			HPulse = 2500
		if(HPulse <= 500):
			HPulse = 500
		#set channel 2, the Horizontal servo
		pwm.setServoPulse(0,HPulse)    
		
	if(VStep != 0):
		VPulse += VStep
		if(VPulse >= 2500): 
			VPulse = 2500
		if(VPulse <= 500):
			VPulse = 500
		#set channel 3, the vertical servo
		pwm.setServoPulse(1,VPulse)   
		# Update each LED color in the buffer.

	global t        #Notice: use global variable!
	t = threading.Timer(0.02, timerfunc)
	t.start()
	
t = threading.Timer(0.02, timerfunc)
t.setDaemon(True)
t.start()

def cam():
    os.system('DISPLAY=:0.0 gst-launch-1.0 nvarguscamerasrc ! \'video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=(fraction)30/1\' ! nvoverlaysink -e')    

tx = threading.Thread(target = cam) 
tx.setDaemon(True)
tx.start()

print('server is running....'  )
server = SocketServer.ThreadingTCPServer(addr,Servers)  
server.serve_forever()  
