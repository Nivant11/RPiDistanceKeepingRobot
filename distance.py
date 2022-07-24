import time as time
import RPi.GPIO as GPIO
from multiprocessing import Queue, Value, Process

#GPIO SETUP
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIGGER_1 = 4
ECHO_1 = 17
TRIGGER_2 = 10
ECHO_2 = 9
TRIGGER_3 = 27
ECHO_3 = 22
TRIGGER_4 = 11
ECHO_4 = 5

IN1 = 19
IN2 = 26
PWM1 = 13

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(PWM1, GPIO.OUT)

p = GPIO.PWM(PWM1, 1000)
p.start(0)

GPIO.setup(TRIGGER_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)
GPIO.setup(TRIGGER_2, GPIO.OUT)
GPIO.setup(ECHO_2, GPIO.IN)
GPIO.setup(TRIGGER_3, GPIO.OUT)
GPIO.setup(ECHO_3, GPIO.IN)
GPIO.setup(TRIGGER_4, GPIO.OUT)
GPIO.setup(ECHO_4, GPIO.IN)

#Returns the distance that a particular sensor sees, takes in the Echo and Trigger pins of the sensor
def getDistance(TRIGGER, ECHO):
    # set Trigger to HIGH
    GPIO.output(TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 13503.94) / 2
 
    return distance

#Process 0, polls all the sensors and determines which motors should respond
def pollSensors(dist1, dist2, dist3, dist4, THRESHOLD):
    global run_flag
    while(run_flag):
        try:
            #First, get all of the distances
            dist1.value = getDistance(TRIGGER_1, ECHO_1)
            dist2.value = getDistance(TRIGGER_2, ECHO_2)
            dist3.value = getDistance(TRIGGER_3, ECHO_3)
            dist4.value = getDistance(TRIGGER_4, ECHO_4)
            
            #Four possible cases for Axis 1 (sensors 1 and 2)
#             if(dist1.value < THRESHOLD.value and dist2.value >= THRESHOLD.value):
#                 print('Axis1: CASE 1: M1 FORWARD', end ="")
#             elif(dist1.value >= THRESHOLD.value and dist2.value < THRESHOLD.value):
#                 print('Axis1: CASE 2: M1 BACKWARD', end = "")
#             elif(dist1.value < THRESHOLD.value and dist2.value < THRESHOLD.value):
#                 if(dist1.value < dist2.value):
#                     print('Axis1: CASE 3: M1 FORWARD', end="")
#                 elif(dist1.value >= dist2.value):
#                     print('Axis1: CASE 3: M1 BACKWARD', end="")
#             elif(dist1.value >= THRESHOLD.value and dist2.value >= THRESHOLD.value):
#                 print('Axis1: CASE 4: M1 OFF', end="")
#             
#             #Four possible cases for Axis 2 (sensors 3 and 4)
#             if(dist3.value < THRESHOLD.value and dist4.value >= THRESHOLD.value):
#                 print(' | Axis2: CASE 1: M2 FORWARD')
#             elif(dist3.value >= THRESHOLD.value and dist4.value < THRESHOLD.value):
#                 print(' | Axis2: CASE 2: M2 BACKWARD')
#             elif(dist3.value < THRESHOLD.value and dist4.value < THRESHOLD.value):
#                 if(dist3.value < dist4.value):
#                     print(' | Axis2: CASE 3: M2 FORWARD')
#                 elif(dist3.value >= dist4.value):
#                     print(' | Axis2: CASE 3: M2 BACKWARD')
#             elif(dist3.value >= THRESHOLD.value and dist4.value >= THRESHOLD.value):
#                 print(' | Axis2: CASE 4: M2 OFF')
        
        except KeyboardInterrupt:
            run_flag = 0
            break
            print("exit")
            
#Returns the PWM Duty Cycle based on the sensor 1 reading
def Sensor1PID(dist1, THRESHOLD, Kp, Ki, PWMSig1):
    global run_flag
    pterm = 0
    iterm = 0
    while(run_flag):
        try:
            start_time = time.time()
            distance = dist1.value
            error = THRESHOLD.value - distance
            pterm = Kp.value * error
            iterm += Ki.value * error * 0.03 #0.03 is the approximate loop time
            output = pterm
            
            if(iterm > 100): iterm = 100
            if(iterm < 0): iterm = 0
            if(output > 100): output = 100
            if(output < 0): output = 0
            if(error < 0): output = 0
            PWMSig1.value = abs(output) #Give the loop some slight delay (might get rid of this if we dont use the ID part of PID)
            #print('PWM: ', PWMSig1.value, ' Dist1: ', distance)
            time.sleep(0.001)
            if(time.time() - start_time < 0.02):
                time.sleep(0.02 - (time.time() - start_time))
            
        except KeyboardInterrupt:
            run_flag = 0
            break
            print("exit")

#Returns the PWM Duty Cycle based on the sensor 2 reading
def Sensor2PID(dist2, THRESHOLD, Kp, Ki, PWMSig2):
    global run_flag
    pterm = 0
    iterm = 0
    while(run_flag):
        try:
            start_time = time.time()
            distance = dist2.value
            error = THRESHOLD.value - distance
            pterm = Kp.value * error
            iterm += Ki.value * error * 0.03 #0.03 is the approximate loop time
            output = pterm
            
            if(iterm > 100): iterm = 100
            if(iterm < 0): iterm = 0
            if(output > 100): output = 100
            if(output < 0): output = 0
            if(error < 0): output = 0
            PWMSig2.value = abs(output)
            #print('PWM: ', PWMSig2.value, ' Dist2: ', distance)
            time.sleep(0.001) #Give the loop some slight delay (might get rid of this if we dont use the ID part of PID)
            if(time.time() - start_time < 0.02):
                time.sleep(0.02 - (time.time() - start_time))
            
        except KeyboardInterrupt:
            run_flag = 0
            break
            print("exit")
            
    
#Returns the PWM Duty Cycle based on the sensor 3 reading
def Sensor3PID(dist3, THRESHOLD, Kp, Ki, PWMSig3):
    global run_flag
    pterm = 0
    iterm = 0
    while(run_flag):
        try:
            start_time = time.time()
            distance = dist3.value
            error = THRESHOLD.value - distance
            pterm = Kp.value * error
            iterm += Ki.value * error * 0.03 #0.03 is the approximate loop time
            output = pterm
            
            if(iterm > 100): iterm = 100
            if(iterm < 0): iterm = 0
            if(output > 100): output = 100
            if(output < 0): output = 0
            if(error < 0): output = 0
            PWMSig3.value = abs(output)
            print('PWM: ', PWMSig3.value, ' Dist3: ', distance)
            time.sleep(0.001) #Give the loop some slight delay (might get rid of this if we dont use the ID part of PID)
            if(time.time() - start_time < 0.02):
                time.sleep(0.02 - (time.time() - start_time))
            
        except KeyboardInterrupt:
            run_flag = 0
            break
            print("exit")

#Returns the PWM Duty Cycle based on the sensor 4 reading
def Sensor4PID(dist4, THRESHOLD, Kp, Ki, PWMSig4):
    global run_flag
    pterm = 0
    iterm = 0
    while(run_flag):
        try:
            start_time = time.time()
            distance = dist4.value
            error = THRESHOLD.value - distance
            pterm = Kp.value * error
            iterm += Ki.value * error * 0.03 #0.03 is the approximate loop time
            output = pterm
            
            if(iterm > 100): iterm = 100
            if(iterm < 0): iterm = 0
            if(output > 100): output = 100
            if(output < 0): output = 0
            if(error < 0): output = 0
            PWMSig4.value = abs(output)
            #print('PWM: ', PWMSig4.value, ' Dist4: ', distance)
            time.sleep(0.001) #Give the loop some slight delay (might get rid of this if we dont use the ID part of PID)
            if(time.time() - start_time < 0.02):
                time.sleep(0.02 - (time.time() - start_time))
            
        except KeyboardInterrupt:
            run_flag = 0
            break
            print("exit")

            

if __name__ == '__main__':
    #Use a run flag to handle quitting processes using Ctrl-C
    run_flag = Value('i', 1)
    THRESHOLD = Value('d', 10) #The threshold distance that defines a "trip"
    #Use 4 shared values for the distances
    dist1 = Value('d', 0.0)
    dist2 = Value('d', 0.0)
    dist3 = Value('d', 0.0)
    dist4 = Value('d', 0.0)
    #Each PID uses the same constants
    Kp = Value('d', 25)
    Ki = Value('d', 2)
    #All the Duty Cycle Signals are shared data
    PWMSig1 = Value('d', 0)
    PWMSig2 = Value('d', 0)
    PWMSig3 = Value('d', 0)
    PWMSig4 = Value('d', 0)
    
    p0 = Process(target=pollSensors, args=(dist1,dist2,dist3,dist4,THRESHOLD,))
    p1 = Process(target=Sensor1PID, args=(dist1, THRESHOLD, Kp, Ki, PWMSig1,))
    p2 = Process(target=Sensor2PID, args=(dist2, THRESHOLD, Kp, Ki, PWMSig2,))
    p3 = Process(target=Sensor3PID, args=(dist3, THRESHOLD, Kp, Ki, PWMSig3,))
    p4 = Process(target=Sensor4PID, args=(dist4, THRESHOLD, Kp, Ki, PWMSig4,))
    
    
    p0.start()
    p1.start()
    p2.start()
    p3.start()
    p4.start()
    
    p0.join()
    p1.join()
    p2.join()
    p3.join()
    p4.join()

GPIO.cleanup()