import time as time
import RPi.GPIO as GPIO
from multiprocessing import Queue, Value, Process
import signal as signal


#GPIO SETUP
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

okSize = 200

TRIGGER_1 = 19
ECHO_1 = 26
TRIGGER_2 = 21
ECHO_2 = 20
TRIGGER_3 = 14
ECHO_3 = 15
TRIGGER_4 = 16
ECHO_4 = 12

M1_IN1 = 3
M1_IN2 = 4
M1_PWM = 2

M2_IN1 = 27
M2_IN2 = 22
M2_PWM = 17

M3_IN1 = 9
M3_IN2 = 11
M3_PWM = 10

M4_IN1 = 6
M4_IN2 = 13
M4_PWM = 5

GPIO.setup(M1_IN1, GPIO.OUT)
GPIO.setup(M1_IN2, GPIO.OUT)
GPIO.setup(M1_PWM, GPIO.OUT)

GPIO.setup(M2_IN1, GPIO.OUT)
GPIO.setup(M2_IN2, GPIO.OUT)
GPIO.setup(M2_PWM, GPIO.OUT)

GPIO.setup(M3_IN1, GPIO.OUT)
GPIO.setup(M3_IN2, GPIO.OUT)
GPIO.setup(M3_PWM, GPIO.OUT)

GPIO.setup(M4_IN1, GPIO.OUT)
GPIO.setup(M4_IN2, GPIO.OUT)
GPIO.setup(M4_PWM, GPIO.OUT)

pm1 = GPIO.PWM(M1_PWM, 1000)
pm1.start(0)
pm2 = GPIO.PWM(M2_PWM, 1000)
pm2.start(0)
pm3 = GPIO.PWM(M3_PWM, 1000)
pm3.start(0)
pm4 = GPIO.PWM(M4_PWM, 1000)
pm4.start(0)

GPIO.setup(TRIGGER_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)
GPIO.setup(TRIGGER_2, GPIO.OUT)
GPIO.setup(ECHO_2, GPIO.IN)
GPIO.setup(TRIGGER_3, GPIO.OUT)
GPIO.setup(ECHO_3, GPIO.IN)
GPIO.setup(TRIGGER_4, GPIO.OUT)
GPIO.setup(ECHO_4, GPIO.IN)

#Sets the motors up so the robot moves left
def setLeft():
    GPIO.output(M1_IN1, GPIO.HIGH)
    GPIO.output(M1_IN2, GPIO.LOW)
    
    GPIO.output(M2_IN1, GPIO.LOW)
    GPIO.output(M2_IN2, GPIO.HIGH)
    
    GPIO.output(M3_IN1, GPIO.HIGH)
    GPIO.output(M3_IN2, GPIO.LOW)
    
    GPIO.output(M4_IN1, GPIO.HIGH)
    GPIO.output(M4_IN2, GPIO.LOW)
    
def turnLeft():
    GPIO.output(M1_IN1, GPIO.HIGH)
    GPIO.output(M1_IN2, GPIO.LOW)
    
    GPIO.output(M2_IN1, GPIO.HIGH)
    GPIO.output(M2_IN2, GPIO.LOW)
    
    GPIO.output(M3_IN1, GPIO.HIGH)
    GPIO.output(M3_IN2, GPIO.LOW)
    
    GPIO.output(M4_IN1, GPIO.LOW)
    GPIO.output(M4_IN2, GPIO.HIGH)


def turnRight():
    GPIO.output(M1_IN1, GPIO.LOW)
    GPIO.output(M1_IN2, GPIO.HIGH)
    
    GPIO.output(M2_IN1, GPIO.LOW)
    GPIO.output(M2_IN2, GPIO.HIGH)
    
    GPIO.output(M3_IN1, GPIO.LOW)
    GPIO.output(M3_IN2, GPIO.HIGH)
    
    GPIO.output(M4_IN1, GPIO.HIGH)
    GPIO.output(M4_IN2, GPIO.LOW)


#Sets the motors up so the robot moves right
def setRight():
    GPIO.output(M1_IN1, GPIO.LOW)
    GPIO.output(M1_IN2, GPIO.HIGH)
    
    GPIO.output(M2_IN1, GPIO.HIGH)
    GPIO.output(M2_IN2, GPIO.LOW)
    
    GPIO.output(M3_IN1, GPIO.LOW)
    GPIO.output(M3_IN2, GPIO.HIGH)
    
    GPIO.output(M4_IN1, GPIO.LOW)
    GPIO.output(M4_IN2, GPIO.HIGH)

#Sets the motors up so the robot moves backward
def setBackward():
    GPIO.output(M1_IN1, GPIO.HIGH)
    GPIO.output(M1_IN2, GPIO.LOW)
    
    GPIO.output(M2_IN1, GPIO.HIGH)
    GPIO.output(M2_IN2, GPIO.LOW)
    
    GPIO.output(M3_IN1, GPIO.LOW)
    GPIO.output(M3_IN2, GPIO.HIGH)
    
    GPIO.output(M4_IN1, GPIO.HIGH)
    GPIO.output(M4_IN2, GPIO.LOW)

#Sets the motors up so the robot moves forward
def setForward():
    GPIO.output(M1_IN1, GPIO.LOW)
    GPIO.output(M1_IN2, GPIO.HIGH)
    
    GPIO.output(M2_IN1, GPIO.LOW)
    GPIO.output(M2_IN2, GPIO.HIGH)
    
    GPIO.output(M3_IN1, GPIO.HIGH)
    GPIO.output(M3_IN2, GPIO.LOW)
    
    GPIO.output(M4_IN1, GPIO.LOW)
    GPIO.output(M4_IN2, GPIO.HIGH)

#set the Duty Cycle of each motor at once
def setDutyCycle(Duty):
    pm1.ChangeDutyCycle(Duty)
    pm2.ChangeDutyCycle(Duty)
    pm3.ChangeDutyCycle(Duty)
    pm4.ChangeDutyCycle(Duty)

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
            print('PWM: ', PWMSig1.value, ' Dist1: ', distance)
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
            #print('PWM: ', PWMSig3.value, ' Dist3: ', distance)
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

#Interrupt handler for Ctrl-C exits which turns all the motors off
def intHandler():
    GPIO.output(M1_IN1, GPIO.LOW)
    GPIO.output(M1_IN2, GPIO.LOW)
    
    GPIO.output(M2_IN1, GPIO.LOW)
    GPIO.output(M2_IN2, GPIO.LOW)
    
    GPIO.output(M3_IN1, GPIO.LOW)
    GPIO.output(M3_IN2, GPIO.LOW)
    
    GPIO.output(M4_IN1, GPIO.LOW)
    GPIO.output(M4_IN2, GPIO.LOW)
    setDutyCycle(0)
            

if __name__ == '__main__':
    
    signal.signal(signal.SIGINT, intHandler)
    #Open CV setup
    
    
    #Use a run flag to handle quitting processes using Ctrl-C
    run_flag = Value('i', 1)
    THRESHOLD = Value('d', 15) #The threshold distance that defines a "trip"
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
    
    p1 = Process(target=Sensor1PID, args=(dist1, THRESHOLD, Kp, Ki, PWMSig1,))
    p2 = Process(target=Sensor2PID, args=(dist2, THRESHOLD, Kp, Ki, PWMSig2,))
    p3 = Process(target=Sensor3PID, args=(dist3, THRESHOLD, Kp, Ki, PWMSig3,))
    p4 = Process(target=Sensor4PID, args=(dist4, THRESHOLD, Kp, Ki, PWMSig4,))
    
    p1.start()
    p2.start()
    p3.start()
    p4.start()
    
    while(True):
        try:
            #First, get all of the distances
            dist1.value = getDistance(TRIGGER_1, ECHO_1)
            dist2.value = getDistance(TRIGGER_2, ECHO_2)
            dist3.value = getDistance(TRIGGER_3, ECHO_3)
            dist4.value = getDistance(TRIGGER_4, ECHO_4)
            
            S1_Trip = dist1.value < THRESHOLD.value
            S2_Trip = dist2.value < THRESHOLD.value
            S3_Trip = dist3.value < THRESHOLD.value
            S4_Trip = dist4.value < THRESHOLD.value
            
            #16 Possible Cases for 4 Sensors
            if(S1_Trip and not S2_Trip and not S3_Trip and not S4_Trip):
                setBackward()
                setDutyCycle(PWMSig1.value)
            elif(not S1_Trip and S2_Trip and not S3_Trip and not S4_Trip):
                setForward()
                setDutyCycle(PWMSig2.value)
            elif(not S1_Trip and not S2_Trip and S3_Trip and not S4_Trip):
                setRight()
                setDutyCycle(PWMSig3.value)
            elif(not S1_Trip and not S2_Trip and not S3_Trip and S4_Trip):
                setLeft()
                setDutyCycle(PWMSig4.value)
            elif(S1_Trip and S2_Trip and not S3_Trip and not S4_Trip):
                if(dist1.value < dist2.value):
                    setBackward()
                    setDutyCycle(PWMSig1.value)
                elif(dist2.value <= dist1.value):
                    setForward()
                    setDutyCycle(PWMSig2.value)
            elif(S1_Trip and not S2_Trip and S3_Trip and not S4_Trip):
                if(dist1.value < dist3.value):
                    setBackward()
                    setDutyCycle(PWMSig1.value)
                elif(dist3.value <= dist1.value):
                    setRight()
                    setDutyCycle(PWMSig3.value)
            elif(S1_Trip and not S2_Trip and not S3_Trip and S4_Trip):
                if(dist1.value < dist4.value):
                    setBackward()
                    setDutyCycle(PWMSig1.value)
                elif(dist4.value <= dist1.value):
                    setLeft()
                    setDutyCycle(PWMSig4.value)
            elif(not S1_Trip and S2_Trip and S3_Trip and not S4_Trip):
                if(dist2.value < dist3.value):
                    setForward()
                    setDutyCycle(PWMSig2.value)
                elif(dist3.value <= dist2.value):
                    setRight()
                    setDutyCycle(PWMSig3.value)
            elif(not S1_Trip and S2_Trip and not S3_Trip and S4_Trip):
                if(dist2.value < dist4.value):
                    setForward()
                    setDutyCycle(PWMSig2.value)
                elif(dist4.value <= dist2.value):
                    setLeft()
                    setDutyCycle(PWMSig4.value)
            elif(not S1_Trip and not S2_Trip and S3_Trip and S4_Trip):
                if(dist3.value < dist4.value):
                    setRight()
                    setDutyCycle(PWMSig3.value)
                elif(dist4.value <= dist3.value):
                    setLeft()
                    setDutyCycle(PWMSig4.value)
            elif(S1_Trip and S2_Trip and S3_Trip and not S4_Trip):
                l = (dist1.value, dist2.value, dist3.value)
                X = l.index(min(l))
                if(X == 0):
                    setBackward()
                    setDutyCycle(PWMSig1.value)
                elif(X == 1):
                    setForward()
                    setDutyCycle(PWMSig2.value)
                elif(X == 2):
                    setRight()
                    setDutyCycle(PWMSig3.value)
            elif(S1_Trip and S2_Trip and not S3_Trip and S4_Trip):
                l = (dist1.value, dist2.value, dist4.value)
                X = l.index(min(l))
                if(X == 0):
                    setBackward()
                    setDutyCycle(PWMSig1.value)
                elif(X == 1):
                    setForward()
                    setDutyCycle(PWMSig2.value)
                elif(X == 2):
                    setLeft()
                    setDutyCycle(PWMSig4.value)
            elif(S1_Trip and not S2_Trip and S3_Trip and S4_Trip):
                l = (dist1.value, dist3.value, dist4.value)
                X = l.index(min(l))
                if(X == 0):
                    setBackward()
                    setDutyCycle(PWMSig1.value)
                elif(X == 1):
                    setRight()
                    setDutyCycle(PWMSig3.value)
                elif(X == 2):
                    setLeft()
                    setDutyCycle(PWMSig4.value)
            elif(not S1_Trip and S2_Trip and S3_Trip and S4_Trip):
                l = (dist2.value, dist3.value, dist4.value)
                X = l.index(min(l))
                if(X == 0):
                    setForward()
                    setDutyCycle(PWMSig2.value)
                elif(X == 1):
                    setRight()
                    setDutyCycle(PWMSig3.value)
                elif(X == 2):
                    setLeft()
                    setDutyCycle(PWMSig4.value)
            elif(S1_Trip and S2_Trip and S3_Trip and S4_Trip):
                l = (dist1.value, dist2.value, dist3.value, dist4.value)
                X = l.index(min(l))
                if(X==0):
                    setBackward()
                    setDutyCycle(PWMSig1.value)
                elif(X == 1):
                    setForward()
                    setDutyCycle(PWMSig2.value)
                elif(X == 2):
                    setRight()
                    setDutyCycle(PWMSig3.value)
                elif(X == 3):
                    setLeft()
                    setDutyCycle(PWMSig4.value)
            elif(not S1_Trip and not S2_Trip and not S3_Trip and not S4_Trip):
                setDutyCycle(0)
   
        except KeyboardInterrupt:
            run_flag = 0
            setDutyCycle(0)
            break
            print("exit")
    
    #p0.join()
    #p1.join()
    #p2.join()
    #p3.join()
    #p4.join()

GPIO.cleanup()