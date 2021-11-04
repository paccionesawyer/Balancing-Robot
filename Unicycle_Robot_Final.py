import paho.mqtt.client as mqtt
import time
import math
import json
import RPi.GPIO as GPIO
import argparse
import csv
from itertools import zip_longest


class UnicycleRobot():
    # MQTT_BROKER_ADDRESS = "10.245.91.207" # IP of my Raspberry PI
    # MQTT_BROKER_ADDRESS = "broker.emqx.io" # For external broker
    def __init__(self,clientname="RPi-Saw",broker="10.245.91.207",topicList=["test"]):        
        self.broker = broker
        self.topicList = topicList
        self.clientname = clientname
        self.client = mqtt.Client(self.clientname)
        self.client.on_connect = self.onConnect
        self.client.on_message = self.onMessage
        self.received = ''
        self.startTime = time.time()
        self.currTime = self.startTime
        self.prevTime = self.startTime
        self.pitchError = 0
        self.rollError = 0

        self.curPitch = 0
        self.curRoll = 0

        GPIO.setmode(GPIO.BOARD)
        self.m1pin1 = 32
        self.m1pin2 = 33
        self.m2pin1 = 29
        self.m2pin2 = 31

        self.rollMotor_F, self.rollMotor_R = self.initMotors(self.m1pin1, self.m1pin2)
        self.pitchMotor_F, self.pitchMotor_R = self.initMotors(self.m2pin1, self.m2pin2)

        self.integral = 0

        self.desiredPitch = 4.730776286262152
        self.desiredRoll  = -10.13540139014226

        self.save = False

    def motorTest(self):
        print("Starting Motor 1")
        self.rollMotor_F.start(30)
        time.sleep(0.5)
        self.rollMotor_F.stop()
        time.sleep(2)
        print("Reversing Motor 1")
        self.rollMotor_R.start(30)
        time.sleep(0.5)
        self.rollMotor_R.stop()
        time.sleep(2)
        print("Starting Motor 2")
        self.pitchMotor_F.start(30)
        time.sleep(0.5)
        self.pitchMotor_F.stop()
        time.sleep(2)
        print("Reversing Motor 2")
        self.pitchMotor_R.start(30)
        time.sleep(0.5)
        self.pitchMotor_R.stop()
        time.sleep(2)

    def runMotor(self, motor, duty_cycle):
        if duty_cycle > 60:
            duty_cycle = 60
        elif duty_cycle < -60:
            duty_cycle = -60

        if (motor == 1):
            if (duty_cycle <= 0):
                self.rollMotor_F.stop()
                self.rollMotor_R.start(abs(duty_cycle))
            else :
                self.rollMotor_R.stop()
                self.rollMotor_F.start(duty_cycle)
        elif (motor == 2):
            if (duty_cycle <= 0):
                self.pitchMotor_F.stop()
                self.pitchMotor_R.start(abs(duty_cycle))
            else :
                self.pitchMotor_R.stop()
                self.pitchMotor_F.start(duty_cycle)
        else:
            print("Wrong Motor Number")

    def subscribeTopics(self):
        for topic in self.topicList:
            self.client.subscribe(topic)

    def onConnect(self,client,userdata,flags,rc):
        if rc == 0:
            print("T.H.A.N.O.S. Connection Established")
        else:
            print("bad connection Returned code=",rc)
        self.subscribeTopics()

    def onMessage(self,client,userdata,msg):

        if msg.topic == 'esp32/acc':

            msgDict = self.mqttMessageToDict(msg.payload)

            accelx = msgDict["accelx"]
            accely = msgDict["accely"]
            accelz = msgDict["accelz"]

            self.curPitch = self.getPitch(accely, accelz)
            self.curRoll = self.getRoll(accelx, accely, accelz)

            print("Pitch:", self.curPitch, "Roll:", self.curRoll)

            pitchPID, self.pitchError = self.getPID(0, self.curPitch, 5, 0, 0, self.pitchError)
            rollPID, self.rollError = self.getPID(0, self.curRoll, 5, 0, 0, self.rollError)

            # print("PitchPID:", pitchPID, "RollPID:", rollPID)
            
            self.runMotor(1, rollPID)
            self.runMotor(2, pitchPID)

            if self.save: 
                self.accXList.append(accelx)
                self.accYList.append(accely)
                self.accZList.append(accelz)

                self.pitchList.append(self.curPitch)
                self.rollList.append(self.curRoll)

                # self.accWriter.writerows(msgDict)

                # angleRows = {'Roll': self.curRoll, 'Pitch': self.curPitch}

                # self.angleWriter.writerows(angleRows)

        elif msg.topic == 'esp32/gyro':
            msgDict = self.mqttMessageToDict(msg.payload)
            
            if self.save: 
                self.gyroXList.append(msgDict["gyrox"])
                self.gyroYList.append(msgDict["gyroy"])
                self.gyroZList.append(msgDict["gyroz"])
                # self.gyroWriter.writerows(msgDict)
    
        elif msg.topic == 'esp32/lidar':
            msgDict = self.mqttMessageToDict(msg.payload)

            if self.save:
                self.distList.append(msgDict['milli'])
                # self.gyroWriter.writerows(msgDict)

        elif msg.topic == 'esp32/time':
            msgDict = self.mqttMessageToDict(msg.payload)

            if self.save:
                self.timeList.append(msgDict['milliseconds'])
                
        else:
            print("Confused")
            print(msg.topic+" "+str(msg.payload))

    def begin(self):
        print('Setting up connection')
        self.client.connect(self.broker)
        self.client.loop_start()

    def end(self):
        print('Ending Connection')
        self.client.loop_stop()
        self.client.disconnect()
        self.rollMotor_F.stop()
        self.rollMotor_R.stop()
        self.pitchMotor_F.stop()
        self.pitchMotor_R.stop()

    def mqttMessageToDict(self, msg):
        msgStr  = msg.decode("utf-8") 
        msgDict = json.loads(msgStr)
        # print(msgDict)
        # print(type(msgDict))
        return msgDict

    def getPID(self, desired, angle, kp, ki, kd, error_prev):
        self.currTime = time.time()
        delta_time = self.currTime - self.prevTime

        error_cur = angle - desired

        delta_error = error_cur - error_prev

        self.integral += delta_error * delta_time
        derivative = delta_error / delta_time
        
        pOut = kp * error_cur
        dOut = kd * derivative
        iOut = ki * self.integral
        
        PID = pOut + dOut + iOut
        
        self.prevTime = time.time()

        return PID, error_cur

    def initMotors(self, PinOne, PinTwo, freqOne=50, freqTwo=50):
        #initializing pins as outputs
        GPIO.setup(PinOne, GPIO.OUT)
        GPIO.setup(PinTwo, GPIO.OUT)
        #initializing pins as PWM pins with a specified frequency
        pwmOne = GPIO.PWM(PinOne, freqOne)
        pwmTwo = GPIO.PWM(PinTwo, freqTwo)

        return pwmOne, pwmTwo

    def getPitch(self, acc_y, acc_z):
        return math.atan(acc_y/acc_z) * (180/math.pi)

    def getRoll(self, acc_x, acc_y, acc_z):
        return math.atan(-acc_x / math.sqrt(acc_y**2 + acc_z**2)) * (180/math.pi)

    def setCSV(self, filePath):
        self.export_data = zip_longest(*data, fillvalue = '')
        self.save = True

        self.csvPath = filePath

        self.accXList = []
        self.accYList = []
        self.accZList = []

        self.gyroXList = []
        self.gyroYList = []
        self.gyroZList = []

        self.pitchList = []
        self.rollList  = []

        self.distList  = []
        self.timeList  = []

        # self.accFile   = open('./data/accData.csv', 'w')
        # self.gyroFile  = open('./data/gyroData.csv', 'w')
        # self.angleFile = open('./data/angleData.csv', 'w')
        # self.lidarFile = open('./data/lidarData.csv', 'w')

        # accHeaders = ['accelX', 'accelY', 'accelZ']
        # gyroHeaders = ['gyroX', 'gyroY', 'gyroZ']
        # angleHeaders = ['Roll', 'Pitch']
        # lidarHeaders = ['lidar']

        # self.accWriter   = csv.DictWriter(self.accFile, fieldnames=accHeaders)
        # self.accWriter.writeheader()

        # self.gyroWriter  = csv.DictWriter(self.gyroFile, fieldnames=gyroHeaders)
        # self.gyroWriter.writeheader()

        # self.angleWriter = csv.DictWriter(self.angleFile, fieldnames=angleHeaders)
        # self.angleWriter.writeheader()

        # self.lidarWriter = csv.DictWriter(self.lidarFile, fieldnames=lidarHeaders)
        # self.lidarWriter.writeheader()

    def saveCSV(self):
        data = [self.accXList, self.accYList, self.accYList, self.accZList, self.gyroXList, self.gyroYList, self.gyroZList, self.pitchList, self.rollList, self.distList, self.timeList]
        export_data = zip_longest(*data, fillvalue = '')

        with open(self.csvPath, 'w', encoding='ISO-8859-1', newline='') as file:
            write = csv.writer(file)
            write.writerow("AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "Pitch (deg)", "Roll (deg)", "Dist (mm)", "Time")
            write.writerows(export_data)


if __name__ == "__main__":
    espTopics = ["esp32/accel", "esp32/gyro", "esp32/lidar", "esp32/time"]

    CSVfile = ''

    parser = argparse.ArgumentParser(
        description="A Self balancing robot that utilizes two-flywheel's angular momentum to keep itself upright"
    )
    parser.add_argument(
        "-sT",
        "--subTopics",
        nargs="+",
        help="Sets the topics the Pi is subscribed to, default={}".format(espTopics),
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Test the motors",
    )
    parser.add_argument(
        "-s"
        "--save",
        type=str,
        help="Saves the received data into a CSV file, default={}".format(),
    )

    if args.subTopics:
        espTopics = args.subtopics
    
    thanos = UnicycleRobot(topicList = espTopics)

    if args.test:
        thanos.motorTest()
    else:
        if args.save:
            CSVfile = args.save
            thanos.setCSV(CSVfile)

    thanos.begin()
    try:
        while True:
            pass
    except KeyboardInterrupt:
        thanos.end()
        GPIO.cleanup() #empties occupied channels

    