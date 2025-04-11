import threading
import RPi.GPIO as GPIO
import time
import cv2
from alphabot_agent.alphabotlib.TRSensors import TRSensor
import numpy as np
from functools import reduce

class AlphaBot2(object):
    def __init__(self, ain1=12, ain2=13, ena=6, bin1=20, bin2=21, enb=26):
        self.AIN1 = ain1
        self.AIN2 = ain2
        self.BIN1 = bin1
        self.BIN2 = bin2
        self.ENA = ena
        self.ENB = enb
        self.PA = 50
        self.PB = 50
        self.PIC_WIDTH = 640
        self.PIC_HEIGHT = 480
        self.CTR = 7
        self.BUZ = 4

        self.DR = 16
        self.DL = 19

        self.turn_speed = 4.3e-3
        self.forward_speed = 0.5

        self.motor_startup_turn = 10e-4
        self.motor_startup_forward = 5e-4

        self.forwardEquation = lambda x: 2.648777 * x + 137.4677

        self.TR = TRSensor()

        # TODO: Remove this when in prod !
        self.TR.calibratedMin = [306, 323, 293, 321, 278]
        self.TR.calibratedMax = [960, 961, 937, 962, 959]

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.BUZ, GPIO.OUT)
        GPIO.setup(self.CTR, GPIO.OUT)
        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.DR, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.DL, GPIO.IN, GPIO.PUD_UP)
        self.PWMA = GPIO.PWM(self.ENA, 500)
        self.PWMB = GPIO.PWM(self.ENB, 500)
        self.PWMA.start(self.PA)
        self.PWMB.start(self.PB)
        self.stop()

    def takePic(self):
        # Open camera
        cap = cv2.VideoCapture(0)

        # Set resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.PIC_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.PIC_HEIGHT)

        # Get actual width/height returned
        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Actual resolution: {actual_width}x{actual_height}")

        time.sleep(0.5)

        ret, frame = cap.read()
        if ret:
            return frame.reshape((self.PIC_HEIGHT, self.PIC_WIDTH, 3))
        raise Exception("Failed to capture image")

    def beep_on(self):
        GPIO.output(self.BUZ, GPIO.HIGH)

    def beep_off(self):
        GPIO.output(self.BUZ, GPIO.LOW)

    def calibrateTRSensors(self):
        self.left(15)
        time.sleep(0.5)
        self.TR.calibrate()
        self.stop()
        print("Calibrated, values:")
        print("Min: ", self.TR.calibratedMin)
        print("Max: ", self.TR.calibratedMax)

    def calibrateForward(self, speed=30):
        lineTreshold = 100
        whiteTreshold = 900

        papers = [[30, 40, 70, 120], [100, 150]]

        measurements = []

        preciseSpeed = 7

        def runUntilLine(timeout=1500):
            armed = False
            counter = 0
            while True:
                if counter >= timeout:
                    self.stop()
                    raise Exception(
                        "Line not detected until timeout ! Calibration failed..."
                    )
                counter += 1
                res = self.TR.readCalibrated()
                if res[3] < lineTreshold and (
                    res[4] < lineTreshold or res[2] < lineTreshold
                ):
                    if armed:
                        break
                if res[3] > whiteTreshold and (
                    res[4] > whiteTreshold or res[2] > whiteTreshold
                ):
                    armed = True
            pass

        def measureTimeToNextLine():
            self.PA = speed
            self.PB = speed
            self.forward()
            start = time.time()
            runUntilLine()
            stop = time.time()
            self.stop()
            return stop - start

        def goToStartLine():
            self.PA = preciseSpeed
            self.PB = preciseSpeed
            self.forward()
            runUntilLine()
            self.stop()

        def goBackToLine():
            self.PA = preciseSpeed
            self.PB = preciseSpeed
            self.backward()
            runUntilLine()
            self.stop()

        def waitForJoystickCenter():
            flag = False
            while True:
                if GPIO.input(self.CTR) == 0:
                    self.beep_on()
                    flag = True
                if flag and GPIO.input(self.CTR) == 1:
                    self.beep_off()
                    break

        def flatten(li):
            return reduce(
                lambda x, y: [*x, y] if not isinstance(y, list) else x + flatten(y),
                li,
                [],
            )

        for p in papers:
            print("Waiting for joystick press to start the forward calibration")
            waitForJoystickCenter()
            time.sleep(0.5)
            goToStartLine()
            print("At start line. starting !")
            time.sleep(0.5)
            for d in p:
                timeTaken = measureTimeToNextLine()
                print("Measurement taken !")
                measurements.append(timeTaken * 1000)
                time.sleep(0.5)
                goBackToLine()
                time.sleep(0.5)
        flattened = flatten(papers)

        a, b = np.polyfit(flattened, measurements, 1)

        print("A is : ", a)
        print("B is : ", b)

        error = 0
        for x, y in zip(flattened, measurements):
            error += abs(a * x + b - y)
        error /= len(flattened)
        print("Error is : ", error)

        self.forwardEquation = lambda x: a * x + b

    def turn(self, angle=90, speed=8):
        self.PWMA.ChangeDutyCycle(speed)
        self.PWMB.ChangeDutyCycle(speed)

        duration = self.turn_speed * abs(angle)
        duration += self.motor_startup_turn

        if angle > 0:
            GPIO.output(self.AIN1, GPIO.LOW)
            GPIO.output(self.AIN2, GPIO.HIGH)
            GPIO.output(self.BIN1, GPIO.HIGH)
            GPIO.output(self.BIN2, GPIO.LOW)
        else:
            GPIO.output(self.AIN1, GPIO.HIGH)
            GPIO.output(self.AIN2, GPIO.LOW)
            GPIO.output(self.BIN1, GPIO.LOW)
            GPIO.output(self.BIN2, GPIO.HIGH)

        time.sleep(duration)

        self.stop()

        pass

    def safeForward(self, mm=1, speed=30):
        if self.forwardEquation:
            duration = self.forwardEquation(mm) / 1000
            print("Duration is : ", duration)
        else:
            duration = self.forward_speed * mm * 150
            duration += self.motor_startup_forward

        self.PWMA.ChangeDutyCycle(speed)
        self.PWMB.ChangeDutyCycle(speed)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH)


        def run_for_time(duration):
            start_time = time.time()
            while time.time() - start_time < duration:
                DR_status = GPIO.input(self.DR)
                DL_status = GPIO.input(self.DL)
                if (DL_status == 0) or (DR_status == 0):
                    break
            self.stop()

        thread = threading.Thread(target=run_for_time, args=(duration,))
        thread.start()

    def forward(self):
        self.PWMA.ChangeDutyCycle(self.PA)
        self.PWMB.ChangeDutyCycle(self.PB)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH)

    def stop(self):
        self.PWMA.ChangeDutyCycle(0)
        self.PWMB.ChangeDutyCycle(0)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.LOW)

    def backward(self):
        self.PWMA.ChangeDutyCycle(self.PA)
        self.PWMB.ChangeDutyCycle(self.PB)
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.HIGH)
        GPIO.output(self.BIN2, GPIO.LOW)

    def left(self, speed=30):
        self.PWMA.ChangeDutyCycle(speed)
        self.PWMB.ChangeDutyCycle(speed)
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH)

    def right(self, speed=30):
        self.PWMA.ChangeDutyCycle(speed)
        self.PWMB.ChangeDutyCycle(speed)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH)
        GPIO.output(self.BIN1, GPIO.HIGH)
        GPIO.output(self.BIN2, GPIO.LOW)

    def setPWMA(self, value):
        self.PA = value
        self.PWMA.ChangeDutyCycle(self.PA)

    def setPWMB(self, value):
        self.PB = value
        self.PWMB.ChangeDutyCycle(self.PB)

    def setMotor(self, left, right):
        if (right >= 0) and (right <= 100):
            GPIO.output(self.AIN1, GPIO.HIGH)
            GPIO.output(self.AIN2, GPIO.LOW)
            self.PWMA.ChangeDutyCycle(right)
        elif (right < 0) and (right >= -100):
            GPIO.output(self.AIN1, GPIO.LOW)
            GPIO.output(self.AIN2, GPIO.HIGH)
            self.PWMA.ChangeDutyCycle(0 - right)
        if (left >= 0) and (left <= 100):
            GPIO.output(self.BIN1, GPIO.HIGH)
            GPIO.output(self.BIN2, GPIO.LOW)
            self.PWMB.ChangeDutyCycle(left)
        elif (left < 0) and (left >= -100):
            GPIO.output(self.BIN1, GPIO.LOW)
            GPIO.output(self.BIN2, GPIO.HIGH)
            self.PWMB.ChangeDutyCycle(0 - left)

if __name__ == "__main__":
    Ab = AlphaBot2()
    Ab.forward()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
