import threading
import RPi.GPIO as GPIO
import time
import cv2
from TRSensors import TRSensor

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

        self.DR = 16
        self.DL = 19

        self.JOY = 7
        self.BUZ = 4

        self.turn_speed = 4.3e-3
        self.forward_speed = 0.5

        self.motor_startup_turn = 10e-4
        self.motor_startup_forward = 5e-4

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.BUZ,GPIO.OUT)
        GPIO.setup(self.JOY,GPIO.IN,GPIO.PUD_UP)
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

    def safeForward(self, tiles=1, speed=20):
        duration = self.forward_speed * tiles
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
                if (DL_status == 0) and (DR_status == 0):
                    self.stop()
                    print("we are facing the wall...")
                elif DL_status == 0:
                    GPIO.output(self.AIN1, GPIO.HIGH)
                    GPIO.output(self.AIN2, GPIO.LOW)
                    GPIO.output(self.BIN1, GPIO.LOW)
                    GPIO.output(self.BIN2, GPIO.HIGH)
                elif DR_status == 0:
                    GPIO.output(self.AIN1, GPIO.LOW)
                    GPIO.output(self.AIN2, GPIO.HIGH)
                    GPIO.output(self.BIN1, GPIO.HIGH)
                    GPIO.output(self.BIN2, GPIO.LOW)
                else:
                    GPIO.output(self.AIN1, GPIO.LOW)
                    GPIO.output(self.AIN2, GPIO.HIGH)
                    GPIO.output(self.BIN1, GPIO.LOW)
                    GPIO.output(self.BIN2, GPIO.HIGH)
                
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

    def left(self, speed = 30):
        self.PWMA.ChangeDutyCycle(speed)
        self.PWMB.ChangeDutyCycle(speed)
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH)

    def right(self, speed = 30):
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
    
    def beep_on(self):
        GPIO.output(self.BUZ,GPIO.HIGH)
    def beep_off(self):
        GPIO.output(self.BUZ,GPIO.LOW)

    def get_rotated_idiot(self):
        t = 3
        try:
            while GPIO.input(self.JOY) == 1:
                pass

            self.beep_on()

            while GPIO.input(self.JOY) == 0:
                pass
            
            self.beep_off()
            time.sleep(0.5)

            while True:
                self.left()
                time.sleep(t)
                self.stop()
                while GPIO.input(self.JOY) == 1:
                    pass

                self.beep_on()

                while GPIO.input(self.JOY) == 0:
                    pass
                
                self.beep_off()
                print(t)
                t += 0.1
                time.sleep(0.5)

        except KeyboardInterrupt:
            pass   

    def calibrate_rotate(self):
        TR = TRSensor()
        self.left()
        time.sleep(1)
        TR.calibrate()
        self.stop()

        print("Max : ", TR.calibratedMax)
        print("Min : ", TR.calibratedMin)

        time.sleep(2)
        self.right(15)

        sensor = TR.readCalibrated()
        print(sensor[2])
        while sensor[2] > 900:
            sensor = TR.readCalibrated()
        print("we are set")

        self.stop()
        
        time.sleep(2)

        t = time.time()
        #self.right()
        sensor = TR.readCalibrated()
        print(sensor[2])
        while sensor[2] < 100: # From bot wiki, black -> ~100-300, white -> ~800-900
            print(sensor[2])
            sensor = TR.readCalibrated()
        print("out of black")
        print(sensor[2])
        time.sleep(0.05)
        while sensor[2] > 900:
            sensor = TR.readCalibrated()
        print("back on black")
        t1 = time.time() - t
        #self.stop()
        time.sleep(2)
        
        #self.left(15)
        while sensor[2] > 900:
            sensor = TR.readCalibrated()
        #self.stop()
        time.sleep(2)

        t = time.time()
        #self.right()
        while sensor[2] < 100:
            sensor = TR.readCalibrated()
        print("out of black2")
        while sensor[2] > 900:
            sensor = TR.readCalibrated()
        print("back on black2")
        while sensor[2] < 100:
            sensor = TR.readCalibrated()
        print("out of black3")
        while sensor[2] > 900:
            sensor = TR.readCalibrated()
        print("back on black3")
        t2 = time.time() - t

        print(f"t1: {t1}, t2: {t2}")



if __name__ == "__main__":
    Ab = AlphaBot2()
    Ab.calibrate_rotate()
    # Ab.forward()
    time.sleep(0.425)
    Ab.stop()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
