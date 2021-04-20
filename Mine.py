import RPi.GPIO as GPIO
import time
from adafruit_htu21d import HTU21D
import paho.mqtt.client as mqtt
import board
import busio


class Mine():
    """
    pins
    fun - 17
    servo - 15
    button - 26
    dist
    trig - 23
    echo - 24

    temp - i2c(2,3)
    """

    def __init__(self):
        self.temp = 0
        self.humi = 0
        self.dist = 0
        self.fun = False
        self.alarm = False
        self.noise = 0
        self.button = False

        self.led_ok = True
        self.led_alarm = False

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.button_pin = 26
        self.servo_pin = 18
        self.trig_pin = 23
        self.echo_pin = 24
        self.fun_pin = 17

        self.door_is_open = False
        self.fun_is_on = False

        self.fun_auto = True
        self.door_auto = True
        # self.fun_is_on = False

        self.green_led_pin = 22
        self.red_led_pin = 27

        # for button
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # for hc distance
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

        # for sensor, temp, humi
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = HTU21D(i2c)

        # for servo
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.servo_pin, 100)
        self.servo.start(5)

        # for fun
        GPIO.setup(self.fun_pin, GPIO.OUT)
        GPIO.output(self.fun_pin, False)

        # for leds
        GPIO.setup(self.green_led_pin, GPIO.OUT)
        GPIO.setup(self.red_led_pin, GPIO.OUT)
        GPIO.output(self.green_led_pin, True)
        GPIO.output(self.red_led_pin, False)

        self.connect = False

        self.try_connect()

    def try_connect(self):
        try:
            print("Start")
            self.client = mqtt.Client(client_id="mqtt-iprofi_766199718-szns88")
            # self.client = mqtt.Client(client_id="mqtt-iprofi_766199718-nvmzm7")
            # client.username_pw_set(username="Nick", password="pass")
            self.client.on_connect = self.on_connect
            self.client.on_message = self.on_message

            self.client.connect("sandbox.rightech.io", 1883, 60)
            self.connect = True

        except ConnectionError:
            print('Connection error')



    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))

    def on_message(self, client, userdata, msg):
        # print(msg.topic+" "+str(msg.payload))
        if msg.topic == "control/alarm":
            if msg.payload == b'1':
                self.alarm_on()
            else:
                self.alarm_off()
        elif msg.topic == "control/ventilation":
            if msg.payload == b'1':
                if self.fun_is_on is False:
                    self.fun_auto = False
                    self.fun_on()
                else:
                    self.fun_auto = True
            else:
                if self.fun_is_on:
                    self.fun_auto = False
                    self.fun_off()
                else:
                    self.fun_auto = True

        elif msg.topic == "control/door":
            if msg.payload == b'1':
                if self.door_is_open is False:
                    self.door_auto = False
                    self.open_door()
                else:
                    self.door_auto = True
            else:
                if self.door_is_open:
                    self.door_auto = True
                    self.close_door()
                

    def open_door(self):
        if self.door_is_open:
            return
        self.door_is_open = True
        angle = 120
        duty = float(angle) / 10.0 + 2.5
        self.servo.ChangeDutyCycle(duty)
        if self.connect:
            self.client.publish("base/door", "true")

    def close_door(self):
        if self.door_is_open is False:
            return
        self.door_is_open = False
        angle = 20
        duty = float(angle) / 10.0 + 2.5
        self.servo.ChangeDutyCycle(duty)
        if self.connect:
            self.client.publish("base/door", "false")

    def fun_on(self):
        if self.fun_is_on:
            return
        self.fun_is_on = True
        GPIO.output(self.fun_pin, True)
        if self.connect:
            self.client.publish("base/ventilation", "true")

    def fun_off(self):
        if self.fun_is_on is False:
            return
        self.fun_is_on = False
        GPIO.output(self.fun_pin, False)
        if self.connect:
            self.client.publish("base/ventilation", "false")

    def alarm_on(self):
        if self.alarm:
            return
        GPIO.output(self.green_led_pin, False)
        GPIO.output(self.red_led_pin, True)
        self.open_door()
        self.fun_on()
        self.alarm = True
        if self.connect:
            self.client.publish("base/buzzer", "true")

    def alarm_off(self):
        if self.alarm is False:
            return
        GPIO.output(self.green_led_pin, True)
        GPIO.output(self.red_led_pin, False)
        self.close_door()
        self.fun_off()
        self.alarm = False
        if self.connect:
            self.client.publish("base/buzzer", "false")

    def get_temp_humi(self):
        temperature = self.sensor.temperature
        humidity = self.sensor.relative_humidity
        return round(temperature, 1), round(humidity, 1)

    def get_gist(self):

        GPIO.output(self.trig_pin, True)

        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)

        StartTime = time.time()
        StopTime = time.time()

        # save StartTime
        while GPIO.input(self.echo_pin) == 0:
            StartTime = time.time()

        # save time of arrival
        while GPIO.input(self.echo_pin) == 1:
            StopTime = time.time()

        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2

        return round(distance, 1)

    def send_temp_humi(self):
        if self.connect:
            self.client.publish("base/temp", self.temp)
            self.client.publish("base/humi", self.humi)

    def get_button(self):
        if GPIO.input(self.button_pin) == 1:
            if self.button is False:
                self.button = True
                self.client.publish("base/alarm", "true")
            return True
        if self.button:
            self.client.publish("base/alarm", "false")
            self.button = False
        return False

    def update(self):
        self.temp, self.humi = self.get_temp_humi()
        self.dist = self.get_gist()

        # self.noise = self.get_noise()
        self.get_button()
        button = "OFF"
        if self.button:
            button = "ON"
        print(f'| Temp {self.temp} | Humi {self.humi} | Dist {self.dist} | Button {button} |')

        # self.publish()
        # if self.noise > 10:
        #    self.alarm_on()
        if self.alarm is True:
            return
        elif self.button:
            self.alarm_on()
            return

        if self.dist < 10 and self.door_auto:
            self.open_door()
        elif self.dist > 20 and self.door_auto:
            self.close_door()

        if self.temp > 30 and self.fun_auto:
            self.fun_on()
        elif self.temp < 25 and self.fun_auto:
            self.fun_off()

        time.sleep(0.1)

    def delete(self):
        self.fun_off()
        GPIO.output(self.green_led_pin, False)
        GPIO.output(self.red_led_pin, False)
        self.servo.stop()
        GPIO.cleanup()


