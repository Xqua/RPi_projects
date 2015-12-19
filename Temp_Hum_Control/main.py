import RPi.GPIO as GPIO
import Adafruit_DHT
import time


class Controler:

    def __init__(self):
        self.pin_map = {
            'IN1': 16,
            'IN2': 17,
            'IN3': 27,
            'IN4': 18,
            'IN5': 24,
            'IN6': 22,
            'IN7': 4,
            'IN8': 20,
            'DHT11': 21
        }
        self.tools = {
            'humidifier': 'IN1',
            'fan': 'IN2',
            'lamp': 'IN3',
            'heater': 'IN4'
        }
        self.state = {
            'humidifier': False,
            'fan': False,
            'lamp': False,
            'heater': False
        }
        print "Initializing GPIOs"
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_map['IN1'], GPIO.OUT)
        GPIO.setup(self.pin_map['IN2'], GPIO.OUT)
        GPIO.setup(self.pin_map['IN3'], GPIO.OUT)
        GPIO.setup(self.pin_map['IN4'], GPIO.OUT)
        GPIO.setup(self.pin_map['IN5'], GPIO.OUT)
        GPIO.setup(self.pin_map['IN6'], GPIO.OUT)
        GPIO.setup(self.pin_map['IN7'], GPIO.OUT)
        GPIO.setup(self.pin_map['IN8'], GPIO.OUT)
        # GPIO.setup(self.pin_map['DHT11'], GPIO.IN)
        self.sensor = Adafruit_DHT.DHT11
        self.target_temp = 24
        self.target_humidity = 95
        self.lamp_hours = [0, 0, 0, 0, 0, 0, 0, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0]
        self.fan_hours = [0, 1, 0, 0, 0, 1, 0, 1, 0, 1,
                          0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
        self.humidity_low = False

    def Test_pins(self):
        tmp = {}
        for p in self.pin_map.keys():
            if 'IN' in p:
                GPIO.output(self.pin_map[p], GPIO.LOW)
                time.sleep(1)
                GPIO.output(self.pin_map[p], GPIO.HIGH)
                tmp[self.pin_map[p]] = raw_input("Which IN turned ON")
        print tmp

    def Read_Temp_Humidity(self):
        humidity, temperature = Adafruit_DHT.read_retry(
            self.sensor, self.pin_map['DHT11'])
        return humidity. temperature

    def Control(self, tool, state):
        if state == 'ON' and self.state[tool] is False:
            GPIO.output(self.pin_map[self.tools[tool]], GPIO.HIGH)
            self.state[tool] = True
            print "Turning %s : %s" % (tool, state)
        elif state == 'OFF' and self.state[tool] is True:
            GPIO.output(self.pin_map[self.tools[tool]], GPIO.LOW)
            self.state[tool] = False
            print "Turning %s : %s" % (tool, state)

    def run(self):
        h = int(time.strftime("%H"))

        humidity, temperature = None, None
        while humidity is not None and temperature is not None:
            humidity, temperature = self.Read_Temp_Humidity()

        if humidity < self.target_humidity - 1:
            if humidity < 82:
                print "DANGER ZONE, Cutting the fans"
                self.Control('fan', 'OFF')
                self.humidity_low = True
            self.Control('humidifier', 'ON')
            print 'Temp={0:0.1f}*C  Humidity={1:0.1f}%'.format(temperature, humidity)
        else:
            self.Control('humidifier', 'OFF')
            print 'Temp={0:0.1f}*C  Humidity={1:0.1f}%'.format(temperature, humidity)

        if temperature < self.target_temp - 1:
            print 'Temp={0:0.1f}*C  Humidity={1:0.1f}%'.format(temperature, humidity)
            self.Control('heater', 'ON')
        else:
            self.Control('heater', 'OFF')
            print 'Temp={0:0.1f}*C  Humidity={1:0.1f}%'.format(temperature, humidity)

        if self.lamp_hours[h] == 1:
            self.Control('lamp', 'ON')
        else:
            self.Control('lamp', 'OFF')

        if self.fan_hours[h] == 1:
            if not self.humidity_low:
                self.Control('fan', 'ON')
        else:
            self.Control('fan', 'OFF')

if __name__ == '__main__':
    Controler = Controler()
    while True:
        Controler.run()
        time.sleep(1)