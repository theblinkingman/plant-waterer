# pyright: reportShadowedImports=false
#!/usr/bin/env python3
import time
import board
import digitalio
import analogio
import adafruit_displayio_ssd1306
import displayio
from adafruit_display_text import label
from digitalio import DigitalInOut, Direction
import terminalio
import os
import wifi
import socketpool
import microcontroller
import ipaddress
from adafruit_httpserver import server, response
import busio
import onewireio

# Setup OLED display
displayio.release_displays()
i2c = busio.I2C(board.GP15, board.GP14)
display_bus = displayio.I2CDisplay(i2c, device_address=0x3c)
display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=128, height=32, rotation = 180)

# setup text area
text_area = label.Label(terminalio.FONT, text="Hello World")
text_area.x = 2
text_area.y = 7
display.show(text_area)

log_line = "Connecting to WiFi %s" % os.getenv('WIFI_SSID')
text_area.text = log_line
print()
print(log_line)

wifi.radio.hostname = "waterer"
#  connect to your SSID
wifi.radio.connect(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))

print("Connected to WiFi")

pool = socketpool.SocketPool(wifi.radio)

#  onboard LED setup
led = DigitalInOut(board.LED)
led.direction = Direction.OUTPUT
led.value = True

# Motor control pins
motor1 = digitalio.DigitalInOut(board.GP16)
motor1.direction = digitalio.Direction.OUTPUT
motor2 = digitalio.DigitalInOut(board.GP17)
motor2.direction = digitalio.Direction.OUTPUT
motor1.value = False
motor2.value = False


def run_motor(sec):
    time.sleep(0.1)
    motor2.value = True
    time.sleep(sec)
    motor2.value = False

# TDR sensor
sensor = analogio.AnalogIn(board.A2)
# sensor = onewireio.OneWire(board.GP28)
# Water level pot
threshold = analogio.AnalogIn(board.A0)
# Determined experimentally
# threshhold = int(1.43/3.3 * 65535)
window_hours = 24
max_waterings = 8
watering_time = 45
avg_window = 50

logs = []

def get_last_waterings():
    count = 0
    for timestamp in logs:
        if time.monotonic_ns() - timestamp < (window_hours * 60 * 60 * 1000 * 1000):
            count += 1
    return count

def read_sensor():
    return sensor.value

def main():
    last_water = 0
    while True:
        reading = 0
        for i in range(avg_window):
            reading += read_sensor()
        reading /= avg_window
        log_line = "Sen:\t{:>5.3f}\nSet:\t{:>5.3f}".format(reading/65535 * 3.3, threshold.value/65535 * 3.3)
        text_area.text = log_line
        print(log_line)
        print()
        
        current = time.monotonic_ns()
        
        # Only water once an hour if needed
        if reading < threshold.value and (current - last_water) > (60 * 60 * 1000 * 1000):
            if get_last_waterings() <= max_waterings:
                last_water = current
                logs.append(last_water)
                # print(timestamp.isoformat() + "\tWatering!!! %d" % get_last_waterings()) 
                text_area.text = "Watering!"
                # run_motor(watering_time)

        time.sleep(1)


if __name__ == "__main__":
    main()

