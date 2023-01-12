# pyright: reportShadowedImports=false
#!/usr/bin/env python3
import time
import board
import digitalio
import analogio
import adafruit_displayio_ssd1306
import displayio
from adafruit_display_text import label
import terminalio

# Motor control pins
motor_enable = digitalio.DigitalInOut(board.D8)
motor_enable.direction = digitalio.Direction.OUTPUT
motor1 = digitalio.DigitalInOut(board.D3)
motor1.direction = digitalio.Direction.OUTPUT
motor2 = digitalio.DigitalInOut(board.D2)
motor2.direction = digitalio.Direction.OUTPUT
motor1.value = False
motor2.value = False
motor_enable.value = False

# Just turn it high for FULL POWER
motor_pwm = digitalio.DigitalInOut(board.D7)
motor_pwm.direction = digitalio.Direction.OUTPUT
motor_pwm.value = True

# Setup OLED display
displayio.release_displays()
i2c = board.I2C()
display_bus = displayio.I2CDisplay(i2c, device_address=0x3c)
display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=128, height=32, rotation = 180)

# setup text area
text_area = label.Label(terminalio.FONT, text="Hello World")
text_area.x = 2
text_area.y = 7
display.show(text_area)

def run_motor(sec):
    motor_enable.value = True
    time.sleep(0.1)
    motor2.value = True
    time.sleep(sec)
    motor2.value = False
    motor_enable.value = False

# TDR sensor
sensor = analogio.AnalogIn(board.A0)
# Water level pot
threshold = analogio.AnalogIn(board.A1)
# Determined experimentally
# threshhold = int(1.43/3.3 * 65535)
window_hours = 24
max_waterings = 8
watering_time = 45
avg_window = 10

logs = []

def get_last_waterings():
    count = 0
    for timestamp in logs:
        if time.monotonic_ns() - timestamp < (window_hours * 60 * 60 * 1000 * 1000):
            count += 1
    return count

def main():
    last_water = 0
    while True:
        reading = 0
        for i in range(avg_window):
            reading += sensor.value
        reading /= avg_window
        log_line = "Sen:\t{:>5.3f}\nSet:\t{:>5.3f}".format(reading/65535 * 3.3, threshold.value/65535 * 3.3)
        text_area.text = log_line
        
        current = time.monotonic_ns()
        
        # Only water once an hour if needed
        if reading < threshold.value and (current - last_water) > (60 * 60 * 1000 * 1000):
            if get_last_waterings() <= max_waterings:
                last_water = current
                logs.append(last_water)
                # print(timestamp.isoformat() + "\tWatering!!! %d" % get_last_waterings()) 
                text_area.text = "Watering!"
                run_motor(watering_time)

        time.sleep(10)


if __name__ == "__main__":
    main()

