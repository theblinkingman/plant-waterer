#!/usr/bin/env python3
import time
from adafruit_datetime import datetime, timedelta
import board
import digitalio
import analogio

motor_enable = digitalio.DigitalInOut(board.D10)
motor_enable.direction = digitalio.Direction.OUTPUT
motor1 = digitalio.DigitalInOut(board.D9)
motor1.direction = digitalio.Direction.OUTPUT
motor2 = digitalio.DigitalInOut(board.D8)
motor2.direction = digitalio.Direction.OUTPUT
motor1.value = False
motor2.value = False
motor_enable.value = False

def run_motor(sec):
    motor_enable.value = True
    time.sleep(0.1)
    motor1.value = True
    time.sleep(sec)
    motor1.value = False
    motor_enable.value = False

sensor = analogio.AnalogIn(board.A0)

threshhold = int(1.4/3.3 * 65535)
window_hours = 24
max_waterings = 8
watering_time = 45
avg_window = 10

logs = []

def get_last_waterings():
    count = 0
    for timestamp in logs:
        if datetime.now() - timestamp < timedelta(hours=window_hours):
            count += 1
    return count

def main():
    while True:
        reading = 0
        for i in range(avg_window):
            reading += sensor.value
        reading /= avg_window
        timestamp = datetime.now()
        log_line = "{}\t{:>5.3f}".format(timestamp, reading)
        print(log_line)
        
        if reading < threshhold:
            if get_last_waterings() <= max_waterings:
                logs.append(timestamp)
                print(timestamp.isoformat() + "\tWatering!!! %d" % get_last_waterings()) 
                run_motor(watering_time)

        # wait an hour and log again
        time.sleep(1 * 60 * 60)


if __name__ == "__main__":
    main()

