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
import ipaddress
from adafruit_httpserver import server, response
import busio
import adafruit_si5351
import adafruit_pioasm
import rp2pio

# Setup OLED display
displayio.release_displays()
i2c = busio.I2C(board.GP15, board.GP14)
display_bus = displayio.I2CDisplay(i2c, device_address=0x3c)
display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=128, height=32, rotation = 180)

# setup text area
text_area = label.Label(terminalio.FONT, text="Hello World")
text_area.x = 2
text_area.y = 7

log_line = "Connecting to WiFi %s" % os.getenv('WIFI_SSID')
text_area.text = log_line
# print()
# print(log_line)

bitmap = displayio.Bitmap(display.width, 1, 2)
palette = displayio.Palette(2)
palette[0] = 0
palette[1] = 0xFFFFFF

tile_grid = displayio.TileGrid(bitmap, pixel_shader=palette)
tile_grid.y = 30

group = displayio.Group()
group.append(tile_grid)
group.append(text_area)

display.show(group)

def show_dots(n):
    bitmap.fill(0)
    for i in range(1, min(n*2, display.width), 2):
        bitmap[i] = 1

show_dots(5)

# wifi.radio.hostname = "waterer"
#  connect to your SSID
# wifi.radio.connect(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))

# print("Connected to WiFi")

# pool = socketpool.SocketPool(wifi.radio)

#  onboard LED setup
led = DigitalInOut(board.LED)
led.direction = Direction.OUTPUT
led.value = True

# Motor control pins
motor1 = digitalio.DigitalInOut(board.GP27)
motor1.direction = digitalio.Direction.OUTPUT
motor2 = digitalio.DigitalInOut(board.GP22)
motor2.direction = digitalio.Direction.OUTPUT
motor1.value = False
motor2.value = False


def run_motor(sec):
    time.sleep(0.1)
    motor2.value = True
    time.sleep(sec)
    motor2.value = False

# TDR sensor
# sensor = analogio.AnalogIn(board.A2)
# sensor = onewireio.OneWire(board.GP28)
# Water level pot
threshold = analogio.AnalogIn(board.A2)
# Determined experimentally
# threshhold = int(1.43/3.3 * 65535)

# TDR Sensor pins
enable = digitalio.DigitalInOut(board.GP8)
enable.direction = digitalio.Direction.OUTPUT
enable.value = False

start_pin = board.GP6
trigger_pin = board.GP9

done = digitalio.DigitalInOut(board.GP26)
done.direction = digitalio.Direction.INPUT
done.pull = digitalio.Pull.UP

cs = digitalio.DigitalInOut(board.GP5)
cs.direction = digitalio.Direction.OUTPUT
cs.value = True
spi = busio.SPI(board.GP2, MISO=board.GP4, MOSI=board.GP3)

def open_spi():
    while not spi.try_lock():
        pass
    spi.configure(baudrate=2000000, phase=0, polarity=0)

CONFIG1_ADDR = 0
CONFIG2_ADDR = 1
INT_STATUS_ADDR = 2
INT_MASK_ADDR = 3
TIME1_ADDR = 0x10
CLOCK1_ADDR = 0x11
TIME2_ADDR = 0x12
CLOCK2_ADDR = 0x13
CAL1_ADDR = 0x1B
CAL2_ADDR = 0x1C

CAL_MASK = (1 << 23) - 1
TIME_MASK = (1 << 23) - 1
COUNT_MASK = (1 << 16) - 1

AUTO_INC_MASK = 0b10000000
RW_BIT = 0b01000000
REG_MASK = 0b0011111

def read_register(reg_addr, reg_size=1):
    result = bytearray(reg_size + 1)

    control = reg_addr & REG_MASK
    open_spi()
    try:
        cs.value = False
        spi.write_readinto(out_buffer=bytes([control] + [0] * reg_size), in_buffer=result)
    finally:
        cs.value = True
        spi.unlock()
    return int.from_bytes(result[1:], 'big', signed=False)

def write_register(reg_addr, data):
    control_data = reg_addr & REG_MASK
    control_data |= RW_BIT
    open_spi()
    try:
        cs.value = False
        spi.write(bytes([control_data, data]))
    finally:
        cs.value = True
        spi.unlock()
    return

def config_si5351():
    i2c = busio.I2C(board.GP1, board.GP0)
    si = adafruit_si5351.SI5351(i2c)
    si.pll_a.configure_integer(30)
    si.clock_0.configure_integer(si.pll_a, 50)
    si.outputs_enabled = True
    print('Clock 0: {0:0.3f} MHz'.format(si.clock_0.frequency/1000000))
    return si

clock = config_si5351()

def calc_tof_mode2(clock, time1, time2, count, cal1, cal2):
    freq = clock.clock_0.frequency
    period = 1/freq
    calCount = cal2 - cal1 # using default cal periods of 2
    normLSB = period / calCount
    TOF = normLSB * (time1 - time2) + count * period
    return TOF

def calc_tof_mode1(clock, time, cal1, cal2):
    if time == 0 or cal1 == 0 or cal2 == 0:
        return -1
    freq = clock.clock_0.frequency
    period = 1/freq
    calCount = (cal2 - cal1)/9 # using default cal periods of 10 
    if calCount == 0:
        return -1
    normLSB = period / calCount
    return time * normLSB

samples = []
next_samp = 0
window_size = 10
samp_min = 8
samp_max = 50
def add_sample(new_samp):
    if new_samp > samp_max or new_samp < samp_min:
        return -1
    global next_samp
    if next_samp < len(samples):
        samples[next_samp] = new_samp
    else:
        samples.append(new_samp)
    next_samp = (next_samp + 1) % window_size

    return median(samples)

def median(lst):
    n = len(lst)
    s = sorted(lst)
    return (s[n//2-1]/2.0+s[n//2]/2.0, s[n//2])[n % 2] if n else None    

# This will trigger a pulse whenever the TDC requests it
trigger_pio = '''
.program trigger
    wait 1 pin 0    ; wait for trigger pin to be set
    set pins, 1     ; send pulse 
    set pins, 0
    ; loop back to the beginning
'''
triggered_asm = adafruit_pioasm.assemble(trigger_pio)
sm = rp2pio.StateMachine(triggered_asm,
                        frequency=10000,
                        first_set_pin=start_pin,
                        first_in_pin=trigger_pin)

def config_tdc():
    calibration_periods = 1 # 1 = 10 periods
    calibration_shift = 6
    avg_cycles = 0b111 # 128 cycles
    avg_cycles_shift = 3
    reg_value = (calibration_periods << calibration_shift) | (avg_cycles << avg_cycles_shift)
    write_register(CONFIG2_ADDR, reg_value)
    read_back = read_register(CONFIG2_ADDR)
    print("CONFIG2 = %X" % read_back)

window_hours = 24
max_waterings = 8
watering_time = 20
avg_window = 50

logs = []

def get_last_waterings():
    count = 0
    now = time.monotonic_ns()
    for timestamp in logs:
        if now - timestamp < (window_hours * 60 * 60 * 10**9):
            count += 1
    return count

def do_measurement():
    "Time of flight in nanoseconds"

    start_measure = 0b1 # start measurement mode 1
    write_register(CONFIG1_ADDR, start_measure)

    # Wait for a measurement to be ready
    while done.value:
        pass
        # print("+", end="")
    # print("")

    time1 = read_register(TIME1_ADDR, reg_size=3) & TIME_MASK
    # print("TIME1 = %s " % hex(time1))
    cal1 = read_register(CAL1_ADDR, reg_size=3) & CAL_MASK
    # print("CAL1 = %s" % hex(cal1))
    cal2 = read_register(CAL2_ADDR, reg_size=3) & CAL_MASK
    # print("CAL2 = %s" % hex(cal2))
    tof = calc_tof_mode1(clock, time1, cal1, cal2)
    # print("TOF = %s" % (tof*10**9))
    return tof * 10**9

last_water = 0

def water(avg):
    global last_water
    current = time.monotonic_ns()

    # Only water once an hour if needed
    if avg < (threshold.value/65535 * 30) and (not last_water or (current - last_water) > (60 * 60 * 10**9)):
        if get_last_waterings() <= max_waterings:
            last_water = current
            logs.append(last_water)
            print(current)
            # print(timestamp.isoformat() + "\tWatering!!! %d" % get_last_waterings()) 
            text_area.text = "Watering!"
            run_motor(watering_time)
            return True
    
    return False

def main():
    enable.value = True
    print("Configuring TDC Chip")
    config_tdc()

    print("Starting read loop")
    counter = 0
    while True:
        # Take a measurement
        avg = add_sample(do_measurement())
        if avg == -1:
            print("-", end="")
            continue
        counter = (counter + 1) % window_size
        if counter == 0:
            print("\nTOF = %s" % (avg))
            water(avg)
        else:
            print(".", end="")

        log_line = "Sen:\t{:>5.3f}\nSet:\t{:>5.3f}".format(avg, threshold.value/65535 * 30)
        text_area.text = log_line
        show_dots(get_last_waterings())
        # print(log_line)
        # print()


if __name__ == "__main__":
    main()

