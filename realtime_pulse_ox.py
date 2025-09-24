#!/usr/bin/env python3
import time
import math
import numpy as np
from collections import deque

import RPi.GPIO as GPIO
from smbus2 import SMBus

# MAX30102 driver (pip: max30102)
import max30102

# Heart rate processing
import heartpy as hp

# 16x2 LCD via PCF8574 backpack
from RPLCD.i2c import CharLCD

# ----------- Config ----------
I2C_BUS_ID = 1
LCD_I2C_ADDR = 0x27      # change if your backpack is different (e.g., 0x3F)
SAMPLE_RATE_HZ = 100     # effective sampling
WINDOW_SECONDS = 5
PROCESS_EVERY_SEC = 1

GREEN_PIN = 17
RED_PIN = 27

# Alert thresholds
MIN_BPM = 50
MAX_BPM = 120
MIN_SPO2 = 92

# -----------------------------

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GREEN_PIN, GPIO.OUT)
    GPIO.setup(RED_PIN, GPIO.OUT)
    GPIO.output(GREEN_PIN, GPIO.LOW)
    GPIO.output(RED_PIN, GPIO.LOW)

def blink(pin, duration=0.1):
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(pin, GPIO.LOW)

def compute_spo2(red, ir):
    """
    Very lightweight ratio-of-ratios estimate.
    Clinical-grade SpO2 needs proper calibration; this is suitable for demo.
    """
    red = np.asarray(red, dtype=np.float64)
    ir  = np.asarray(ir,  dtype=np.float64)

    if len(red) < 100 or len(ir) < 100:
        return None

    # Remove obvious negatives/zeros
    red = red[red > 0]
    ir  = ir[ir > 0]
    if len(red) < 50 or len(ir) < 50:
        return None

    # DC = mean, AC = std (simple proxy over short window)
    red_dc = np.mean(red)
    ir_dc  = np.mean(ir)
    red_ac = np.std(red)
    ir_ac  = np.std(ir)

    if red_dc == 0 or ir_dc == 0 or ir_ac == 0:
        return None

    R = (red_ac / red_dc) / (ir_ac / ir_dc)

    # Empirical mapping often used in hobby projects:
    spo2 = 110.0 - 25.0 * R
    # Constrain to plausible range
    spo2 = max(70.0, min(100.0, spo2))
    return round(spo2, 1)

def main():
    setup_gpio()

    # LCD init
    lcd = CharLCD(i2c_expander='PCF8574', address=LCD_I2C_ADDR,
                  port=I2C_BUS_ID, cols=16, rows=2, dotsize=8)
    lcd.clear()
    lcd.write_string('PulseOx Init...')

    # Sensor init
    m = max30102.MAX30102(bus=I2C_BUS_ID)
    # Configure sensor (defaults are generally fine; tweak if noisy)
    # Examples:
    # m.set_led_pulse_amplitude(0x24, 0x24, 0x00)  # red, IR, green (off)

    # Buffers
    maxlen = WINDOW_SECONDS * SAMPLE_RATE_HZ
    red_buf = deque(maxlen=maxlen)
    ir_buf  = deque(maxlen=maxlen)
    last_process = time.time()

    try:
        lcd.clear()
        lcd.write_string('Place finger...')
        time.sleep(1.5)

        while True:
            # Read available samples from FIFO
            red_list, ir_list = m.read_sequential()  # returns lists; may be empty between FIFO fills

            if red_list and ir_list:
                red_buf.extend(red_list)
                ir_buf.extend(ir_list)

            now = time.time()
            if now - last_process >= PROCESS_EVERY_SEC and len(ir_buf) > SAMPLE_RATE_HZ*2:
                last_process = now

                # Copy buffers for processing
                red_np = np.array(red_buf, dtype=np.float64)
                ir_np  = np.array(ir_buf,  dtype=np.float64)

                # Heart rate from IR channel using heartpy
                try:
                    wd, m_out = hp.process(ir_np, sample_rate=SAMPLE_RATE_HZ)
                    bpm = float(m_out['bpm'])
                    if math.isnan(bpm) or bpm <= 0:
                        bpm = None
                except Exception:
                    bpm = None

                # SpO2
                spo2 = compute_spo2(red_np, ir_np)

                # LED logic
                alert = False
                if spo2 is not None and spo2 < MIN_SPO2:
                    alert = True
                if bpm is not None and (bpm < MIN_BPM or bpm > MAX_BPM):
                    alert = True

                if alert:
                    GPIO.output(GREEN_PIN, GPIO.LOW)
                    blink(RED_PIN, duration=0.05)  # fast blink
                else:
                    GPIO.output(RED_PIN, GPIO.LOW)
                    GPIO.output(GREEN_PIN, GPIO.HIGH)

                # LCD update
                line1 = f'HR:{int(bpm):3d} bpm' if bpm is not None else 'HR: --  bpm'
                line2 = f'SpO2:{spo2:5.1f}%' if spo2 is not None else 'SpO2:  --.-%'
                lcd.home()
                lcd.write_string(line1.ljust(16)[:16])
                lcd.cursor_pos = (1, 0)
                lcd.write_string(line2.ljust(16)[:16])

            # Modest sleep to avoid a tight loop; sensor FIFO drives the pace
            time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    finally:
        lcd.clear()
        lcd.write_string('Shutting down')
        GPIO.output(GREEN_PIN, GPIO.LOW)
        GPIO.output(RED_PIN, GPIO.LOW)
        GPIO.cleanup()

if __name__ == '__main__':
    main()
