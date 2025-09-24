#!/usr/bin/env python3
import time
import numpy as np
import heartpy as hp
import max30102

def compute_spo2(red, ir):
    """
    Simple SpO2 estimate (not medical grade).
    Uses ratio of AC/DC components.
    """
    red = np.array(red, dtype=np.float64)
    ir  = np.array(ir, dtype=np.float64)

    if len(red) < 100 or len(ir) < 100:
        return None

    red = red[red > 0]
    ir  = ir[ir > 0]
    if len(red) < 50 or len(ir) < 50:
        return None

    red_dc = np.mean(red)
    ir_dc  = np.mean(ir)
    red_ac = np.std(red)
    ir_ac  = np.std(ir)

    if red_dc == 0 or ir_dc == 0 or ir_ac == 0:
        return None

    R = (red_ac / red_dc) / (ir_ac / ir_dc)
    spo2 = 110.0 - 25.0 * R
    spo2 = max(70.0, min(100.0, spo2))  # keep in reasonable range
    return round(spo2, 1)

def main():
    m = max30102.MAX30102()
    sample_rate = 100  # Hz
    window_sec = 5

    print("Place your finger on the sensor...")
    time.sleep(2)

    try:
        while True:
            red, ir = m.read_sequential()  # get available samples

            if len(ir) >= window_sec * sample_rate:
                # Heart rate (use IR channel)
                try:
                    wd, measures = hp.process(ir, sample_rate=sample_rate)
                    bpm = measures['bpm']
                except Exception:
                    bpm = None

                # SpO2 estimate
                spo2 = compute_spo2(red, ir)

                # Display
                if bpm is not None:
                    print(f"Heart Rate: {bpm:.1f} bpm", end=" | ")
                else:
                    print("Heart Rate: -- bpm", end=" | ")

                if spo2 is not None:
                    print(f"SpO2: {spo2:.1f}%")
                else:
                    print("SpO2: -- %")

                time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()
