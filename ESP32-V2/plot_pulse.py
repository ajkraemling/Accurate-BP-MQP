# ============================================================================
# Blood Pressure Monitor - Live Plot + CSV Logger (FAST STREAM VERSION)
# Works with firmware that sends:
#
#   time,pressure,rawPPG,ppg     (continuous)
#   #SUMMARY_START
#   Detector,timestamp,pressure,confidence
#   ...
#   #SUMMARY_END
# ============================================================================

import serial
import time
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import os

# ============================================================================
# SERIAL SETUP
# ============================================================================

PORT = 'COM6'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

ser.dtr = False
ser.rts = False
time.sleep(0.2)
ser.dtr = True
ser.rts = True
time.sleep(0.2)

ser.reset_input_buffer()
ser.reset_output_buffer()

print("Connected to serial")

# ============================================================================
# WAIT FOR HEADER
# ============================================================================

print("Waiting for stream header...")

while True:
    line = ser.readline().decode(errors='ignore').strip()
    if not line:
        continue

    print(line)

    if line.startswith("time,pressure,rawPPG,ppg"):
        break

print("Streaming started\n")

# ============================================================================
# CSV FILES
# ============================================================================

os.makedirs("data", exist_ok=True)

timestamp_str = str(int(time.time()))
csv_filename = f"data/bp_data_{timestamp_str}.csv"
summary_filename = f"data/bp_data_{timestamp_str}_summary.csv"

csv_file = open(csv_filename, 'w', newline='')
csv_writer = csv.writer(csv_file)

csv_writer.writerow(["time","pressure","rawPPG","ppg"])

summary_rows = []
in_summary = False

# ============================================================================
# PLOTTING SETUP
# ============================================================================

window = 300

ppg = deque([0.0]*window, maxlen=window)
raw_ppg = deque([0.0]*window, maxlen=window)
pressure = deque([0.0]*window, maxlen=window)

fig = plt.figure(figsize=(12, 8))
gs = fig.add_gridspec(2, 1, height_ratios=[1,1])

# ---------------- PPG ----------------
ax_ppg = fig.add_subplot(gs[0])
line_ppg, = ax_ppg.plot(ppg, label="Filtered PPG")
line_raw, = ax_ppg.plot(raw_ppg, alpha=0.6, label="Raw PPG")

ax_ppg.set_ylim(-2000, 2000)
ax_ppg.set_title("PPG Signals")
ax_ppg.legend()

# ---------------- Pressure ----------------
ax_pressure = fig.add_subplot(gs[1])
line_pressure, = ax_pressure.plot(pressure)

ax_pressure.set_ylim(0, 250)
ax_pressure.set_title("Pressure (mmHg)")

# ============================================================================
# UPDATE LOOP
# ============================================================================

def update(frame):
    global in_summary

    for _ in range(10):

        if not ser.in_waiting:
            break

        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue

        # --------------------------------------------------
        # SUMMARY SECTION
        # --------------------------------------------------
        if line == "#SUMMARY_START":
            print("Receiving summary...")
            in_summary = True
            continue

        if line == "#SUMMARY_END":
            print("Summary complete")
            in_summary = False
            continue

        if in_summary:
            parts = line.split(',')
            if len(parts) == 4:
                summary_rows.append(parts)
            continue

        # --------------------------------------------------
        # NORMAL DATA
        # --------------------------------------------------
        if ',' not in line:
            continue

        try:
            parts = line.split(',')

            t = float(parts[0])
            pres = float(parts[1])
            raw = float(parts[2])
            ppg_val = float(parts[3])

            ppg.append(ppg_val)
            raw_ppg.append(raw)
            pressure.append(pres)

            csv_writer.writerow(parts)

        except:
            pass

    # update plots
    x = range(len(ppg))
    line_ppg.set_data(x, ppg)
    line_raw.set_data(x, raw_ppg)
    line_pressure.set_data(x, pressure)

    return line_ppg, line_raw, line_pressure


# ============================================================================
# CLEANUP
# ============================================================================

def on_close(event):
    csv_file.close()

    if summary_rows:
        with open(summary_filename, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["Detector","Timestamp","Pressure","Confidence"])
            w.writerows(summary_rows)

        print("Saved summary:", summary_filename)

    ser.close()
    print("Saved raw data:", csv_filename)


fig.canvas.mpl_connect('close_event', on_close)

ani = animation.FuncAnimation(fig, update, interval=40, blit=False)

plt.show()
