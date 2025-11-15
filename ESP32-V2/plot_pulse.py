# ============================================================================
# Blood Pressure Monitor - Live Data Visualization and CSV Logger
# ============================================================================
import serial
import time
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import os
import numpy as np
from matplotlib.widgets import TextBox     # For scrollable display

# ============================================================================
# SERIAL PORT CONFIGURATION
# ============================================================================

ser = serial.Serial('COM6', 115200, timeout=1)

ser.dtr = False
ser.rts = False
time.sleep(0.2)
ser.dtr = True
ser.rts = True
time.sleep(0.2)

ser.reset_input_buffer()
ser.reset_output_buffer()

print("Waiting for calibration...\n")

while True:
    try:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue
        print(line)
        if "Calibration complete" in line:
            print("\nCalibration complete!\n")
            break
    except UnicodeDecodeError:
        continue

csv_header = None
detector_names = []

while True:
    try:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue
        print(line)

        if line.startswith("Time,Pressure,PPGSignal"):
            csv_header = line
            parts = line.split(',')
            detector_names = parts[3:]

            print(f"\nDetected {len(detector_names)} algorithms:")
            for i, name in enumerate(detector_names):
                print(f"  {i+1}. {name}")
            print("\nStarting...\n")
            break
    except UnicodeDecodeError:
        continue

csv_filename = f"bp_data_{int(time.time())}.csv"
csv_file = open(csv_filename, 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(csv_header.split(','))

window = 200

ppg = deque([0.0]*window, maxlen=window)
pressure = deque([0.0]*window, maxlen=window)
detection_data = {name: deque([0.0]*window, maxlen=window) for name in detector_names}

# ============================================================================
# FIGURE LAYOUT
# ============================================================================

fig = plt.figure(figsize=(16, 22))
gs = fig.add_gridspec(5, 1, hspace=0.6)

# ---------------------------------------------------------------------------
# PPG Plot
# ---------------------------------------------------------------------------
ax_ppg = fig.add_subplot(gs[0, 0])
line_ppg, = ax_ppg.plot(ppg, color='tab:blue')
ax_ppg.set_ylim(0, 4000)
ax_ppg.set_title("PPG Signal")
ax_ppg.set_xlabel("Samples")
ax_ppg.set_ylabel("Amplitude")

# ---------------------------------------------------------------------------
# Pressure Plot
# ---------------------------------------------------------------------------
ax_pressure = fig.add_subplot(gs[1, 0])
line_pressure, = ax_pressure.plot(pressure, color='tab:orange')
ax_pressure.set_ylim(0, 250)
ax_pressure.set_title("Pressure")
ax_pressure.set_xlabel("Samples")
ax_pressure.set_ylabel("mmHg")

# ---------------------------------------------------------------------------
# Combined Plot
# ---------------------------------------------------------------------------
ax_combined = fig.add_subplot(gs[2, 0])
line_comb_ppg, = ax_combined.plot(ppg, color='tab:blue', alpha=0.7)
ax2 = ax_combined.twinx()
line_comb_pressure, = ax2.plot(pressure, color='tab:red', alpha=0.7)

ax_combined.set_title("Combined View (Dual Axis)")
ax_combined.set_ylabel("PPG Amplitude")
ax2.set_ylabel("mmHg")

# ---------------------------------------------------------------------------
# TEXT PANEL (Not a Matplotlib table; fast & scrollable)
# ---------------------------------------------------------------------------
ax_text = fig.add_subplot(gs[3:, 0])
ax_text.axis('off')

# Create a big text box
initial_text = "Detector Values Will Appear Here..."
text_box = ax_text.text(
    0.01, 1.0,
    initial_text,
    fontsize=11,
    va='top',
    ha='left',
    family='monospace',
)

# ============================================================================
# LIVE UPDATE LOOP
# ============================================================================

text_ppg = ax_ppg.text(0.95, 0.95, '', transform=ax_ppg.transAxes,
                       ha='right', va='top', fontsize=10, color='red')
text_pressure = ax_pressure.text(0.95, 0.95, '', transform=ax_pressure.transAxes,
                                 ha='right', va='top', fontsize=10, color='red')

def update(frame):
    for _ in range(5):
        if not ser.in_waiting:
            break

        line = ser.readline().decode(errors='ignore').strip()
        if not line or ',' not in line:
            continue

        try:
            parts = line.split(',')
            timestamp = float(parts[0])
            pres = float(parts[1])
            ppg_val = float(parts[2])

            det_vals = []
            for v in parts[3:]:
                try:
                    det_vals.append(float(v))
                except:
                    det_vals.append(0)

            # Update buffers
            ppg.append(ppg_val)
            pressure.append(pres)

            for i, name in enumerate(detector_names):
                detection_data[name].append(det_vals[i] if i < len(det_vals) else 0)

            csv_writer.writerow(parts)

        except:
            continue

    # Update plots
    x = range(len(ppg))
    line_ppg.set_data(x, ppg)
    line_pressure.set_data(x, pressure)
    line_comb_ppg.set_data(x, ppg)
    line_comb_pressure.set_data(x, pressure)

    # Numerical readouts on plots
    text_ppg.set_text(f'{ppg[-1]:.1f}')
    text_pressure.set_text(f'{pressure[-1]:.1f} mmHg')

    # -----------------------------------------------------------------------
    # UPDATE THE TEXT PANEL
    # -----------------------------------------------------------------------
    text_content = []

    for name in detector_names:
        text_content.append(f"{name:20s} : {detection_data[name][-1]:8.2f}")

    # Join lines, update the panel
    text_box.set_text("\n".join(text_content))

    return [
        line_ppg,
        line_pressure,
        line_comb_ppg,
        line_comb_pressure,
        text_box,
    ]

# ============================================================================
# Cleanup on close
# ============================================================================
def on_close(event):
    csv_file.close()
    ser.close()
    print(f"\nSaved: {csv_filename}")

fig.canvas.mpl_connect('close_event', on_close)

ani = animation.FuncAnimation(fig, update, interval=50, blit=False)

plt.subplots_adjust(
    left=0.05, right=0.95,
    top=0.98, bottom=0.03,
)

plt.show()
