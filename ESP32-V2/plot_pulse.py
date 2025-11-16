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
# FIGURE LAYOUT - TWO COLUMNS
# ============================================================================

fig = plt.figure(figsize=(16, 12))
gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1.2], hspace=0.5, wspace=0.3)

# ---------------------------------------------------------------------------
# PPG Plot (Top-left)
# ---------------------------------------------------------------------------
ax_ppg = fig.add_subplot(gs[0, 0])
line_ppg, = ax_ppg.plot(ppg, color='tab:blue')
ax_ppg.set_ylim(-2000, 2000)
ax_ppg.set_title("PPG Signal")
ax_ppg.set_xlabel("Samples")
ax_ppg.set_ylabel("Amplitude")
text_ppg = ax_ppg.text(0.95, 0.95, '', transform=ax_ppg.transAxes,
                       ha='right', va='top', fontsize=10, color='red')

# ---------------------------------------------------------------------------
# Pressure Plot (Top-right)
# ---------------------------------------------------------------------------
ax_pressure = fig.add_subplot(gs[0, 1])
line_pressure, = ax_pressure.plot(pressure, color='tab:orange')
ax_pressure.set_ylim(0, 250)
ax_pressure.set_title("Pressure")
ax_pressure.set_xlabel("Samples")
ax_pressure.set_ylabel("mmHg")
text_pressure = ax_pressure.text(0.95, 0.95, '', transform=ax_pressure.transAxes,
                                 ha='right', va='top', fontsize=10, color='red')

# ---------------------------------------------------------------------------
# Combined Plot (Bottom, spans two columns)
# ---------------------------------------------------------------------------
ax_combined = fig.add_subplot(gs[1, :])
line_comb_ppg, = ax_combined.plot(ppg, color='tab:blue', alpha=0.7, label='PPG')

ax2 = ax_combined.twinx()
line_comb_pressure, = ax2.plot(pressure, color='tab:red', alpha=0.7, label='Pressure')

ax_combined.set_title("Combined View (Dual Axis)")
ax_combined.set_xlabel("Samples")
ax_combined.set_ylabel("PPG Amplitude")
ax2.set_ylabel("Pressure (mmHg)")

# FIXED AXIS RANGES
ax_combined.set_ylim(-2000, 2000)
ax2.set_ylim(0, 220)

# Text overlays for real-time values
text_comb_ppg = ax_combined.text(
    0.02, 0.95, "", transform=ax_combined.transAxes,
    ha="left", va="top", fontsize=10, color="blue"
)
text_comb_pressure = ax2.text(
    0.98, 0.95, "", transform=ax2.transAxes,
    ha="right", va="top", fontsize=10, color="red"
)

# ---------------------------------------------------------------------------
# TEXT PANEL - split into two columns
# ---------------------------------------------------------------------------
# ax_text = fig.add_axes([0.01, 0.01, 0.98, 0.15])  # fixed axes below plots
ax_text = fig.add_subplot(gs[2, :])
ax_text.axis('off')

# Create initial text content
initial_text = "Detector Values Will Appear Here..."
text_box = ax_text.text(
    0.0, 1.0,
    initial_text,
    fontsize=11,
    va='top',
    ha='left',
    family='monospace',
)

# ============================================================================
# LIVE UPDATE LOOP
# ============================================================================

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


    ax_combined.relim()
    ax_combined.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()

    # Update numerical readouts
    text_ppg.set_text(f'{ppg[-1]:.1f}')
    text_pressure.set_text(f'{pressure[-1]:.1f} mmHg')
    text_comb_ppg.set_text(f"PPG: {ppg[-1]:.1f}")
    text_comb_pressure.set_text(f"Pressure: {pressure[-1]:.1f} mmHg")

    # Update text box - up to 4 columns, 10 detectors each
    col1, col2, col3, col4 = [], [], [], []
    for i, name in enumerate(detector_names):
        line = f"{name:20s}: {detection_data[name][-1]:8.2f}"
        if i < 10:
            col1.append(line)
        elif i < 20:
            col2.append(line)
        elif i < 30:
            col3.append(line)
        elif i < 40:
            col4.append(line)

    # Pad columns so they all have equal height
    max_len = max(len(col1), len(col2), len(col3), len(col4))
    for col in (col1, col2, col3, col4):
        while len(col) < max_len:
            col.append("")

    # Combine into rows
    combined_lines = [
        f"{l1}      {l2}      {l3}      {l4}"
        for l1, l2, l3, l4 in zip(col1, col2, col3, col4)
    ]

    text_box.set_text("\n".join(combined_lines))

    return [
        line_ppg, line_pressure, line_comb_ppg, line_comb_pressure, text_box
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
plt.show()