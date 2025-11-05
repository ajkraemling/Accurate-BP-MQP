import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

# --- Configure serial connection ---
# Run `ls /dev/cu.*` in terminal to find your ESP32 port
ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)
time.sleep(2)  # Allow ESP32 to reset

ser.reset_input_buffer()
ser.reset_output_buffer()

print("Collecting first 400 readings to auto-set y-axis range...\n")

calibration_data = []

# --- Collect initial calibration samples ---
while len(calibration_data) < 400:
    if ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        if line.startswith("IR="):
            try:
                val = float(line.split('=')[1])
                calibration_data.append(val)
            except (ValueError, IndexError):
                pass

# --- Compute Y-axis range based on calibration data ---
y_min = np.min(calibration_data)
y_max = np.max(calibration_data)

# Add a small buffer around the range (5% padding)
padding = 0.05 * (y_max - y_min)
y_min -= padding
y_max += padding

print(f"✅ Calibration complete:")
print(f"   Y-axis range set to {y_min:.0f} – {y_max:.0f}\n")

# --- Prepare live plotting ---
window = 250
ir_data = deque([0.0]*window, maxlen=window)

fig, ax = plt.subplots(figsize=(9, 5))
line_ir, = ax.plot(ir_data, color='tab:red', linewidth=1.3)
ax.set_title("Live IR Signal (MAX30105 + ESP32)")
ax.set_xlabel("Samples")
ax.set_ylabel("IR Value")
ax.set_ylim(y_min, y_max)
ax.grid(True)

text_ir = ax.text(0.95, 0.90, '', transform=ax.transAxes,
                  ha='right', va='top', fontsize=10, color='red')

def update(frame):
    for _ in range(10):  # Read multiple lines per frame
        if not ser.in_waiting:
            break
        line = ser.readline().decode(errors='ignore').strip()
        if not line.startswith("IR="):
            continue
        try:
            value = float(line.split('=')[1])
            ir_data.append(value)
        except (IndexError, ValueError):
            continue

    x = range(len(ir_data))
    line_ir.set_data(x, ir_data)
    text_ir.set_text(f'IR: {ir_data[-1]:.0f}')
    ax.set_xlim(0, len(ir_data))
    return line_ir, text_ir

ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.tight_layout()
plt.show(block=True)
