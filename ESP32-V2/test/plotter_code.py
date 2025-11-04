import serial
import time
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# --- Configure serial connection ---
# You can check your exact port by running:
#   ls /dev/cu.*

ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)

time.sleep(2)  # Allow ESP32 to reset

# --- Clear any initial junk data ---
ser.reset_input_buffer()
ser.reset_output_buffer()

print("Starting live MAX30105 plot...\n")

# --- Rolling data windows ---
window = 250
red_data = deque([0.0]*window, maxlen=window)
ir_data = deque([0.0]*window, maxlen=window)
green_data = deque([0.0]*window, maxlen=window)

# --- Figure setup ---
fig, ax = plt.subplots(figsize=(9, 5))
line_red, = ax.plot(red_data, color='tab:red', linewidth=1.2, label='Red')
line_ir, = ax.plot(ir_data, color='tab:orange', linewidth=1.2, label='IR')
line_green, = ax.plot(green_data, color='tab:green', linewidth=1.2, label='Green')

ax.set_title("Live MAX30105 Readings (ESP32)")
ax.set_xlabel("Samples")
ax.set_ylabel("Amplitude")
ax.legend(loc='upper right')
ax.set_ylim(0, 200000)
ax.grid(True)

# --- Regex to extract R[], IR[], G[] values ---
pattern = re.compile(r"R\[(\d+)\]\s*IR\[(\d+)\]\s*G\[(\d+)\]")

# --- Update function for animation ---
def update(frame):
    for _ in range(10):  # Read multiple lines per frame
        if not ser.in_waiting:
            break
        try:
            line = ser.readline().decode(errors='ignore').strip()
            match = pattern.search(line)
            if match:
                r_val = float(match.group(1))
                ir_val = float(match.group(2))
                g_val = float(match.group(3))

                red_data.append(r_val)
                ir_data.append(ir_val)
                green_data.append(g_val)
        except Exception:
            continue

    x = range(len(red_data))
    line_red.set_data(x, red_data)
    line_ir.set_data(x, ir_data)
    line_green.set_data(x, green_data)
    ax.set_xlim(0, len(red_data))

    return line_red, line_ir, line_green

# --- Animate ---
ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.show(block=True)
