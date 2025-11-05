import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# --- Configure serial connection ---
# Run `ls /dev/cu.*` in terminal to find your ESP32’s port
ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)
time.sleep(2)  # Allow ESP32 to reset

# --- Clear buffers ---
ser.reset_input_buffer()
ser.reset_output_buffer()

print("Starting live IR plot...\n")

# --- Rolling data window ---
window = 250
ir_data = deque([0.0]*window, maxlen=window)

# --- Setup plot ---
fig, ax = plt.subplots(figsize=(9, 5))
line_ir, = ax.plot(ir_data, color='tab:red', linewidth=1.3)
ax.set_title("Live IR Signal (MAX30105 + ESP32)")
ax.set_xlabel("Samples")
ax.set_ylabel("IR Value")
ax.set_ylim(82000, 87000)  # adjust based on your readings
ax.grid(True)

# --- Text overlay for latest value ---
text_ir = ax.text(0.95, 0.90, '', transform=ax.transAxes,
                  ha='right', va='top', fontsize=10, color='red')

# --- Update function for animation ---
def update(frame):
    for _ in range(10):  # read multiple lines per frame
        if not ser.in_waiting:
            break
        line = ser.readline().decode(errors='ignore').strip()
        if not line or not line.startswith("IR="):
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

# --- Animate and show plot ---
ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.tight_layout()
plt.show(block=True)
