import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re

# --------------------------------------------------
# SERIAL SETUP
# --------------------------------------------------
ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)

# Reset ESP32
ser.dtr = False
ser.rts = False
time.sleep(0.2)
ser.dtr = True
ser.rts = True
time.sleep(0.2)

ser.reset_input_buffer()
print("Listening for pressure data...\n")

# --------------------------------------------------
# DATA STORAGE (UNLIMITED)
# --------------------------------------------------
time_ms = []
gauge_pressure = []

start_time = time.time()

# Regex to extract gauge pressure
pattern = re.compile(
    r"Gauge Pressure \(mmHg\): ([\d\.]+)"
)

# --------------------------------------------------
# PLOT SETUP
# --------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 5))
line_gauge, = ax.plot([], [], label="Gauge Pressure (mmHg)", color='tab:red')

ax.set_title("Live Motor Pressure Output")
ax.set_xlabel("Time (seconds)")
ax.set_ylabel("Pressure (mmHg)")
ax.legend()
ax.grid(True)

# --------------------------------------------------
# UPDATE FUNCTION
# --------------------------------------------------
def update(frame):
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        match = pattern.search(line)

        if match:
            pressure = float(match.group(1))
            t = (time.time() - start_time)

            time_ms.append(t)
            gauge_pressure.append(pressure)

    line_gauge.set_data(time_ms, gauge_pressure)

    # Auto-scale axes
    ax.relim()
    ax.autoscale_view()

    return line_gauge,

# --------------------------------------------------
# CLEAN EXIT
# --------------------------------------------------
def on_close(event):
    ser.close()
    print("Serial port closed")

fig.canvas.mpl_connect('close_event', on_close)

ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
plt.show()
