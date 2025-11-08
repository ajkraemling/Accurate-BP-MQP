import serial
import time
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ---------------------------------------------------------------------------------
# MACOS SERIAL PORT SETUP
# ---------------------------------------------------------------------------------
# On macOS, replace 'COM6' with the correct serial device path.
# To find your ESP32 or Arduino port, open Terminal and run:
#     ls /dev/tty.*
#
# You’ll see something like:
#     /dev/tty.usbserial-0001
#     /dev/tty.usbmodem1101
#
# Use the one that starts with /dev/tty.usb or /dev/tty.SLAB_USB.
# Example:
#     ser = serial.Serial('/dev/tty.usbserial-0001', 115200, timeout=1)
#
# If you get a “Permission denied” error, run this in Terminal:
#     sudo chmod 666 /dev/tty.usbserial-0001
#
# ---------------------------------------------------------------------------------
ser = serial.Serial('COM6', 115200, timeout=1)

# --- force ESP32 reset like PlatformIO does ---
ser.dtr = False
ser.rts = False
time.sleep(0.2)
ser.dtr = True
ser.rts = True
time.sleep(0.2)

# --- clear junk from serial buffer ---
ser.reset_input_buffer()
ser.reset_output_buffer()

print("Waiting for calibration to complete...\n")

# --- wait for ESP32 to finish calibration ---
while True:
    try:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue
        print(line)
        if "Calibration complete" in line or "Baseline Pressure" in line or "!" in line:
            print("\nCalibration complete, starting live plot...\n")
            break
    except UnicodeDecodeError:
        continue

# --- Rolling data windows ---
window = 200
ppg = deque([0.0]*window, maxlen=window)
pressure = deque([0.0]*window, maxlen=window)

# --- CSV logging setup ---
csv_filename = f"bp_data_{int(time.time())}.csv"
csv_file = open(csv_filename, 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Timestamp", "PPG", "Pressure", "Systolic"])  # header

# --- Figure layout: 2 rows, 2 columns ---
fig, ((ax_ppg, ax_combined),
      (ax_pressure, _)) = plt.subplots(2, 2, figsize=(10, 6))
fig.subplots_adjust(wspace=0.3, hspace=0.4)

# --- Line plots ---
line_ppg, = ax_ppg.plot(ppg, color='tab:blue')
ax_ppg.set_ylim(0, 4000)
ax_ppg.set_title("PPG Signal")
ax_ppg.set_xlabel("Samples")
ax_ppg.set_ylabel("Amplitude")

line_pressure, = ax_pressure.plot(pressure, color='tab:orange')
ax_pressure.set_ylim(0, 250)
ax_pressure.set_title("Pressure")
ax_pressure.set_xlabel("Samples")
ax_pressure.set_ylabel("mmHg")

line_comb_ppg, = ax_combined.plot(ppg, label='PPG', color='tab:blue')
line_comb_pressure, = ax_combined.plot(pressure, label='Pressure', color='tab:orange')
ax_combined.set_ylim(0, 4000)
ax_combined.set_title("Combined View")
ax_combined.legend(loc='upper left')

# Hide the unused bottom-right subplot
_.axis('off')

# --- Text objects to display latest values ---
text_ppg = ax_ppg.text(0.95, 0.95, '', transform=ax_ppg.transAxes,
                       ha='right', va='top', fontsize=10, color='red')
text_pressure = ax_pressure.text(0.95, 0.95, '', transform=ax_pressure.transAxes,
                                 ha='right', va='top', fontsize=10, color='red')
text_comb_ppg = ax_combined.text(0.95, 0.90, '', transform=ax_combined.transAxes,
                                 ha='right', va='top', fontsize=10, color='tab:blue')
text_comb_pressure = ax_combined.text(0.95, 0.95, '', transform=ax_combined.transAxes,
                                      ha='right', va='top', fontsize=10, color='tab:orange')
text_systolic = ax_combined.text(0.95, 0.95, '', transform=ax_combined.transAxes,
                                 ha='right', va='bottom', fontsize=10, color='blue')

# --- Update function for FuncAnimation ---
def update(frame):
    global last_save_time
    for _ in range(10):
        if not ser.in_waiting:
            break
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue

        # Skip lines without numeric data
        if not any(c.isdigit() for c in line) or ',' not in line:
            continue

        try:
            v1, v2, v3 = map(float, line.split(','))
            ppg.append(v1)
            pressure.append(v2)

            # --- Save to CSV with timestamp ---
            timestamp = time.time()
            csv_writer.writerow([timestamp, v1, v2, v3])
        except ValueError:
            continue

    x_ppg = range(len(ppg))
    x_pressure = range(len(pressure))

    # Update line plots
    line_ppg.set_data(x_ppg, ppg)
    line_pressure.set_data(x_pressure, pressure)
    line_comb_ppg.set_data(x_ppg, ppg)
    line_comb_pressure.set_data(x_pressure, pressure)

    # Update text to show latest values
    text_systolic.set_text(f'{ppg[-1]:.1f}')
    text_pressure.set_text(f'{pressure[-1]:.1f}')
    text_comb_ppg.set_text(f'PPG: {ppg[-1]:.1f}')
    text_comb_pressure.set_text(f'Pressure: {pressure[-1]:.1f}')

    return line_ppg, line_pressure, line_comb_ppg, line_comb_pressure, \
           text_ppg, text_pressure, text_comb_ppg, text_comb_pressure

# --- Close CSV properly when plot window closes ---
def on_close(event):
    csv_file.close()
    ser.close()
    print(f"\nData saved to {csv_filename}")

fig.canvas.mpl_connect('close_event', on_close)

# --- Start animation ---
ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.show(block=True)
