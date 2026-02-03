# ============================================================================
# Blood Pressure Monitor - Live Plot + CSV Logger (FINAL VERSION)
# ============================================================================

import serial
import time
import csv
import matplotlib
matplotlib.use('TkAgg')  # Force backend
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import os

print("Starting Blood Pressure Monitor...")

# ============================================================================
# SERIAL SETUP
# ============================================================================

PORT = 'COM5'
BAUD = 115200

try:
    print(f"Connecting to {PORT} at {BAUD} baud...")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    
    ser.dtr = False
    ser.rts = False
    time.sleep(0.2)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.2)
    
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    print("✓ Connected to serial")
except Exception as e:
    print(f"✗ Failed to connect: {e}")
    exit(1)

# ============================================================================
# WAIT FOR HEADER (handles both formats)
# ============================================================================

print("Waiting for stream header...")

header_found = False
timeout_counter = 0

while not header_found and timeout_counter < 150:  # 15 second timeout
    try:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            timeout_counter += 1
            time.sleep(0.1)
            continue
        
        # Show what we're receiving
        if "Detector:" in line:
            # Skip detector config lines
            continue
        
        print(f"Received: {line}")
        
        # Check for header (either format)
        if (line.startswith("time,pressure,rawPPG,ppg") or 
            line.startswith("Time,Pressure,PPGSignal,rawPPGSignal")):
            header_found = True
            print("✓ Header found! Starting data collection...")
            break
            
    except Exception as e:
        print(f"Error reading serial: {e}")
        timeout_counter += 1

if not header_found:
    print("✗ Timeout waiting for header. Check your device output.")
    ser.close()
    exit(1)

# ============================================================================
# CSV FILES
# ============================================================================

try:
    cwd = os.getcwd()
    print(f"Working directory: {cwd}")
    
    data_dir = os.path.join(cwd, "data")
    os.makedirs(data_dir, exist_ok=True)
    
    timestamp_str = str(int(time.time()))
    csv_filename = os.path.join(data_dir, f"bp_data_{timestamp_str}.csv")
    
    csv_file = open(csv_filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["time","pressure","rawPPG","ppg"])
    
    print(f"✓ CSV file: {csv_filename}")
    
except Exception as e:
    print(f"✗ Failed to create CSV: {e}")
    ser.close()
    exit(1)

summary_rows = []
in_summary = False
data_count = 0

# ============================================================================
# PLOTTING SETUP
# ============================================================================

print("Setting up plots...")

window = 300

ppg = deque([0.0]*window, maxlen=window)
raw_ppg = deque([0.0]*window, maxlen=window)
raw_ppg_scaled = deque([0.0]*window, maxlen=window)  # For scaled display
pressure = deque([0.0]*window, maxlen=window)

fig = plt.figure(figsize=(16, 10))
gs = fig.add_gridspec(2, 2, height_ratios=[1.5, 1], width_ratios=[1, 1], hspace=0.35, wspace=0.3)

# ---------------- COMBINED VIEW (Top, spans both columns) ----------------
ax_combined = fig.add_subplot(gs[0, :])

# Left axis: PPG signals
line_ppg_combined, = ax_combined.plot(ppg, label="PPG (Filtered)", color='darkred', linewidth=1.5)
line_raw_scaled_combined, = ax_combined.plot(raw_ppg_scaled, label="Raw PPG (scaled)", 
                                    color='lightcoral', alpha=0.7, linewidth=1)

ax_combined.set_ylim(-2000, 2000)
ax_combined.set_xlim(0, window)
ax_combined.set_ylabel("PPG Amplitude", color='darkred')
ax_combined.tick_params(axis='y', labelcolor='darkred')
ax_combined.legend(loc='upper left')

# Right axis: Pressure
ax_pressure_combined = ax_combined.twinx()
line_pressure_combined, = ax_pressure_combined.plot(pressure, color='blue', 
                                                     linewidth=1.5, label="Pressure")

ax_pressure_combined.set_ylim(0, 250)
ax_pressure_combined.set_ylabel("Pressure (mmHg)", color='blue')
ax_pressure_combined.tick_params(axis='y', labelcolor='blue')
ax_pressure_combined.legend(loc='upper right')

ax_combined.set_title("Combined View - PPG (left axis) & Pressure (right axis)")
ax_combined.set_xlabel("Samples")

# Text overlays for current values
text_combined = ax_combined.text(0.5, 0.95, '', transform=ax_combined.transAxes, 
                                 ha='center', va='top', fontsize=10, 
                                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# ---------------- PPG SIGNALS (Bottom-left) ----------------
ax_ppg = fig.add_subplot(gs[1, 0])

# Show both PPG signals like the combined view
line_ppg, = ax_ppg.plot(ppg, label="PPG (Filtered)", color='darkred', linewidth=1.5)
line_raw_scaled, = ax_ppg.plot(raw_ppg_scaled, label="Raw PPG (scaled)", 
                                color='lightcoral', alpha=0.7, linewidth=1)

ax_ppg.set_ylim(-2000, 2000)
ax_ppg.set_xlim(0, window)
ax_ppg.set_title("PPG Signals")
ax_ppg.set_xlabel("Samples")
ax_ppg.set_ylabel("Amplitude")
ax_ppg.legend()

text_ppg = ax_ppg.text(0.95, 0.95, '', transform=ax_ppg.transAxes, 
                       ha='right', va='top', fontsize=10, color='red')

# ---------------- PRESSURE (Bottom-right) ----------------
ax_pressure = fig.add_subplot(gs[1, 1])
line_pressure, = ax_pressure.plot(pressure, color='blue', linewidth=1)

ax_pressure.set_ylim(0, 250)
ax_pressure.set_xlim(0, window)
ax_pressure.set_title("Pressure")
ax_pressure.set_xlabel("Samples")
ax_pressure.set_ylabel("mmHg")

text_pressure = ax_pressure.text(0.95, 0.95, '', transform=ax_pressure.transAxes,
                                 ha='right', va='top', fontsize=10, color='red')

print("✓ Plots ready")

# ============================================================================
# UPDATE LOOP
# ============================================================================

def update(frame):
    global in_summary, data_count
    
    # Process multiple lines per frame
    for _ in range(5):
        
        if not ser.in_waiting:
            break
        
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue
        
        # SUMMARY SECTION
        if line == "#SUMMARY_START":
            print("\n✓ Receiving summary...")
            in_summary = True
            continue
        
        if line == "#SUMMARY_END":
            print(f"✓ Summary complete ({len(summary_rows)} detections)\n")
            in_summary = False
            continue
        
        if in_summary:
            parts = line.split(',')
            if len(parts) == 4:
                summary_rows.append(parts)
            continue
        
        # NORMAL DATA
        if ',' not in line:
            continue
        
        try:
            parts = line.split(',')
            
            if len(parts) != 4:
                continue
            
            t = float(parts[0])
            pres = float(parts[1])
            raw = float(parts[2])
            ppg_val = float(parts[3])
            
            # Update deques
            ppg.append(ppg_val)
            raw_ppg.append(raw)
            # Scale raw PPG for combined view: subtract 2000 and scale down
            raw_ppg_scaled.append((raw - 2000) / 2.0)
            pressure.append(pres)
            
            # Write to CSV
            csv_writer.writerow(parts)
            data_count += 1
            
            # Print progress every 200 samples
            if data_count % 200 == 0:
                print(f"Samples: {data_count}, Pressure: {pres:.1f} mmHg, PPG: {ppg_val:.1f}")
            
        except ValueError:
            pass
    
    # Update plots
    x = range(len(ppg))
    
    # Combined view (top - spans both columns)
    line_ppg_combined.set_data(x, ppg)
    line_raw_scaled_combined.set_data(x, raw_ppg_scaled)
    line_pressure_combined.set_data(x, pressure)
    
    # PPG signals view (bottom-left)
    line_ppg.set_data(x, ppg)
    line_raw_scaled.set_data(x, raw_ppg_scaled)
    
    # Pressure view (bottom-right)
    line_pressure.set_data(x, pressure)
    
    # Update text displays
    if len(ppg) > 0:
        text_combined.set_text(f'PPG: {ppg[-1]:.1f} | Raw: {raw_ppg[-1]:.0f} | Pressure: {pressure[-1]:.1f} mmHg')
        text_ppg.set_text(f'PPG: {ppg[-1]:.1f} | Raw: {raw_ppg[-1]:.0f}')
        text_pressure.set_text(f'{pressure[-1]:.1f} mmHg')
    
    return (line_ppg_combined, line_raw_scaled_combined, line_pressure_combined, 
            line_ppg, line_raw_scaled, line_pressure, 
            text_combined, text_ppg, text_pressure)

# ============================================================================
# CLEANUP
# ============================================================================

def on_close(event):
    print("\n" + "="*60)
    print("Saving data...")
    
    csv_file.close()
    
    # Append summary
    if summary_rows:
        with open(csv_filename, 'a', newline='') as f:
            w = csv.writer(f)
            w.writerow([])
            w.writerow(["#SUMMARY_START"])
            w.writerow(["Detector","Timestamp","Pressure","Confidence"])
            w.writerows(summary_rows)
            w.writerow(["#SUMMARY_END"])
        print(f"✓ Summary appended ({len(summary_rows)} detections)")
    
    ser.close()
    
    print(f"\n✓ COMPLETE!")
    print(f"  File: {csv_filename}")
    print(f"  Samples: {data_count}")
    print("="*60)

# ============================================================================
# RUN
# ============================================================================

print("\n✓ Starting live plot...")
print("Close the window to stop and save.\n")

fig.canvas.mpl_connect('close_event', on_close)
ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.tight_layout()
plt.show()