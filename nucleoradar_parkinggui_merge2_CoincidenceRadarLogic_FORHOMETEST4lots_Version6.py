import struct
import math
import binascii
import codecs
import numpy as np
import serial
import threading
import tkinter as tk
from tkinter import messagebox, simpledialog, filedialog
import re
import time
import csv
from datetime import datetime

TC_PASS = 0
TC_FAIL = 1

# ========== CONFIGURATION ==========
RADAR_SERIAL_PORT = "COM3"
RADAR_BAUD_RATE = 921600
NUCLEO_SERIAL_PORT = "COM5"
NUCLEO_BAUD_RATE = 115200
DEFAULT_NUM_LOTS = 4  # Updated for 4 lots
RADAR_PULSE_SEC = 2

# --- CAR DETECTION PARAMETERS (for general detection, no mapping/zones) ---
MIN_RANGE = 0.1
MAX_RANGE = 20.0
MIN_Y = -2.5
MAX_Y = 2.5
MIN_Z = -0.5
MAX_Z = 2.8

MIN_POINTS_FOR_CAR_CLUSTER = 1
DBSCAN_EPS = 0.3
DBSCAN_MIN_SAMPLES = 2

COINCIDENCE_TIMEOUT = 2.0
NUM_SENSORS = 4  # Updated for 4 sensors

# --- NEW: Debounce and state tracking ---
DEBOUNCE_FREE_FRAMES = 3  # Number of consecutive ADC frames below threshold to mark as free
RADAR_CONFIRM_FRAMES = 5  # How many radar frames to process after trigger to confirm

class ParkingLotGUI:
    def __init__(self, master, num_lots):
        self.master = master
        master.title("Parking Lot Visualization")
        self.lot_buttons = []
        self.status_label = tk.Label(master, text="Waiting for STM32...", font=("Arial", 12))
        self.status_label.pack(pady=10)

        self.radar_frame = tk.Frame(master)
        self.radar_label = tk.Label(self.radar_frame, text="Radar: OFF", font=("Arial", 14, "bold"), fg="darkgray")
        self.radar_label.pack(side="left")
        self.radar_bar_canvas = tk.Canvas(self.radar_frame, width=120, height=18, bg="lightgray", highlightthickness=1, highlightbackground="gray")
        self.radar_bar = self.radar_bar_canvas.create_rectangle(0, 0, 0, 18, fill="orange")
        self.radar_bar_canvas.pack(side="left", padx=10)
        self.radar_frame.pack(pady=10)

        frame = tk.Frame(master)
        frame.pack(padx=20, pady=20)
        self.lot_states = [0] * num_lots
        for i in range(num_lots):
            btn = tk.Label(frame, text=f"Lot {i+1}", width=10, height=2, relief="ridge", bg="grey", font=("Arial", 14, "bold"))
            btn.grid(row=i//4, column=i%4, padx=10, pady=10)
            self.lot_buttons.append(btn)

        stats_frame = tk.Frame(master)
        stats_frame.pack(pady=5)
        self.occupied_free_label = tk.Label(stats_frame, text="", font=("Arial", 14))
        self.occupied_free_label.pack(side="top", anchor="w")
        self.stats_total_label = tk.Label(stats_frame, text="", font=("Arial", 12, "italic"))
        self.stats_total_label.pack(side="top", anchor="w")
        self.stats_per_lot_label = tk.Label(stats_frame, text="", font=("Arial", 10))
        self.stats_per_lot_label.pack(side="top", anchor="w")

        self.adc_label = tk.Label(master, text="ADC Value: ---", font=("Arial", 14, "bold"), fg="blue")
        self.adc_label.pack(pady=5)

        save_btn = tk.Button(master, text="Save Session Stats to CSV", command=self.export_csv)
        save_btn.pack(pady=5)

        self.radar_canvas = tk.Canvas(master, width=400, height=240, bg="white", highlightthickness=1, highlightbackground="gray")
        self.radar_canvas.pack(pady=10)
        self.radar_points = []
        self.radar_clusters = []
        self.radar_cluster_lots = []
        self.num_lots = num_lots
        self.lot_states = [0] * num_lots
        self.occupied = 0
        self.free = num_lots
        self.radar_on = False
        self.radar_pulse_start = None
        self.radar_pulse_timer_id = None
        self.session_stats = {
            "cars_detected": 0,
            "radar_pulses": 0,
            "per_lot_usage": [0]*num_lots,
        }
        self.session_events = []
        self.master.after(100, self.process_queue)
        self.queue = []

        self.radar_event_time = 0
        self.radar_delay_sec = RADAR_PULSE_SEC
        self.master.after(200, self.check_radar_off)
        self.update_stats_labels()

    def set_status(self, lot_states, occupied, free):
        for i, state in enumerate(lot_states):
            color = "red" if state else "green"
            self.lot_buttons[i].configure(bg=color)
        self.occupied = occupied
        self.free = free
        self.lot_states = list(lot_states)
        self.update_stats_labels()

    def set_status_for_lot(self, lot_idx, occupied):
        self.lot_states[lot_idx] = 1 if occupied else 0
        color = "red" if occupied else "green"
        self.lot_buttons[lot_idx].configure(bg=color)
        self.occupied = sum(self.lot_states)
        self.free = self.num_lots - self.occupied
        self.update_stats_labels()

    def set_radar(self, radar_on):
        if radar_on:
            self.radar_label.config(text="Radar: ON", fg="green")
            if not self.radar_on:
                self.session_stats["radar_pulses"] += 1
                self.session_events.append(
                    (time.strftime("%Y-%m-%d %H:%M:%S"), "Radar ON")
                )
            self.radar_on = True
            self.radar_pulse_start = time.time()
            self.animate_radar_bar()
        else:
            if self.radar_on:
                self.session_events.append(
                    (time.strftime("%Y-%m-%d %H:%M:%S"), "Radar OFF")
                )
            self.radar_label.config(text="Radar: OFF", fg="red")
            self.radar_on = False
            self.radar_bar_canvas.coords(self.radar_bar, 0, 0, 0, 18)
            if self.radar_pulse_timer_id:
                self.master.after_cancel(self.radar_pulse_timer_id)
                self.radar_pulse_timer_id = None

    def animate_radar_bar(self):
        if not self.radar_on or self.radar_pulse_start is None:
            return
        elapsed = time.time() - self.radar_pulse_start
        fraction = min(elapsed / self.radar_delay_sec, 1.0)
        width = int(120 * (1.0 - fraction))
        self.radar_bar_canvas.coords(self.radar_bar, 0, 0, width, 18)
        if self.radar_on:
            self.radar_pulse_timer_id = self.master.after(30, self.animate_radar_bar)

    def check_radar_off(self):
        if self.radar_on and (time.time() - self.radar_event_time > self.radar_delay_sec):
            self.set_radar(False)
        self.master.after(200, self.check_radar_off)

    def update_stats_labels(self):
        self.occupied_free_label.config(text=f"Occupied: {self.occupied}   Free: {self.free}")
        self.stats_total_label.config(
            text=f"Cars detected (coincidence logic): {self.session_stats['cars_detected']},  Radar pulses: {self.session_stats['radar_pulses']}"
        )
        per_lot_usage = self.session_stats["per_lot_usage"]
        per_lot_str = ", ".join([f"Lot {i+1}: {per_lot_usage[i]}" for i in range(self.num_lots)])
        self.stats_per_lot_label.config(text=f"Per-lot usage: {per_lot_str}")

    def process_queue(self):
        while self.queue:
            what, data = self.queue.pop(0)
            if what == "coincidence_update":
                lot_idx, occupied = data
                self.set_status_for_lot(lot_idx, occupied)
                if occupied:
                    self.session_stats["cars_detected"] += 1
                    self.session_stats["per_lot_usage"][lot_idx] += 1
                    self.session_events.append(
                        (time.strftime("%Y-%m-%d %H:%M:%S"), f"Lot {lot_idx+1} occupied (coincidence)")
                    )
                else:
                    self.session_events.append(
                        (time.strftime("%Y-%m-%d %H:%M:%S"), f"Lot {lot_idx+1} freed")
                    )
            elif what == "lots":
                lot_states, occupied, free, adc_display = data
                self.set_status(lot_states, occupied, free)
                self.adc_label.config(text=f"ADC Value: {adc_display}")
            elif what == "radar":
                radar_on = data
                self.radar_event_time = time.time()
                self.set_radar(radar_on)
            elif what == "adc_value":
                adc_val = data
                self.adc_label.config(text=f"ADC Value: {adc_val}")
            elif what == "radar_points":
                points, car_clusters, cluster_lots = data
                self.radar_points = points
                self.radar_clusters = car_clusters
                self.radar_cluster_lots = cluster_lots
                self.update_radar_canvas()
        self.master.after(100, self.process_queue)

    def coincidence_update(self, lot_idx, occupied):
        self.queue.append(("coincidence_update", (lot_idx, occupied)))

    def update_from_serial(self, lot_states, occupied, free, adc_display):
        self.queue.append(("lots", (lot_states, occupied, free, adc_display)))

    def update_radar(self, radar_on):
        self.queue.append(("radar", radar_on))

    def notify_adc_value(self, adc_val):
        self.queue.append(("adc_value", adc_val))

    def update_radar_points(self, points, car_clusters, cluster_lots):
        self.queue.append(("radar_points", (points, car_clusters, cluster_lots)))

    def export_csv(self):
        filename = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV files", "*.csv")])
        if not filename:
            return
        try:
            with open(filename, "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Session Statistics"])
                writer.writerow(["Cars detected (coincidence logic)", self.session_stats["cars_detected"]])
                writer.writerow(["Radar pulses", self.session_stats["radar_pulses"]])
                writer.writerow([])
                writer.writerow(["Per-lot usage"])
                for i, usage in enumerate(self.session_stats["per_lot_usage"]):
                    writer.writerow([f"Lot {i+1}", usage])
                writer.writerow([])
                writer.writerow(["Session Events"])
                writer.writerow(["Timestamp", "Event"])
                for ts, event in self.session_events:
                    writer.writerow([ts, event])
            messagebox.showinfo("Export Complete", f"Session stats saved to {filename}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to save CSV: {e}")

    def update_radar_canvas(self):
        canvas = self.radar_canvas
        canvas.delete("all")
        width, height = 400, 240

        # Draw radar points
        for pt in self.radar_points:
            x, y, z = pt
            px = width // 2 + int(x * 15)
            py = int(height // 2 - int(y * 40))
            canvas.create_oval(px-2, py-2, px+2, py+2, fill="blue", outline="")

        # Draw cluster centers
        colors = ["#ff4d4d", "#4dff4d"]
        for idx, (cl, cluster_points) in enumerate(self.radar_clusters):
            if len(cluster_points) == 0:
                continue
            mean_x = np.mean(cluster_points[:,0])
            mean_y = np.mean(cluster_points[:,1])
            px = width // 2 + int(mean_x * 15)
            py = int(height // 2 - int(mean_y * 40))
            color = colors[idx % len(colors)]
            canvas.create_oval(px-8, py-8, px+8, py+8, outline=color, width=2)
            canvas.create_text(px, py-14, text=f"Cluster {idx+1}", fill=color, font=("Arial", 8, "bold"))
            canvas.create_text(px, py+14, text=f"({mean_x:.1f},{mean_y:.1f}) n={len(cluster_points)}", fill=color, font=("Arial", 7))

def getUint32(data):
    data = [int(x) for x in data]
    return (data[0] +
            data[1]*256 +
            data[2]*65536 +
            data[3]*16777216)

def getHex(data):
    word = [1, 2**8, 2**16, 2**24]
    data = [int(x) for x in data]
    return np.matmul(data,word)

def checkMagicPattern(data):
    if len(data) < 8:
        return 0
    return (
        data[0] == 2 and data[1] == 1 and data[2] == 4 and data[3] == 3 and
        data[4] == 6 and data[5] == 5 and data[6] == 8 and data[7] == 7
    )

def parser_helper(data, readNumBytes, debug=False):
    headerStartIndex = -1
    for index in range(readNumBytes - 7):
        if checkMagicPattern(data[index:index+8]):
            headerStartIndex = index
            break

    if headerStartIndex == -1:
        totalPacketNumBytes = -1
        numDetObj           = -1
        numTlv              = -1
        subFrameNumber      = -1
        platform            = -1
        frameNumber         = -1
        timeCpuCycles       = -1
    else:
        if headerStartIndex + 40 > readNumBytes:
            return (-1, -1, -1, -1, -1)
        totalPacketNumBytes = getUint32(data[headerStartIndex+12:headerStartIndex+16])
        platform            = getHex(data[headerStartIndex+16:headerStartIndex+20])
        frameNumber         = getUint32(data[headerStartIndex+20:headerStartIndex+24])
        timeCpuCycles       = getUint32(data[headerStartIndex+24:headerStartIndex+28])
        numDetObj           = getUint32(data[headerStartIndex+28:headerStartIndex+32])
        numTlv              = getUint32(data[headerStartIndex+32:headerStartIndex+36])
        subFrameNumber      = getUint32(data[headerStartIndex+36:headerStartIndex+40])

    return (headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber)

def parser_one_mmw_demo_output_packet(data, readNumBytes, debug=False):
    headerNumBytes = 40
    detectedX_array = []
    detectedY_array = []
    detectedZ_array = []
    detectedV_array = []
    detectedRange_array = []

    result = TC_PASS

    (headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber) = parser_helper(data, readNumBytes, debug)

    if headerStartIndex == -1 or headerStartIndex + 40 > readNumBytes:
        result = TC_FAIL
        return (result, -1, -1, -1, -1, -1,
                [],[],[],[],[],[],[],[],[])
    if headerStartIndex + totalPacketNumBytes > readNumBytes:
        result = TC_FAIL
        return (result, headerStartIndex, totalPacketNumBytes, numDetObj, -1, subFrameNumber,
                [],[],[],[],[],[],[],[],[])
    nextHeaderStartIndex = headerStartIndex + totalPacketNumBytes
    if nextHeaderStartIndex + 8 < readNumBytes and checkMagicPattern(data[nextHeaderStartIndex:nextHeaderStartIndex+8]) == 0:
        result = TC_FAIL
        return (result, headerStartIndex, totalPacketNumBytes, numDetObj, -1, subFrameNumber,
                [],[],[],[],[],[],[],[],[])
    if numDetObj <= 0:
        result = TC_FAIL
        return (result, headerStartIndex, totalPacketNumBytes, numDetObj, -1, subFrameNumber,
                [],[],[],[],[],[],[],[],[])
    if subFrameNumber > 3:
        result = TC_FAIL
        return (result, headerStartIndex, totalPacketNumBytes, numDetObj, -1, subFrameNumber,
                [],[],[],[],[],[],[],[],[])

    tlvStart = headerStartIndex + headerNumBytes
    if tlvStart + 8 > readNumBytes:
        result = TC_FAIL
        return (result, headerStartIndex, totalPacketNumBytes, numDetObj, -1, subFrameNumber,
                [],[],[],[],[],[],[],[],[])
    tlvType    = getUint32(data[tlvStart+0:tlvStart+4])
    tlvLen     = getUint32(data[tlvStart+4:tlvStart+8])
    offset = 8

    if tlvType == 1 and tlvLen < totalPacketNumBytes:
        for obj in range(numDetObj):
            if tlvStart + offset + 16 > readNumBytes:
                break
            x = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset:tlvStart + offset+4]),'hex'))[0]
            y = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+4:tlvStart + offset+8]),'hex'))[0]
            z = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+8:tlvStart + offset+12]),'hex'))[0]
            v = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+12:tlvStart + offset+16]),'hex'))[0]
            compDetectedRange = math.sqrt((x * x)+(y * y)+(z * z))
            detectedX_array.append(x)
            detectedY_array.append(y)
            detectedZ_array.append(z)
            detectedV_array.append(v)
            detectedRange_array.append(compDetectedRange)
            offset = offset + 16

    return (result, headerStartIndex, totalPacketNumBytes, numDetObj, 2, subFrameNumber,
            detectedX_array, detectedY_array, detectedZ_array, detectedV_array,
            detectedRange_array, [], [], [], [])

def is_static_point(x, y, z, static_points, tol=0.2):
    for sx, sy, sz in static_points:
        if abs(x-sx)<tol and abs(y-sy)<tol and abs(z-sz)<tol:
            return True
    return False

# --- CLUSTER LOGIC WITH HEIGHT FILTER ADDED ---
def cluster_and_detect_car(x_array, y_array, z_array, range_array, background_points=None):
    from sklearn.cluster import DBSCAN
    points = []
    idx_map = []
    for idx, (x, y, z, r) in enumerate(zip(x_array, y_array, z_array, range_array)):
        if MIN_RANGE <= r <= MAX_RANGE and MIN_Y <= y <= MAX_Y and MIN_Z <= z <= MAX_Z:
            if background_points is None or not is_static_point(x, y, z, background_points):
                points.append([x, y, z])
                idx_map.append(idx)
    if len(points) == 0:
        return False, 0, [], [], idx_map
    points_np = np.array(points)
    clustering = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(points_np)
    labels = clustering.labels_
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    car_detected = False
    car_clusters = []
    for cl in set(labels):
        if cl == -1:
            continue
        cluster_points = points_np[labels == cl]
        mean_z = np.mean(cluster_points[:, 2])
        # Only consider clusters with mean_z < 0.3 meters (maketa height + margin)
        if len(cluster_points) >= MIN_POINTS_FOR_CAR_CLUSTER and mean_z < 0.3:
            car_detected = True
            car_clusters.append((cl, cluster_points))
    return car_detected, n_clusters, labels, car_clusters, idx_map

def radar_nucleo_thread(gui: ParkingLotGUI, num_lots):
    import serial

    try:
        ser_radar = serial.Serial(RADAR_SERIAL_PORT, RADAR_BAUD_RATE, timeout=0.5)
        print(f"Opened {RADAR_SERIAL_PORT} at {RADAR_BAUD_RATE} baud.")
    except Exception as e:
        print(f"Could not open radar serial port {RADAR_SERIAL_PORT}: {e}")
        return

    try:
        ser_nucleo = serial.Serial(NUCLEO_SERIAL_PORT, NUCLEO_BAUD_RATE, timeout=0.5)
        print(f"Opened Nucleo on {NUCLEO_SERIAL_PORT} at {NUCLEO_BAUD_RATE} baud.")
    except Exception as e:
        print(f"Could not open Nucleo serial port {NUCLEO_SERIAL_PORT}: {e}")
        ser_radar.close()
        return

    buffer = bytearray()
    background_points = []
    frames_seen = 0
    BACKGROUND_LEARN_FRAMES = 10

    lot_waiting_for_radar = [False] * num_lots
    lot_last_trigger_time = [0.0] * num_lots
    lot_last_adc = [0] * num_lots
    threshold = 3500
    lot_armed = [True] * num_lots  # For debounce

    # --- NEW state for robust free detection ---
    lot_occupied_state = [False] * num_lots
    lot_free_debounce = [0] * num_lots

    # --- NEW: Radar confirm burst ---
    radar_confirm_count = [0] * num_lots

    magic = b'\x02\x01\x04\x03\x06\x05\x08\x07'

    while True:
        try:
            data = ser_radar.read(4096)
            while ser_nucleo.in_waiting:
                resp = ser_nucleo.readline().decode(errors='ignore').strip()
                if resp.startswith("ADC"):
                    try:
                        matches = re.findall(r'ADC(\d+):(\d+)', resp)
                        if matches:
                            adc_values = {int(ch): int(val) for ch, val in matches}
                            adc_display = " ".join([f"ADC{i+1}:{adc_values.get(i+1, 0)}" for i in range(num_lots)])
                            for i in range(num_lots):
                                adc_val = adc_values.get(i+1, 0)
                                # Debounced logic: Only trigger on rising edge and if not already waiting
                                if lot_armed[i] and adc_val > threshold and not lot_waiting_for_radar[i]:
                                    lot_waiting_for_radar[i] = True
                                    lot_last_trigger_time[i] = time.time()
                                    gui.set_radar(True)
                                    radar_confirm_count[i] = RADAR_CONFIRM_FRAMES
                                    print(f"DRV5055 Lot {i+1} triggered, waiting for radar confirmation...")
                                    lot_armed[i] = False  # Disarm until released
                                # Re-arm when sensor drops below threshold
                                if adc_val <= threshold:
                                    lot_armed[i] = True
                                lot_last_adc[i] = adc_val

                                # --- NEW: Free detection when already occupied ---
                                if lot_occupied_state[i]:
                                    if adc_val <= threshold:
                                        lot_free_debounce[i] += 1
                                        if lot_free_debounce[i] >= DEBOUNCE_FREE_FRAMES:
                                            gui.coincidence_update(i, False)
                                            lot_occupied_state[i] = False
                                            lot_free_debounce[i] = 0
                                            print(f"[FREE] Lot {i+1} is now FREE (ADC below threshold, debounce ok)")
                                    else:
                                        lot_free_debounce[i] = 0
                        else:
                            print(f"ADC parse error: {resp}")
                    except Exception as e:
                        print("ADC parse error:", e)

            if data:
                buffer += data
                while True:
                    result = parser_one_mmw_demo_output_packet(buffer, len(buffer), debug=False)
                    if result[0] == TC_PASS:
                        x_array, y_array, z_array, range_array = result[6], result[7], result[8], result[10]
                        if frames_seen < BACKGROUND_LEARN_FRAMES:
                            background_points.extend(zip(x_array, y_array, z_array))
                            frames_seen += 1
                            print(f"Learning background... frame {frames_seen}/{BACKGROUND_LEARN_FRAMES}")
                            buffer = buffer[result[1]+result[2]:]
                            continue
                        car_detected, n_clusters, labels, car_clusters, idx_map = cluster_and_detect_car(
                            x_array, y_array, z_array, range_array, background_points)
                        gui.update_radar_points([(x_array[i], y_array[i], z_array[i]) for i in range(len(x_array))], car_clusters, [0]*len(car_clusters))
                        # --- For each lot, check if we're in a radar confirmation burst ---
                        for lot_idx in range(num_lots):
                            if lot_waiting_for_radar[lot_idx] or radar_confirm_count[lot_idx] > 0:
                                if car_detected:
                                    gui.coincidence_update(lot_idx, True)
                                    lot_occupied_state[lot_idx] = True
                                    lot_waiting_for_radar[lot_idx] = False
                                    radar_confirm_count[lot_idx] = 0
                                    print(f"[RADAR CONFIRM] Lot {lot_idx+1} OCCUPIED. {len(car_clusters)} cluster(s) detected:")
                                    for cidx, (cl, cluster_points) in enumerate(car_clusters):
                                        mean_x = np.mean(cluster_points[:,0])
                                        mean_y = np.mean(cluster_points[:,1])
                                        mean_z = np.mean(cluster_points[:,2])
                                        print(f"    Cluster {cidx+1}: mean_x={mean_x:.2f}, mean_y={mean_y:.2f}, mean_z={mean_z:.2f}, size={len(cluster_points)}")
                                elif radar_confirm_count[lot_idx] > 0:
                                    radar_confirm_count[lot_idx] -= 1
                                    if radar_confirm_count[lot_idx] == 0:
                                        lot_waiting_for_radar[lot_idx] = False
                                # If radar burst finishes with no car detected, do nothing (wait for ADC to go low for FREE)
                        buffer = buffer[result[1]+result[2]:]
                    else:
                        if len(buffer) > 8192:
                            print("Frame fail (buffer large, out of sync)")
                        idx = buffer.find(magic, 1)
                        if idx != -1:
                            buffer = buffer[idx:]
                        else:
                            if len(buffer) > 16384:
                                buffer = buffer[-8192:]
                            break
            now = time.time()
            for lot_idx in range(num_lots):
                if lot_waiting_for_radar[lot_idx] and (now - lot_last_trigger_time[lot_idx] > COINCIDENCE_TIMEOUT):
                    gui.coincidence_update(lot_idx, False)
                    lot_waiting_for_radar[lot_idx] = False

            time.sleep(0.01)
        except KeyboardInterrupt:
            print("Exiting...")
            ser_radar.close()
            ser_nucleo.close()
            return
        except Exception as ex:
            print(f"Error: {ex}")
            time.sleep(0.1)

def main():
    root = tk.Tk()
    num_lots = 4  # Updated for 4 lots
    gui = ParkingLotGUI(root, num_lots)
    threading.Thread(target=radar_nucleo_thread, args=(gui, num_lots), daemon=True).start()
    root.mainloop()

if __name__ == "__main__":
    main()