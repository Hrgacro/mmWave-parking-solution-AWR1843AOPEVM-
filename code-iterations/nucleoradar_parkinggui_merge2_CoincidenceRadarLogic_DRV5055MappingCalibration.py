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
DEFAULT_NUM_LOTS = 6  # Set to 6 for your 6 sensors
RADAR_PULSE_SEC = 2

# --- CAR DETECTION PARAMETERS (TUNED FOR INDOOR/BIG BOX TESTING) ---
MIN_RANGE = 1.0
MAX_RANGE = 25.0
MIN_Y = -2.5
MAX_Y = 2.5
MIN_Z = -0.5
MAX_Z = 2.8
MIN_POINTS_FOR_CAR_CLUSTER = 5      # lowered for indoor testing
DBSCAN_EPS = 0.7                    # tighter clusters for smaller objects
DBSCAN_MIN_SAMPLES = 3

COINCIDENCE_TIMEOUT = 2.0   # seconds: How long to wait for radar after DRV5055 triggers

NUM_SENSORS = 6

# --- For mapping, don't care about LOT_Y_ZONES yet! ---
LOT_Y_ZONES = [(-2.5, -1.7), (-1.7, -1.0), (-1.0, -0.3), (-0.3, 0.3), (0.3, 1.0), (1.0, 1.7)]

class ParkingLotGUI:
    def __init__(self, master, num_lots):
        self.master = master
        master.title("Mapping Calibration Mode (DRV5055 + Radar)")
        self.lot_buttons = []
        self.status_label = tk.Label(master, text="CALIBRATION: Trigger each sensor when a car/object is in the lot.\nCluster Y will be mapped to lot.", font=("Arial", 10))
        self.status_label.pack(pady=10)

        frame = tk.Frame(master)
        frame.pack(padx=20, pady=20)
        self.lot_states = [0] * num_lots
        for i in range(num_lots):
            btn = tk.Label(frame, text=f"Lot {i+1}", width=10, height=2, relief="ridge", bg="grey", font=("Arial", 14, "bold"))
            btn.grid(row=i//4, column=i%4, padx=10, pady=10)
            self.lot_buttons.append(btn)

        self.mapping_text = tk.Text(master, height=12, width=60, font=("Consolas", 10))
        self.mapping_text.pack(padx=10, pady=10)
        self.mapping_text.insert(tk.END, "Lot mappings (Y):\n")
        self.mapping_text.config(state=tk.DISABLED)
        self.mapping_results = {}

        self.master.after(100, self.process_queue)
        self.queue = []

    def report_mapping(self, lot_idx, mean_y, mean_x, mean_z):
        self.mapping_results[lot_idx] = (mean_y, mean_x, mean_z, datetime.now().strftime("%H:%M:%S"))
        self.mapping_text.config(state=tk.NORMAL)
        self.mapping_text.delete(1.0, tk.END)
        self.mapping_text.insert(tk.END, "Lot mappings (Y):\n")
        for idx in sorted(self.mapping_results):
            y, x, z, t = self.mapping_results[idx]
            self.mapping_text.insert(
                tk.END, f"Lot {idx+1}: mean_y={y:.2f}, mean_x={x:.2f}, mean_z={z:.2f}  ({t})\n"
            )
        self.mapping_text.config(state=tk.DISABLED)
    
    def process_queue(self):
        while self.queue:
            what, data = self.queue.pop(0)
            if what == "mapping":
                lot_idx, mean_y, mean_x, mean_z = data
                self.report_mapping(lot_idx, mean_y, mean_x, mean_z)
        self.master.after(100, self.process_queue)

    def add_mapping(self, lot_idx, mean_y, mean_x, mean_z):
        self.queue.append(("mapping", (lot_idx, mean_y, mean_x, mean_z)))
        
# ========== RADAR PARSER CODE ==========

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
        if len(cluster_points) >= MIN_POINTS_FOR_CAR_CLUSTER:
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

    lot_last_adc = [0] * num_lots
    threshold = 3500

    magic = b'\x02\x01\x04\x03\x06\x05\x08\x07'

    # For mapping: store last car_clusters for mapping
    latest_car_clusters = []
    mapping_done = [False] * num_lots

    while True:
        try:
            data = ser_radar.read(4096)
            # Always process Nucleo serial, regardless of radar state
            while ser_nucleo.in_waiting:
                resp = ser_nucleo.readline().decode(errors='ignore').strip()
                if resp.startswith("ADC"):
                    try:
                        matches = re.findall(r'ADC(\d+):(\d+)', resp)
                        if matches:
                            adc_values = {int(ch): int(val) for ch, val in matches}
                            for i in range(num_lots):
                                adc_val = adc_values.get(i+1, 0)
                                # Only trigger on rising edge, and only if mapping not already done
                                if adc_val > threshold and lot_last_adc[i] <= threshold and not mapping_done[i]:
                                    print(f"[MAPPING] DRV5055 Lot {i+1} triggered. Looking for radar cluster...")
                                    # Find cluster closest to radar (smallest range)
                                    best_cluster = None
                                    best_range = float('inf')
                                    for cl, cluster_points in latest_car_clusters:
                                        if len(cluster_points) == 0:
                                            continue
                                        mean_x = np.mean(cluster_points[:,0])
                                        mean_y = np.mean(cluster_points[:,1])
                                        mean_z = np.mean(cluster_points[:,2])
                                        mean_r = np.mean(np.sqrt(cluster_points[:,0]**2 + cluster_points[:,1]**2 + cluster_points[:,2]**2))
                                        if mean_r < best_range:
                                            best_range = mean_r
                                            best_cluster = (mean_y, mean_x, mean_z)
                                    if best_cluster:
                                        mean_y, mean_x, mean_z = best_cluster
                                        print(f"[MAPPING] Lot {i+1}: mean_y={mean_y:.2f}, mean_x={mean_x:.2f}, mean_z={mean_z:.2f}")
                                        gui.add_mapping(i, mean_y, mean_x, mean_z)
                                        mapping_done[i] = True
                                    else:
                                        print(f"[MAPPING] Lot {i+1}: No radar cluster found!")
                                lot_last_adc[i] = adc_val
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
                        # Save latest clusters for mapping association
                        latest_car_clusters = car_clusters
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
    num_lots = None
    while num_lots is None:
        try:
            num_lots = simpledialog.askinteger(
                "Number of Parking Lots",
                f"Enter number of lots (default {DEFAULT_NUM_LOTS}):",
                minvalue=1, maxvalue=32
            )
            if num_lots is None:
                num_lots = DEFAULT_NUM_LOTS
        except Exception:
            num_lots = DEFAULT_NUM_LOTS

    gui = ParkingLotGUI(root, num_lots)
    threading.Thread(target=radar_nucleo_thread, args=(gui, num_lots), daemon=True).start()
    root.mainloop()

if __name__ == "__main__":
    main()