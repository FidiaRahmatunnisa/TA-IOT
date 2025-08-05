import serial
import csv
import time

# Konfigurasi port serial sesuai ESP Anda (cek di Device Manager / dmesg)
ser = serial.Serial('COM5', 115200, timeout=1)

# Buat file CSV dan header jika belum ada
with open('toa_data.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['M1_TOA_us', 'M2_TOA_us'])

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith("M1:") and "M2:" in line:
                # Parse nilai
                try:
                    parts = line.replace("M1:", "").replace("M2:", "").split(",")
                    t1 = int(parts[0])
                    t2 = int(parts[1])
                    writer.writerow([t1, t2])
                    print(f"Simpan: M1={t1}, M2={t2}")
                except Exception as e:
                    print(f"Error parsing line: {line} -> {e}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Stopped by user")
