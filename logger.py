import serial
import csv
from datetime import datetime

# === Konfigurasi ===
PORT = 'COM5'            # Ganti dengan COM port ESP32 kamu (misal: COM3, COM6, dsb)
BAUD_RATE = 115200
CSV_FILE = 'data_log.csv'  # Nama file output

def main():
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        print(f"[INFO] Membuka port {PORT}...")

        # Buat file CSV dan tulis header
        with open(CSV_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['ts1', 'ts2', 'ts3'])

            print(f"[INFO] Logging ke {CSV_FILE}... Tekan Ctrl+C untuk berhenti.\n")

            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                parts = line.split(',')

                if len(parts) == 8:
                    writer.writerow(parts)
                    print(f"[DATA] {parts}")

    except KeyboardInterrupt:
        print("\n[INFO] Logging dihentikan.")
    except serial.SerialException as e:
        print(f"[ERROR] Tidak bisa membuka port: {e}")
    except Exception as e:
        print(f"[ERROR] Terjadi error: {e}")

if __name__ == '__main__':
    main()
