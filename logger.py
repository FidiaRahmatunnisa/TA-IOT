import os
import serial
import csv

PORT = 'COM5'
BAUD_RATE = 115200
CSV_FILE = 'data_log.csv'

def main():
    try:
        # Hapus file lama jika ada
        if os.path.exists(CSV_FILE):
            os.remove(CSV_FILE)
            print(f"[INFO] File lama '{CSV_FILE}' dihapus.")

        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        print(f"[INFO] Membuka port {PORT}...")

        with open(CSV_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['waktu1 (s)', 'waktu2 (s)', 'waktu3 (s)', 'lokasi'])

            print(f"[INFO] Logging ke {CSV_FILE}... Tekan Ctrl+C untuk berhenti.\n")

            last_entry = None

            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or line.startswith("Mic1"):
                    continue

                parts = line.split(',')
                if len(parts) != 4:
                    continue

                try:
                    ts1 = int(parts[0]) / 1_000_000
                    ts2 = int(parts[1]) / 1_000_000
                    ts3 = int(parts[2]) / 1_000_000
                    lokasi = parts[3].strip()

                    current_entry = (round(ts1, 6), round(ts2, 6), round(ts3, 6), lokasi)

                    if current_entry != last_entry:
                        writer.writerow(current_entry)
                        print(f"[DATA] {current_entry}")
                        last_entry = current_entry

                except ValueError:
                    print(f"[WARNING] Gagal parsing: {line}")

    except KeyboardInterrupt:
        print("\n[INFO] Logging dihentikan.")
    except serial.SerialException as e:
        print(f"[ERROR] Tidak bisa membuka port: {e}")
    except Exception as e:
        print(f"[ERROR] Terjadi error: {e}")

if __name__ == '__main__':
    main()
