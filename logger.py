import serial
import csv

PORT = 'COM5'
BAUD_RATE = 115200
CSV_FILE = 'data_log.csv'

def main():
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        print(f"[INFO] Membuka port {PORT}...")

        with open(CSV_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Kolom sudah dalam detik
            writer.writerow(['ts1 (s)', 'ts2 (s)', 'ts3 (s)', 'lokasi'])

            print(f"[INFO] Logging ke {CSV_FILE}... Tekan Ctrl+C untuk berhenti.\n")

            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                parts = line.split(',')
                if len(parts) != 8:
                    continue

                try:
                    ts1_us = int(parts[0])
                    ts2_us = int(parts[2])
                    ts3_us = int(parts[4])

                    # Konversi ke detik (float)
                    ts1 = ts1_us / 1_000_000
                    ts2 = ts2_us / 1_000_000
                    ts3 = ts3_us / 1_000_000

                    # Tentukan lokasi
                    min_ts = min(ts1, ts2, ts3)
                    if min_ts == ts1:
                        lokasi = "Dekat Mic1"
                    elif min_ts == ts2:
                        lokasi = "Dekat Mic2"
                    else:
                        lokasi = "Dekat Mic3"

                    writer.writerow([ts1, ts2, ts3, lokasi])
                    print(f"[DATA] ts1={ts1:.6f}, ts2={ts2:.6f}, ts3={ts3:.6f} --> {lokasi}")

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
