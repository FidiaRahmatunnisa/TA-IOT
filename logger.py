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
            writer.writerow(['ts1', 'ts2', 'ts3', 'lokasi'])

            print(f"[INFO] Logging ke {CSV_FILE}... Tekan Ctrl+C untuk berhenti.\n")

            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                parts = line.split(',')
                if len(parts) != 8:
                    continue

                try:
                    ts1 = int(parts[0])
                    ts2 = int(parts[2])
                    ts3 = int(parts[4])

                    min_ts = min(ts1, ts2, ts3)
                    if min_ts == ts1:
                        lokasi = "Dekat Mic1"
                    elif min_ts == ts2:
                        lokasi = "Dekat Mic2"
                    else:
                        lokasi = "Dekat Mic3"

                    # Hanya simpan ts1, ts2, ts3, lokasi
                    writer.writerow([ts1, ts2, ts3, lokasi])
                    print(f"[DATA] ts1={ts1}, ts2={ts2}, ts3={ts3} --> {lokasi}")

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
