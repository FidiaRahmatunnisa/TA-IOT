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
            writer.writerow(['waktu1 (s)', 'waktu2 (s)', 'waktu3 (s)', 'lokasi'])

            print(f"[INFO] Logging ke {CSV_FILE}... Tekan Ctrl+C untuk berhenti.\n")

            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or line.startswith("Mic1"):
                    continue  # Lewati log yang panjang, ambil hanya yang kita format khusus

                parts = line.split(',')
                if len(parts) != 4:
                    continue

                try:
                    waktu1 = int(parts[0]) / 1_000_000  # µs to s
                    waktu2 = int(parts[1]) / 1_000_000
                    waktu3 = int(parts[2]) / 1_000_000
                    lokasi = parts[3].strip()

                    writer.writerow([waktu1, waktu2, waktu3, lokasi])
                    print(f"[DATA] {waktu1:.6f}, {waktu2:.6f}, {waktu3:.6f} --> {lokasi}")

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
