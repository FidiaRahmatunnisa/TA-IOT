import serial
import csv
import time

# Ganti dengan COM port milikmu
ser = serial.Serial('COM5', 115200, timeout=1)

with open('toa_full_data_05082025_2.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        'M1_TOA_us', 'M1_STE', 'M1_ZCR', 'M1_PEAK',
        'M2_TOA_us', 'M2_STE', 'M2_ZCR', 'M2_PEAK',
        'Lokasi'
    ])

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue

            try:
                # Pisahkan berdasarkan koma
                parts = line.split(',')
                if len(parts) != 9:
                    print(f"Format tidak sesuai (harus 9 kolom): {line}")
                    continue

                # Parsing dan simpan
                row = [
                    int(parts[0]),    # M1_TOA_us
                    float(parts[1]),  # M1_STE
                    float(parts[2]),  # M1_ZCR
                    int(parts[3]),    # M1_PEAK
                    int(parts[4]),    # M2_TOA_us
                    float(parts[5]),  # M2_STE
                    float(parts[6]),  # M2_ZCR
                    int(parts[7]),    # M2_PEAK
                    parts[8].strip()  # Lokasi
                ]

                writer.writerow(row)
                print(f"Tersimpan: {row}")

            except Exception as e:
                print(f"Gagal parsing: {line} -> {e}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Dihentikan oleh user.")
