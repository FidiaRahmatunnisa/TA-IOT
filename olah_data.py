import pandas as pd

# Baca file CSV
df = pd.read_csv("toa_full_data.csv")

# Hilangkan spasi di nama kolom
df.columns = df.columns.str.strip()

# Daftar kolom yang akan dianalisis
kolom_kolom = [
    'M1_STE', 'M1_ZCR', 'M1_PEAK',
    'M2_STE', 'M2_ZCR', 'M2_PEAK',
]

# File output
output_file = "hasil_statistik_05082025_2.txt"

with open(output_file, "w") as f:
    f.write("=== HASIL PERHITUNGAN STATISTIK ===\n\n")
    
    for sensor in ['M1', 'M2']:
        f.write(f"--- Statistik untuk {sensor} ---\n")
        for fitur in ['STE', 'ZCR', 'PEAK']:
            kolom = f"{sensor}_{fitur}"
            if kolom in df.columns:
                data = df[kolom]
                f.write(f"{kolom}:\n")
                f.write(f"  Mean     : {data.mean():.2f}\n")
                f.write(f"  Std Dev  : {data.std():.2f}\n")
                f.write(f"  Min      : {data.min():.2f}\n")
                f.write(f"  Max      : {data.max():.2f}\n\n")
            else:
                f.write(f"[PERINGATAN] Kolom '{kolom}' tidak ditemukan di file CSV.\n\n")
    
    f.write("=== Selesai ===\n")

print(f"Hasil statistik telah disimpan ke file '{output_file}'")
