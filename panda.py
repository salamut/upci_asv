import pandas as pd
import folium

# =========================
# FILE CSV
# =========================
file_csv = "gps_data.csv"

# =========================
# LOAD CSV
# =========================
df = pd.read_csv(file_csv)

# Ambil hanya data longitude timur
df = df[df["Lon Dir"] == "E"]

# Hapus data kosong
df = df.dropna(subset=[
    "Latitude",
    "Longitude",
    "Lat Dir",
    "Lon Dir"
])

# Pastikan format angka valid
df = df[df["Latitude"].astype(str).str.contains(r"\.", na=False)]
df = df[df["Longitude"].astype(str).str.contains(r"\.", na=False)]

# =========================
# KONVERSI LATITUDE
# =========================
def convert_lat(lat_str, direction):

    lat = float(lat_str)

    deg = int(lat / 100)
    minutes = lat - (deg * 100)

    dec = deg + (minutes / 60)

    if direction == "S":
        dec *= -1

    return dec

# =========================
# KONVERSI LONGITUDE
# =========================
def convert_lon(lon_str, direction):

    lon = float(lon_str)

    deg = int(lon / 100)
    minutes = lon - (deg * 100)

    dec = deg + (minutes / 60)

    if direction == "W":
        dec *= -1

    return dec

# =========================
# CONVERT TO DECIMAL DEGREE
# =========================
df["lat_dd"] = df.apply(
    lambda row: convert_lat(row["Latitude"], row["Lat Dir"]),
    axis=1
)

df["lon_dd"] = df.apply(
    lambda row: convert_lon(row["Longitude"], row["Lon Dir"]),
    axis=1
)

# =========================
# LIST KOORDINAT
# =========================
coords = list(zip(df["lat_dd"], df["lon_dd"]))

# Cek data kosong
if len(coords) == 0:
    print("❌ Tidak ada data GPS valid")
    exit()

# =========================
# CENTER MAP
# =========================
map_center = [
    df["lat_dd"].mean(),
    df["lon_dd"].mean()
]

# =========================
# BUAT MAP
# =========================
m = folium.Map(
    location=map_center,
    zoom_start=24,
    max_zoom=30,
    control_scale=True
)

# =========================
# SATELLITE LAYER
# =========================
folium.TileLayer(
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr="Esri Satellite",
    name="Satellite",
    overlay=False,
    control=True,
    max_zoom=30,
    max_native_zoom=24
).add_to(m)

# =========================
# OPENSTREETMAP LAYER
# =========================
folium.TileLayer(
    "OpenStreetMap",
    name="OpenStreetMap",
    max_zoom=30
).add_to(m)

# =========================
# TRAJECTORY LINE
# =========================
folium.PolyLine(
    coords,
    color="red",
    weight=2,
    opacity=1.0,
    smooth_factor=0,
    tooltip="GPS RTK Trajectory"
).add_to(m)

# =========================
# GPS POINTS
# =========================
for i, (lat, lon) in enumerate(coords):

    folium.CircleMarker(
        location=[lat, lon],
        radius=1,
        color="yellow",
        fill=True,
        fill_color="yellow",
        fill_opacity=1.0,
        popup=f"""
        <b>Point {i}</b><br>
        Latitude : {lat:.8f}<br>
        Longitude: {lon:.8f}
        """
    ).add_to(m)

# =========================
# LAYER CONTROL
# =========================
folium.LayerControl(collapsed=False).add_to(m)

# =========================
# AUTO FIT
# =========================
m.fit_bounds(coords)

# =========================
# SAVE HTML
# =========================
m.save("gps_map.html")

print("✅ Map berhasil dibuat: gps_map.html")