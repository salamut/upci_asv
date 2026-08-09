import csv

input_file = "gngga_only.txt"
output_file = "gps_data.csv"

with open(input_file, "r") as infile, open(output_file, "w", newline="") as outfile:

    writer = csv.writer(outfile)

    writer.writerow([
        "Latitude",
        "Lat Dir",
        "Longitude",
        "Lon Dir"
    ])

    for line in infile:

        if line.startswith("$GNGGA"):

            parts = line.strip().split(",")

            if len(parts) > 5:

                latitude = parts[2]
                lat_dir = parts[3]
                longitude = parts[4]
                lon_dir = parts[5]

                writer.writerow([
                    latitude,
                    lat_dir,
                    longitude,
                    lon_dir
                ])

print("CSV berhasil dibuat")