import csv

with open("datartk.txt", "r") as infile, \
     open("gps_data.csv", "w", newline='') as outfile:

    writer = csv.writer(outfile)

    writer.writerow([
        "UTC",
        "Latitude",
        "Lat Dir",
        "Longitude",
        "Lon Dir",
        "Fix",
        "Satellite"
    ])

    for line in infile:
        if line.startswith("$GNGGA"):
            data = line.split(',')

            writer.writerow([
                data[1],
                data[2],
                data[3],
                data[4],
                data[5],
                data[6],
                data[7]
            ])

print("CSV berhasil dibuat")