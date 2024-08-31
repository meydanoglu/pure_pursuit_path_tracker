import csv

input_file = 'src/pure_pursuit/racelines/velocity_tracks/modena2019.csv'
output_file = 'src/pure_pursuit/racelines/velocity_tracks/modena2019_adjusted.csv'

with open(input_file, 'r') as infile, open(output_file, 'w', newline='') as outfile:
    reader = csv.reader(infile)
    writer = csv.writer(outfile)
    
    # Read the first row to get the initial x and y values
    first_row = next(reader)
    initial_x = float(first_row[0])
    initial_y = float(first_row[1])
    
    # Write the adjusted first row
    writer.writerow([0, 0])
    
    # Iterate through the rest of the rows and adjust the values
    for row in reader:
        x = float(row[0]) - initial_x

        y = float(row[1]) - initial_y
        writer.writerow([x, y])