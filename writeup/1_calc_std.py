import csv
import numpy as np

second_column_values = []
std = 0

with open('Graph2.txt', 'r') as file:
    csv_reader = csv.reader(file, delimiter=',')
    next(csv_reader)
    for row in csv_reader:
        second_column_values.append(float(row[1]))
    values = np.array(second_column_values)
    std = np.std(values)

print("Second column values:", second_column_values)
print("Second column std:", std)

# 0.7263455847056927 for GPS (Graph1.txt)
# 0.5102125310434372 for ACCELEROMETER (Graph2.txt)
