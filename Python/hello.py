# shitty, sad way to work with data
import csv
with open("winequality-red.csv", 'r') as f:
    wines = list(csv.reader(f, delimiter=";"))
print(wines[:3])

qualities = [float(item[-1]) for item in wines[1:]]
print(sum(qualities) / len(qualities))

# superior, intellectual way to work with data
import numpy as np
wines = np.array(wines[1:], dtype=np.float)
