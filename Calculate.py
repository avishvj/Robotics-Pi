import math
import numpy as np

def calculate_mean(values):
    sum = np.sum(values)
    return 1/N * sum

def calculate_topLeft(values, mean):
    f = lambda x: (x - mean) * (x - mean)
    return 1/N * np.sum(list(map(f, values)))

def calculate_diagonal(x, y, x_mean, y_mean):
    f = lambda x, y: (x - x_mean) * (y - y_mean)
    return 1/N * np.sum(list(map(f, x, y)))


x = [1, 2, 3, 4, 5, 6, 7, 8, 90, 10]
y = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
N = 10

x_mean = calculate_mean(x)
y_mean = calculate_mean(y)

g = lambda value: (value - x_mean)

topLeft = calculate_topLeft(x, x_mean)
bottomRight = calculate_topLeft(y, y_mean)
diag = calculate_diagonal(x, y, x_mean, y_mean)

p = [[topLeft, diag],[diag, bottomRight]]
print(p[0])
print(p[1])
