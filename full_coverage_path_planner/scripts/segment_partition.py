#!/usr/bin/env python3

import numpy as np

# Sample boolean matrix (replace this with your matrix)

# original_matrix = [
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
# ]

# boolean_matrix = [[val == 1 for val in row] for row in original_matrix]
boolean_matrix = np.array([[True, True, False],
                           [True, False, False],
                           [False, False, True]])

# Convert boolean matrix to list of indices where value is True
data_points = []
for i in range(boolean_matrix.shape[0]):
    for j in range(boolean_matrix.shape[1]):
        if boolean_matrix[i, j]:
            data_points.append([i, j])

# Apply k-means clustering
from sklearn.cluster import KMeans

k = 2  # Number of clusters (adjust as needed)
kmeans = KMeans(n_clusters=k)
kmeans.fit(data_points)

# Get cluster labels and centroids
cluster_labels = kmeans.labels_
cluster_centroids = kmeans.cluster_centers_

# Segment the boolean matrix based on cluster labels
segmented_matrix = np.zeros_like(boolean_matrix, dtype=int)
for point, label in zip(data_points, cluster_labels):
    segmented_matrix[point[0], point[1]] = label + 1  # Add 1 to distinguish from background (0)

print("Segmented Matrix:")
print(segmented_matrix)
