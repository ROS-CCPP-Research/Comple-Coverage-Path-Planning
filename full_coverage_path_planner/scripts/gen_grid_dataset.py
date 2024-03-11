#!/usr/bin/env python3
import random


def generate_grid_map(rows, cols):
    # Generate a grid map filled with 1s (occupied space)
    grid_map = [[1] * cols for _ in range(rows)]
    
    # Determine the size and position of the free area
    free_area_size = min(rows, cols) // 2
    free_area_start_row = (rows - free_area_size) // 2
    free_area_start_col = (cols - free_area_size) // 2
    free_area_end_row = free_area_start_row + free_area_size
    free_area_end_col = free_area_start_col + free_area_size
    
    # Fill the free area with 0s (free space)
    for row in range(free_area_start_row, free_area_end_row):
        for col in range(free_area_start_col, free_area_end_col):
            grid_map[row][col] = 0
    
    # Define the lengths of the narrow areas for each row
    narrow_area_lengths = [random.randint(1, cols - free_area_size) for _ in range(rows)]
    
    # Add narrow areas around the free space
    for row in range(rows):
        # Ensure the narrow area does not exceed the boundaries
        narrow_area_length = min(narrow_area_lengths[row], cols - free_area_end_col)
        if narrow_area_length > 0:
            start_col = free_area_end_col + random.randint(1, cols - free_area_end_col - 1)
            end_col = start_col + narrow_area_length
            for col in range(start_col, end_col):
                if col < cols:  # Check if the column index is within the grid boundaries
                    grid_map[row][col] = 1
    
    return grid_map

def save_dataset(dataset_size, rows, cols, narrow_area_size_range, filename):
    with open(filename, 'w') as file:
        for _ in range(dataset_size):
            # Generate grid map
            grid_map = generate_grid_map(rows, cols)
            
            # Write grid map to file
            for row in grid_map:
                file.write(''.join(map(str, row)) + '\n')
            file.write('\n')

# Parameters for generating the dataset
dataset_size = 100  # Number of grid maps in the dataset
rows = 80  # Number of rows in each grid map
cols = 80  # Number of columns in each grid map
narrow_area_size_range = 3  # Range of possible narrow area sizes
filename = 'grid_map_dataset.txt'  # Output file name

# Generate and save the dataset
save_dataset(dataset_size, rows, cols, narrow_area_size_range, filename)
print("Dataset generated and saved successfully!")
