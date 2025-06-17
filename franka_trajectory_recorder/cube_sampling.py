import numpy as np

def sample_cube_position():
    x = np.random.uniform(0.4, 0.6)
    y = np.random.uniform(-0.3, 0.3)
    return np.array([x, y])

def sample_three_cubes(min_dist=0.04):
    while True:
        p1 = sample_cube_position()
        p2 = sample_cube_position()
        p3 = sample_cube_position()
        
        # Check pairwise distances
        if (np.linalg.norm(p1 - p2) > min_dist and
            np.linalg.norm(p1 - p3) > min_dist and
            np.linalg.norm(p2 - p3) > min_dist):
            return p1, p2, p3

# Example usage:
for i in range(5):
    p1, p2, p3 = sample_three_cubes()
    print(f"Trajectory {i+1}:")
    print(f" Cube 1: {p1}")
    print(f" Cube 2: {p2}")
    print(f" Cube 3: {p3}")
    print("---")
