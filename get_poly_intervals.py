import numpy as np
import polytope as pc
import matplotlib.pyplot as plt

def get_polytope_intervals(poly):
    # For each dimension, find min and max values using vertices
    vertices = pc.extreme(poly)
    intervals = []
    
    # Get min and max for each dimension
    for dim in range(vertices.shape[1]):
        min_val = np.min(vertices[:, dim])
        max_val = np.max(vertices[:, dim])
        intervals.append((min_val, max_val))
    
    return intervals

if __name__ == "__main__":
    # Create 2D polytopes
    # First polytope: Rectangle in the first quadrant
    A1 = np.array([
        [-1, 0],  # x ≥ 1
        [1, 0],   # x ≤ 3
        [0, -1],  # y ≥ 1
        [0, 1]    # y ≤ 2
    ])
    b1 = np.array([-1, 3, -1, 2])
    poly1 = pc.Polytope(A1, b1)

    # Second polytope: Triangle
    A2 = np.array([
        [-1, -1],  # x + y ≥ 2
        [1, 0],    # x ≤ 4
        [0, 1]     # y ≤ 3
    ])
    b2 = np.array([-2, 4, 3])
    poly2 = pc.Polytope(A2, b2)

    # Third polytope: Another rectangle
    A3 = np.array([
        [-1, 0],  # x ≥ 2
        [1, 0],   # x ≤ 4
        [0, -1],  # y ≥ 1.5
        [0, 1]    # y ≤ 3.5
    ])
    b3 = np.array([-2, 4, -1.5, 3.5])
    poly3 = pc.Polytope(A3, b3)
    # Create plot
    plt.figure(figsize=(10, 8))

    # Plot each polytope

    # Get intervals for each polytope and their intersections
    print("Polytope 1 intervals:")
    intervals1 = get_polytope_intervals(poly1)
    print("X:", intervals1[0])
    print("Y:", intervals1[1])

    print("\nPolytope 2 intervals:")
    intervals2 = get_polytope_intervals(poly2)
    print("X:", intervals2[0])
    print("Y:", intervals2[1])

    print("\nPolytope 3 intervals:")
    intervals3 = get_polytope_intervals(poly3)
    print("X:", intervals3[0])
    print("Y:", intervals3[1])

    # plot intervals as rectangles on a 2D plot
    plt.figure(figsize=(10, 8))
    # Plot each polytope's intervals along with the polytopes

    plt.fill([intervals1[0][0], intervals1[0][1], intervals1[0][1], intervals1[0][0]], 
            [intervals1[1][0], intervals1[1][0], intervals1[1][1], intervals1[1][1]], 
            color='blue', alpha=0.3, label='Polytope 1 Intervals')
    plt.fill([intervals2[0][0], intervals2[0][1], intervals2[0][1], intervals2[0][0]],
            [intervals2[1][0], intervals2[1][0], intervals2[1][1], intervals2[1][1]],
            color='green', alpha=0.3, label='Polytope 2 Intervals')
    plt.fill([intervals3[0][0], intervals3[0][1], intervals3[0][1], intervals3[0][0]],
            [intervals3[1][0], intervals3[1][0], intervals3[1][1], intervals3[1][1]],
                    color='red', alpha=0.3, label='Polytope 3 Intervals')
        
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Polytope Intervals Visualization")
    plt.show()