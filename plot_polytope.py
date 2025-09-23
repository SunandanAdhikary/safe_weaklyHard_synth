import numpy as np
import polytope as pc
import matplotlib.pyplot as plt


# Function to plot a polytope
def plot_polytope(poly, color, alpha=0.3, label=None):
    # Get vertices of the polytope
    vertices = pc.extreme(poly)
    # Sort vertices to create a proper polygon
    hull = vertices[np.append(np.argsort(np.arctan2(vertices[:, 1], vertices[:, 0])), 0)]
    
    # Plot the filled polygon
    plt.fill(hull[:, 0], hull[:, 1], color=color, alpha=alpha, label=label)
    # Plot the boundary
    plt.plot(hull[:, 0], hull[:, 1], color=color, linewidth=2)

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
    plot_polytope(poly1, 'blue', label='Polytope 1')
    plot_polytope(poly2, 'green', label='Polytope 2')
    plot_polytope(poly3, 'red', label='Polytope 3')

    # Try to compute union (Note: exact union computation might be complex)
    # Here we'll show the polytopes overlapping which effectively represents their union
    plt.grid(True, alpha=0.3)
    plt.xlabel('X axis')
    plt.ylabel('Y axis')
    plt.title('2D Polytopes Visualization')
    plt.legend()
    plt.axis('equal')  # Make axes equal scale
    plt.xlim(0, 5)
    plt.ylim(0, 4)
    plt.show()

    # Compute intersections between polytopes
    intersection_12 = pc.intersect(poly1, poly2)  # Intersection of polytope 1 and 2
    intersection_23 = pc.intersect(poly2, poly3)  # Intersection of polytope 2 and 3
    intersection_13 = pc.intersect(poly1, poly3)  # Intersection of polytope 1 and 3

    # Create a new figure to show intersections
    plt.figure(figsize=(15, 5))

    # Plot first intersection
    plt.subplot(131)
    plot_polytope(poly1, 'blue', alpha=0.2, label='Polytope 1')
    plot_polytope(poly2, 'green', alpha=0.2, label='Polytope 2')
    plot_polytope(intersection_12, 'purple', alpha=0.5, label='Intersection')
    plt.title('Intersection of Polytopes 1 & 2')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

    # Plot second intersection
    plt.subplot(132)
    plot_polytope(poly2, 'green', alpha=0.2, label='Polytope 2')
    plot_polytope(poly3, 'red', alpha=0.2, label='Polytope 3')
    plot_polytope(intersection_23, 'purple', alpha=0.5, label='Intersection')
    plt.title('Intersection of Polytopes 2 & 3')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

    # Plot third intersection
    plt.subplot(133)
    plot_polytope(poly1, 'blue', alpha=0.2, label='Polytope 1')
    plot_polytope(poly3, 'red', alpha=0.2, label='Polytope 3')
    plot_polytope(intersection_13, 'purple', alpha=0.5, label='Intersection')
    plt.title('Intersection of Polytopes 1 & 3')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

    plt.tight_layout()
    plt.show()