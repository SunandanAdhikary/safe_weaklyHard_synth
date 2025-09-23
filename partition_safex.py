import numpy as np
import itertools
from polytope import Polytope

def partition_safex(safex, grid_delta, dims):
    """
    Partition the safe space into smaller polytopes using given grid delta.
    Each partition is represented as a Polytope object.
    """
    # dims = len(grid_delta)
    # convert numpy array to list of tuples
    safe_space_bounds = [tuple(row) for row in safex.T]
    # print(f"Safe space bounds: {safe_space_bounds} from {safex}")
    # print(dims)
    # print(safe_space_bounds[2][0])
    partitions = []
    # print([np.arange(safe_space_bounds[i][0], safe_space_bounds[i][1], grid_delta[i]) for i in range(dims)])
    # Use Polytope for partitioning
    ii = 0
    for point in itertools.product(*[np.arange(safe_space_bounds[i][0], safe_space_bounds[i][1], grid_delta[i]) for i in range(dims)]):
        # Each partition is a box, but represented as a Polytope
        ii += 1
        A = []
        b = []
        equations = []
        for i in range(dims):
            # A x <= b representation of box constraints
            lower = point[i]
            upper = point[i] + grid_delta[i]
            # print(f"Dimension {i+1} in [{point[i]}, {point[i] + grid_delta[i]}]")
            # Create equations in the form: x_i <= upper and -x_i <= -lower
            a_upper = np.zeros(dims)
            a_upper[i] = 1
            A.append(a_upper)
            b.append(upper)
            
            a_lower = np.zeros(dims)
            a_lower[i] = -1
            A.append(a_lower)
            b.append(-lower)
            print(f"Creating {str(ii)}-th partition [{lower}, {upper}]")
        partitions.append(Polytope(np.array(A), np.array(b)))
        # for part in partitions:  
        #     for i in range(dims):
        #         print(f'{part.A[2*i][i]}*x{i+1} <= {part.b[2*i]}  {-1*part.A[2*i+1][i]}*x{i+1} >= {-1*part.b[2*i+1]}')
        #         print(f'{part.A[2*i][i]}*x{i+1}_hat <= {part.b[2*i]}  {-1*part.A[2*i+1][i]}*x{i+1}_hat >= {-1*part.b[2*i+1]}')
        return (partitions, ii)
    
    if __name__ == "__main__":
        safex = np.array([[-2, -200, -100, -600],
                            [ 2, 200, 100, 600]])
        grid_delta = [0.001, 0.1, 0.05, 0.1] # [1, 100, 50, 600]
        dims = len(safex[0])
        partitions, count = partition_safex(safex, grid_delta, dims)
        print(f"Partitioned safe space into {str(count)} grid cells using Polytope.")
        for part in partitions:  
            for i in range(dims):
                print(f'{part.A[2*i][i]}*x{i+1} <= {part.b[2*i]}  {-1*part.A[2*i+1][i]}*x{i+1} >= {-1*part.b[2*i+1]}')
                print(f'{part.A[2*i][i]}*x{i+1}_hat <= {part.b[2*i]}  {-1*part.A[2*i+1][i]}*x{i+1}_hat >= {-1*part.b[2*i+1]}')