import numpy as np

def generate_model(A, B, C, K, L, safex, Kmax, mmax, h, i, x0_bounds, xhat0_bounds, filename="suspension.model"):
    """
    Generate Flow* .model file for suspension system with arbitrary locations.
    
    Args:
        A, B, C, K, L : numpy arrays defining system matrices
        safex : 2x4 array with min/max bounds for x1..x4
        Kmax, mmax, h : parameters
        i : number of locations = i+1 (1,10,...)
        filename : output file
    """

    n = A.shape[0]
    locations = [10**j for j in range(i)]  # e.g. i=4 -> [1, 10, 100, 1000]

    # Precompute (A - BK - LC)
    BK = B @ K.reshape(1,-1)
    LC = L @ C.reshape(1,-1)
    Acl = A - BK - LC

    # Build file
    with open(filename, "w") as f:
        f.write("hybrid reachability\n{\n")
        
        # Declare variables
        f.write(" state var " + ", ".join([f"x{j+1}" for j in range(n)]) + ", ")
        f.write(", ".join([f"x{j+1}_hat" for j in range(n)]) + ", dt, gt, length, missct\n\n")

        # Parameters
        f.write(" par\n {\n")
        f.write(f"  Kmax = {Kmax}\n")
        f.write(f"  mmax = {mmax}\n")
        f.write(f"  h = {h}\n")
        f.write(" }\n\n")

        # Settings
        f.write(" setting\n {\n")
        f.write("  fixed steps 0.001\n")
        f.write("  time 0.2\n")
        f.write("  remainder estimation 1e-3\n")
        f.write("  identity precondition\n")
        f.write("  gnuplot octagon gt , x1\n")
        f.write("  fixed orders 12\n")
        f.write("  cutoff 1e-8\n")
        f.write("  precision 53\n")
        f.write("  output suspension_controller\n")
        f.write(f"  max jumps {Kmax}\n")
        f.write("  print on\n")
        f.write(" }\n\n")

        # Modes
        f.write(" modes\n {\n")
        for loc in locations:
            f.write(f"  l{loc}\n  {{\n")
            f.write("   poly ode 1\n   {\n")
            # x dynamics
            for j in range(n):
                rhs = " + ".join([f"{A[j,k]}*x{k+1}" for k in range(n)])
                rhs += " - (" + " + ".join([f"{B[j,0]*K[k]}*x{k+1}_hat" for k in range(n)]) + ")"
                f.write(f"    x{j+1}' = {rhs}\n")
            # x_hat dynamics
            for j in range(n):
                #rhs = " + ".join([f"{Acl[j,k]}*x{k+1}_hat" for k in range(n)])
                #rhs += " + " + " + ".join([f"{LC[j,0]}*x1"])
                rhs = 0
                f.write(f"    x{j+1}_hat' = {rhs}\n")
            f.write(f"    dt' = {h}\n    gt' = 1\n    length' = 0\n    missct' = 0\n")
            f.write("   }\n   inv\n   {\n")
            f.write("    length <= Kmax\n    missct <= mmax\n")
            #for j in range(n):
                #f.write(f"    x{j+1} <= {safex[1,j]}\n")
                #f.write(f"    x{j+1} >= {safex[0,j]}\n")
            f.write("   }\n  }\n\n")
        f.write(" }\n\n")

        # Jumps
        f.write(" jumps\n {\n")
        for i_from, loc_from in enumerate(locations):
            for i_to, loc_to in enumerate(locations):
                f.write(f"  l{loc_from} -> l{loc_to}\n")
                f.write("  guard { ")
                digits = len(str(loc_to))
                f.write(f"length + {digits} <= Kmax   missct + {digits-1} <= mmax   dt = {(i_from+1)*h} ")
                f.write("}\n")
                f.write("  reset {\n")
                for j in range(n):
                    rhs = " + ".join([f"{Acl[j,k]}*x{k+1}_hat" for k in range(n)])
                    rhs += " + " + " + ".join([f"{LC[j,k]}*x{k+1}" for k in range(n)])
                    f.write(f"    x{j+1}_hat' := {rhs}\n")
                f.write(f"    length' := length + {digits}\n    missct' := missct + {digits-1}\n    dt' := 0\n")
                f.write("  }\n  interval aggregation\n\n")
        f.write(" }\n\n")

        # Init
        f.write(" init\n {\n")
        f.write("  l1\n  {\n")
        for j in range(n):
            #f.write(f"   x{j+1} in [-0.1,0.1]\n")
            f.write(f"   x{j+1} in [{x0_bounds[j,0]}, {x0_bounds[j,1]}]\n")
        for j in range(n):
            #f.write(f"   x{j+1}_hat in [-0.1,0.1]\n")
            f.write(f"   x{j+1}_hat in [{xhat0_bounds[j,0]}, {xhat0_bounds[j,1]}]\n")
        f.write("   dt in [0,0]\n   gt in [0,0]\n   length in [0,0]\n   missct in [0,0]\n")
        f.write("  }\n }\n }\n\n")

        # Unsafe
        f.write(" unsafe\n {\n")
        for loc in locations:
            for j in range(n):
                f.write(f"  l{loc} {{ x{j+1} >= {safex[1,j]} }}\n")
                f.write(f"  l{loc} {{ x{j+1} <= {safex[0,j]} }}\n")
        f.write(" }\n")

    print(f".model file written to {filename}")


# --------------------------
# Main script entry point
# --------------------------
if __name__ == "__main__":
    # Example matrices
    A = np.array([[0.995860265603438, 0.0378696105301410, 0.00212669277812880, 0.00160492825553481],
                  [-0.174562623798343,	0.908578953704267,	0.0461683633555589,	0.0573094395666044],
                  [0.0311138708080227, 0.0160492825553481, 0.935759216765522, 0.0151544413769853],
                  [1.08396104971604, 0.573094395666044, -2.29631635987487, 0.0906898643677941]])
    B = np.array([[0.0341116067291290],[1.27458305177487],[0.336215739966162],[-16.9738474024853]])
    C = np.array([[1,0,0,0]])
    K = np.array([3.69840485205769, 0.527354601469353, -0.0569469847116374, 0.0311945072320842])  # example
    L = np.array([[0.627216669816836],[0.0776672166534858],[0.0335031868276200],[0.804439585315081]])   # example

    safex = np.array([[-2, -200, -100, -600],
                      [ 2, 200, 100, 600]])

    # Initial ranges for x and x_hat
    x0_bounds = np.array([[-1, 1],
                          [-100, 100],
                          [-50, 50],
                          [-300, 300]])
    xhat0_bounds = np.array([[-1, 1],
                          [-100, 100],
                          [-50, 50],
                          [-300, 300]])

    generate_model(A, B, C, K, L, safex, Kmax=20, mmax=5, h=0.04, i=4, x0_bounds=x0_bounds, xhat0_bounds=xhat0_bounds, filename="suspension.model")

