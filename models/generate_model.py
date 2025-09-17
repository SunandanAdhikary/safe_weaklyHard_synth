import numpy as np

def generate_model(A, B, C, K, L, safex, Kmax, mmax, h, i, x0_bounds, xhat0_bounds, filename="suspension.model"):
    """
    Generate Flow* .model file for suspension system with arbitrary locations.
    
    Args:
        A, B, C, K, L : numpy arrays defining system matrices
        safex : 2x4 array with min/max bounds for x1..x4
        Kmax, mmax, h : parameters
        i : number of locations = i+1 (1,10,100,...)
        filename : output file
    """

    n = A.shape[0]
    locations = [10**j for j in range(i)]  # e.g. i=4 -> [1, 10, 100, 1000]
    
    # Calculate length and misscount for each location
    loc_properties = {}
    for loc in locations:
        digits = len(str(loc))
        loc_properties[loc] = {
            'length': digits,  # number of steps to add
            'missct': digits-1,  # number of misses to add
            'h_val': f"h{loc}" if loc > 1 else "h"  # timing parameter name
        }

    # Precompute (A - BK - LC)
    BK = B @ K.reshape(1,-1)
    LC = L @ C.reshape(1,-1)
    Acl = A - BK - LC

    # Build file
    with open(filename, "w") as f:
        f.write("hybrid reachability\n{\n")
        
        # Declare variables
        f.write(" state var " + ", ".join([f"x{j+1}" for j in range(n)]) + ", ")
        f.write(", ".join([f"x{j+1}_hat" for j in range(n)]) + ", dt, gt, length, missct, mdadt, mdadtl1\n\n")

        # Parameters
        f.write(" par\n {\n")
        f.write(f"  Kmax = {Kmax}  mmax = {mmax}  h = {h}\n")
        # Add timing parameters for each location
        timing_params = []
        for loc in locations:
            if loc > 1:
                timing_params.append(f"h{loc} = {h*(len(str(loc)))}")
        f.write("  " + "  ".join(timing_params) + "\n")
        f.write(f"  mdadt1 = 4  mdadt1h = {h*4}\n")
        # Write matrix A coefficients
        for i in range(n):
            for j in range(n):
                f.write(f"  a{i+1}{j+1} = {A[i,j]}  ")
            f.write("\n")
        # Write matrix B coefficients
        for i in range(n):
            f.write(f"  b{i+1}1 = {B[i,0]}  ")
        f.write("\n")
        # Write matrix C coefficients
        for i in range(n):
            f.write(f"  c11 = {C[0,i]}  ")
        f.write("\n")
        # Write Acl coefficients as ad (discrete A) matrix
        BK = B @ K.reshape(1,-1)
        LC = L @ C.reshape(1,-1)
        Acl = A - BK - LC
        for i in range(n):
            for j in range(n):
                f.write(f"  ad{i+1}{j+1} = {Acl[i,j]}  ")
            f.write("\n")
        # Write BD coefficients
        for i in range(n):
            f.write(f"  bd{i+1}1 = {B[i,0]}  ")
        f.write("\n")
        # Write K and L coefficients
        for i in range(n):
            f.write(f"  k{i+1} = {K[i]}  ")
        f.write("\n")
        for i in range(n):
            f.write(f"  l{i+1} = {L[i,0]}  ")
        f.write("\n")
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
        # Add l0 location
        f.write("  l0\n  {\n")
        f.write("   poly ode 1\n   {\n")
        # All derivatives are 0 in l0
        for j in range(n):
            f.write(f"    x{j+1}' = 0\n")
        for j in range(n):
            f.write(f"    x{j+1}_hat' = 0\n")
        f.write("    dt' = 0\n    gt' = 0\n    length' = 0\n    missct' = 0\n")
        f.write("    mdadt' = 0\n    mdadtl1' = 0\n")
        f.write("   }\n   inv\n   {\n")
        f.write("    length = 0\n    missct = 0\n")
        f.write("   }\n  }\n\n")
        
        # Add other locations
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
        
        # Add l1dt location
        f.write("  l1dt\n  {\n")
        f.write("   poly ode 1\n   {\n")
        # x dynamics
        for j in range(n):
            rhs = " + ".join([f"a{j+1}{k+1}*x{k+1}" for k in range(n)])
            rhs += " - (" + " + ".join([f"b{j+1}1*(k{k+1}*x{k+1}_hat)" for k in range(n)]) + ")"
            f.write(f"    x{j+1}' = {rhs}\n")
        # x_hat dynamics
        for j in range(n):
            f.write(f"    x{j+1}_hat' = 0\n")
        f.write("    dt' = 1\n    gt' = 1\n    length' = 0\n    missct' = 0\n")
        f.write("    mdadt' = 0\n    mdadtl1' = 0\n")
        f.write("   }\n   inv\n   {\n")
        f.write("    length <= Kmax\n    missct <= mmax\n")
        f.write("   }\n  }\n")
        
        f.write(" }\n\n")

        # Jumps
        f.write(" jumps\n {\n")
        # Jumps from l0 to other locations
        for loc in locations:
            f.write(f"  l0 -> l{loc}\n")
            f.write("  guard { length = 0  missct = 0  dt = 0 }\n")
            f.write("  reset {\n")
            for j in range(n):
                rhs = f"l{j+1}*(c11*x1 + c12*x2 + c13*x3 + c14*x4) + "
                rhs += " + ".join([f"ad{j+1}{k+1}*x{k+1}_hat" for k in range(n)])
                rhs += f" - bd{j+1}1*(k1*x1_hat + k2*x2_hat + k3*x3_hat + k4*x4_hat)"
                rhs += f" - l{j+1}*(c11*x1_hat + c12*x2_hat + c13*x3_hat + c14*x4_hat)"
                f.write(f"    x{j+1}_hat' := {rhs}\n")
            if loc == '1':
                f.write("    length' := length + 1\n    missct' := missct + 0\n    dt' := 0\n    mdadtl1' := mdadtl1 + 1\n")
            elif loc == '10':
                f.write("    length' := length + 2\n    missct' := missct + 1\n    dt' := 0\n")
            elif loc == '1dt':
                f.write("    length' := length + 1\n    missct' := missct + 0\n    dt' := 0\n")
            f.write("  }\n  interval aggregation\n\n")

        # Jumps between other locations
        all_locs = locations + ['1dt']
        for loc_from in all_locs:
            for loc_to in all_locs:
                # Skip invalid transitions
                if loc_from == '1dt' and loc_to == '1dt':
                    continue
                h_val = h
                if loc_from == '10':
                    h_val = f"{h}*2"
                elif loc_from == '1dt':
                    h_val = f"mdadt1h"

                f.write(f"  l{loc_from} -> l{loc_to}\n")
                f.write("  guard { ")
                if loc_to == '1':
                    if loc_from == '1':
                        f.write("length + 1 <= Kmax  missct <= mmax  dt = h ")
                    elif loc_from == '10':
                        f.write("length + 1 <= Kmax  missct <= mmax  dt = h10 ")
                    elif loc_from == '1dt':
                        f.write("length + 2 <= Kmax  missct + 1 <= mmax  dt = mdadt1h  mdadt = 0 ")
                # Regular locations (l1, l10, l100, etc)
                for loc_to in locations:
                    digits = len(str(loc_to))
                    f.write(f"  l{loc_from} -> l{loc_to}\n")
                    f.write("  guard { ")
                    f.write(f"length + {digits} <= Kmax  missct + {digits-1} <= mmax  ")
                    if loc_from == '1dt':
                        f.write(f"dt = mdadt1h  mdadt = 0")
                    else:
                        h_param = "h" if loc_from == '1' else f"h{loc_from}"
                        f.write(f"dt = {h_param}")
                    f.write(" }\n")
                if loc_to == '1dt':
                    if loc_from == '1':
                        f.write("length + mdadt1 + 1 <= Kmax  missct <= mmax  dt = h  mdadtl1 + 1 <= mdadt1 ")
                    elif loc_from == '10':
                        f.write("length + mdadt1 <= Kmax  missct <= mmax  dt = h10  mdadt + 1 <= mdadt1 ")
                    elif loc_from == '1dt':
                        f.write("length + mdadt <= Kmax  missct <= mmax  dt = h  mdadt >= 1 ")
                f.write("}\n")
                f.write("  reset {\n")
                for j in range(n):
                    rhs = f"l{j+1}*(c11*x1 + c12*x2 + c13*x3 + c14*x4) + "
                    rhs += " + ".join([f"ad{j+1}{k+1}*x{k+1}_hat" for k in range(n)])
                    rhs += f" - bd{j+1}1*(k1*x1_hat + k2*x2_hat + k3*x3_hat + k4*x4_hat)"
                    rhs += f" - l{j+1}*(c11*x1_hat + c12*x2_hat + c13*x3_hat + c14*x4_hat)"
                    f.write(f"    x{j+1}_hat' := {rhs}\n")
                if loc_to == '1':
                    f.write("    length' := length + 1\n    missct' := missct + 0\n    dt' := 0\n    mdadtl1' := mdadtl1 + 1\n")
                elif loc_to == '10':
                    f.write("    length' := length + 2\n    missct' := missct + 1\n    dt' := 0\n")
                elif loc_to == '1dt':
                    if loc_from == '1dt':
                        f.write("    length' := length + 1\n    missct' := missct + 0\n    dt' := 0\n    mdadt' := mdadt - 1\n")
                    else:
                        f.write("    length' := length + 3\n    missct' := missct + 2\n    dt' := 0\n    mdadt' := mdadt1 - 1\n    mdadtl1' := mdadt1\n")
                f.write("  }\n  interval aggregation\n\n")
        f.write(" }\n\n")

        # Init
        f.write(" init\n {\n")
        f.write("  l0\n  {\n")
        for j in range(n):
            f.write(f"   x{j+1} in [{x0_bounds[j,0]}, {x0_bounds[j,1]}]\n")
        for j in range(n):
            f.write(f"   x{j+1}_hat in [{xhat0_bounds[j,0]}, {xhat0_bounds[j,1]}]\n")
        f.write("   dt in [0,0]\n   gt in [0,0]\n   length in [0,0]\n   missct in [0,0]\n")
        f.write("   mdadt in [4,4]\n   mdadtl1 in [0,0]\n")
        f.write("  }\n }\n }\n\n")

        # Unsafe
        f.write(" unsafe\n {\n")
        all_locs = ['0'] + [str(loc) for loc in locations] + ['1dt']
        for loc in all_locs:
            f.write(f"  l{loc}\n  {{\n")
            conditions = []
            for j in range(n):
                conditions.extend([
                    f"    x{j+1} >= {safex[1,j]}",
                    f"    x{j+1} <= {safex[0,j]}"
                ])
            f.write("    " + "  ".join(conditions) + "\n  }}\n")
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

