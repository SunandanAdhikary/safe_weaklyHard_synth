import numpy as np
import os

def generate_model(A, B, C, Ad, Bd, K, L, h, safex, x0_bounds, filename, mdadt1=4, locn=4, Kmax=20, mmax=10):
    """
    Generate Flow* .model file for suspension system with arbitrary locations.

    """
    
    fixed_step = 0.00005
    rem = 1e-3  # remainder estimation
    cutoff = 1e-8  # cutoff for polynomial terms
    prec = 53  # precision
    orders = [3, 5]  # fixed orders for Taylor model
    n = A.shape[0]  # number of states
    p = K.shape[0]  # number of inputs
    m = C.shape[0]  # number of outputs
    # ----------------define locations and their properties: length, miss count, sampling period---------------- ##
    if not isinstance(locn, int):
        locations = locn
    else:
        locations = [10**j for j in range(locn)]  # e.g. locn=4 -> [1, 10, 100, 1000]
    all_locs = ['0'] + [str(loc) for loc in locations] + ['1'*mdadt1] + ['end'] # adding start, end, mdadt locations
    # Calculate length and misscount for each location
    loc_properties = {}
    for loc in all_locs:
        if loc == '0':
            loc_properties[loc] = {
                'length': 0,
                'missct': 0,
                'h_var': 'h0',  # no timing parameter
                'h_val': 0  # timing parameter value
            }
        elif loc == 'end':
            loc_properties[loc] = {
                'length': Kmax,
                'missct': mmax,
                'h_var': 'h0',  # no timing parameter
                'h_val': 0  # timing parameter value
            }
        elif loc == '1'*mdadt1:
            loc_properties[loc] = {
                'length': mdadt1,  
                'missct': 0,  # no misses to add
                'h_var': f"h",  # same timing parameter as 1 for MDADT 1
                'h_val': h  # timing parameter value
            }
        else:
            digits = len(loc)
            loc_properties[loc] = {
                'length': digits,  # number of steps to add
                'missct': digits-1,  # number of misses to add
                'h_var': f"h" if loc == '1' else f"h{loc}",  # timing parameter name
                'h_val': h if loc == '1' else h*digits  # timing parameter value
        }

    ## ---------Precompute (A - BK - LC), (Ad - BdK - LC) for dynamics---------- ##
    BK = B @ K.reshape(1,-1)
    LC = L @ C.reshape(1,-1)
    Acl = A - BK - LC

    BdK = Bd @ K.reshape(1,-1)
    Adcl = Ad - BdK - LC
    
    #### -----------Build file-------------- ####
    modelfile = f"{filename}.model"
    with open(modelfile, "w") as f:
        f.write("hybrid reachability\n{\n")
        
        # -------------Declare variables------------- #
        f.write(" state var " + ", ".join([f"x{j+1}" for j in range(n)]) + ", ")
        f.write(", ".join([f"x{j+1}_hat" for j in range(n)]) + ", dt, gt, length, missct, mdadt\n\n")

        # -------------Parameters------------- #
        f.write(" par\n {\n")
        
        # Write matrix A coefficients
        # for i in range(n):
        #     for j in range(n):
        #         f.write(f"  a{i+1}{j+1} = {A[i,j]}  ")
        #     # f.write("\n")
        # # Write matrix B coefficients
        # for i in range(n):
        #     for j in range(p):
        #         f.write(f"  b{i+1}{j+1} = {B[i,j]}  ")
        # # f.write("\n")
        # # Write matrix C coefficients
        # for i in range(m):
        #     for j in range(n):
        #         f.write(f"  c{i+1}{j+1} = {C[i,j]}  ")
        # # f.write("\n")
        # # Write Ad coefficients as ad (discrete A) matrix
        # for i in range(n):
        #     for j in range(n):
        #         f.write(f"  ad{i+1}{j+1} = {Acl[i,j]}  ")
        # # f.write("\n")
        # # Write Bd coefficients
        # for i in range(n):
        #     for j in range(p):
        #         f.write(f"  bd{i+1}{j+1} = {B[i,j]}  ")
        # # f.write("\n")
        # # Write K and L coefficients
        # for i in range(p):
        #     for j in range(n):
        #         f.write(f"  k{i+1}{j+1} = {K[i,j]}  ")
        # # f.write("\n")
        # for i in range(n):
        #     for j in range(m):
        #         f.write(f"  l{i+1}{j+1} = {L[i,j]}  ")
        # f.write("\n")
        
        f.write(f"  Kmax = {Kmax}   mmax = {mmax}   mdadt1 = {mdadt1}\n")
                
        # create timing parameters for each location
        timing_params = []
        for loc in all_locs:
            if loc > '0' and loc != '1'*mdadt1 and loc != 'end':  # skip l0, l1's mdadt variant
                timing_params.append(f"{loc_properties[loc]['h_var']} = {loc_properties[loc]['h_val']}")
        f.write("  " + "  ".join(timing_params) + "\n")
        f.write(" }\n\n")

        # ----------------Settings---------------- #
        f.write(" setting\n {\n")
        f.write(f"  fixed steps {str(fixed_step)}\n")
        f.write(f"  time {str(Kmax*h)}\n")
        f.write(f"  remainder estimation {str(rem)}\n")
        f.write("  identity precondition\n")
        f.write("  matlab octagon gt , x1\n")
        if len(orders) == 2:
            f.write(f"  adaptive orders {{ min {str(orders[0])} , max {str(orders[1])} }}\n")
        if len(orders) == 1:
            f.write(f"  fixed orders {str(orders)}\n")
        f.write(f"  cutoff {str(cutoff)}\n")
        f.write(f"  precision {str(prec)}\n")
        f.write(f"  output {os.path.basename(filename)}_op\n")
        f.write(f"  max jumps {Kmax}\n")
        f.write("  print on\n")
        f.write(" }\n\n")

        # ----------------locations---------------- #
        f.write(" modes\n {\n")
        
        for loc in all_locs:
            f.write(f"  l{loc}\n  {{\n")
             # --------------------- flow --------------------- #
            f.write("   poly ode 1\n   {\n")
            # x dynamics
            if loc > '0' and loc != 'end':  # not l0 or lend
                for j in range(n):
                    rhs = " + ".join([f"{A[j,k]}*x{k+1}" for k in range(n)])
                    rhs += " - (" + " + ".join([f"{BK[j,k]}*x{k+1}_hat" for k in range(n)]) + ")"
                    # for j1 in range(p):
                    #     rhs += " - (" + " + ".join([f"{B[j,j1]*K[j1,k]}*x{k+1}_hat" for k in range(n)]) + ")"
                    f.write(f"    x{j+1}' = {rhs}\n")
                # x_hat dynamics
                for j in range(n):
                    #rhs = " + ".join([f"{Acl[j,k]}*x{k+1}_hat" for k in range(n)])
                    #rhs += " + " + " + ".join([f"{LC[j,0]}*x1"])
                    rhs = 0
                    f.write(f"    x{j+1}_hat' = {rhs}\n")
                f.write(f"    dt' = 1\n    gt' = 1\n")
                f.write("    length' = 0\n    missct' = 0\n    mdadt' = 0\n")
                f.write("   }\n")
            else:
                # All derivatives are 0 in l0
                for j in range(n):
                    f.write(f"    x{j+1}' = 0\n")
                for j in range(n):
                    f.write(f"    x{j+1}_hat' = 0\n")
                f.write("    dt' = 1\n    gt' = 1\n    length' = 0\n    missct' = 0\n    mdadt' = 0\n")
                f.write("   }\n")
            
            # ------------------- invariants --------------------- #
            if loc == '0':# start a l0
                f.write("   inv\n   {\n")
                f.write("    length = 0\n    missct = 0\n     gt = 0\n")
                f.write("   }\n  }\n\n")
            elif loc == 'end':# end at lend
                f.write("   inv\n   {\n")
                f.write(f"    length = {Kmax}\n    missct = {mmax}\n   gt = {Kmax*h}\n")
                f.write("   }\n  }\n\n")
            else:
                f.write("   inv\n   {\n")
                f.write(f"    length <= {Kmax}\n    missct <= {mmax}\n")
                f.write("   }\n  }\n\n")
        
        f.write(" }\n\n")
        
        # -------------Jumps-------------- #
        f.write(" jumps\n {\n")
        for i_to, loc_to in enumerate(all_locs):
            for i_from, loc_from in enumerate(all_locs):
                # ------------------guards--------------------- #
                if loc_from != 'end' and loc_to != '0' and not (loc_from == '1'*mdadt1 and loc_to == '1'*mdadt1): # no self loop at l0,lend,l1mdadt, no jump to l0, no jump from lend
                    f.write(f"  l{loc_from} -> l{loc_to}\n")
                    f.write("  guard { ")
                    if loc_from == '0':
                        # if loc_to != 'end':
                        f.write(f"length = 0   missct = 0   dt = 0   gt = 0")   # start from l0, no self loop at l0
                    elif loc_to == 'end':
                        f.write(f"length = {Kmax}   missct = {mmax}   dt = {loc_properties[loc_from]['h_val']}   gt = {Kmax*h}")   # end at l0
                    else:
                        f.write(f"length + {loc_properties[loc_to]['length']} <= {Kmax-1}   missct + {loc_properties[loc_to]['missct']} <= {mmax}   dt = {loc_properties[loc_from]['h_val']}   gt <= {Kmax*h-h}")
                        if loc_to == '1'*mdadt1:
                            f.write(f"  mdadt = {mdadt1}")  # jump to l1's madadt variant only if mdadt is not yet maintained
                        if loc_from == '1'*mdadt1:
                            if loc_to == '1'*mdadt1:
                                f.write(f"  mdadt <= {mdadt1-1}  mdadt >= 2")   # self loop in l1's madadt variant until mdadt is 1
                            elif loc_to == '0':
                                f.write(f"  mdadt = 0")     #  jump to l0 when mdadt is 0 since there is no 1 in next location
                            else:
                                f.write(f"  mdadt = 1")     # jump to anywhere other than l0 when mdadt is 1 since there is another 1 in next location
                    
                    f.write("}\n")
                    # ------------------resets--------------------- #
                    f.write("  reset {\n")
                    if loc_to > '0':
                        for j in range(n):
                            rhs = " + ".join([f"{Adcl[j,k]}*x{k+1}_hat" for k in range(n)])
                            rhs += " + " + " + ".join([f"{LC[j,k]}*x{k+1}" for k in range(n)])
                            f.write(f"    x{j+1}_hat' := {rhs}\n")
                        f.write(f"    length' := length + {loc_properties[loc_to]['length']}\n    missct' := missct + {loc_properties[loc_to]['missct']}\n")
                    if loc_to == '1'*mdadt1:
                        f.write(f"    mdadt' := mdadt - 1\n")
                    f.write("  }\n  interval aggregation\n\n")
        f.write(" }\n\n")

        # -------------Init------------- #
        f.write(" init\n {\n")
        f.write("  l0\n  {\n")
        for j in range(n):
            f.write(f'   x{j+1} in [{-1*x0_bounds.b[2*j+1]/-1*x0_bounds.A[2*j+1][j]} ,{x0_bounds.b[2*j]/x0_bounds.A[2*j][j]}] \n')
            f.write(f'   x{j+1}_hat in [{-1*x0_bounds.b[2*j+1]/-1*x0_bounds.A[2*j+1][j]} ,{x0_bounds.b[2*j]/x0_bounds.A[2*j][j]}] \n')

            # f.write(f'   {x0_bounds.A[2*j][j]}*x{j+1} <= {x0_bounds.b[2*j]}  {-1*x0_bounds.A[2*j+1][j]}*x{j+1} >= {-1*x0_bounds.b[2*j+1]}\n')
            # f.write(f"   {x0_bounds.A[2*j][j]}*x{j+1}_hat <= {x0_bounds.b[2*j]}  {-1*x0_bounds.A[2*j+1][j]}*x{j+1}_hat >= {-1*x0_bounds.b[2*j+1]}\n")

        f.write("   dt in [0,0]\n   gt in [0,0]\n   length in [0,0]\n   missct in [0,0]\n")
        f.write(f"   mdadt in [{str(mdadt1)},{str(mdadt1)}]\n")
        f.write("  }\n")
        f.write(" }\n")
        f.write("}\n\n")

        # -------------Unsafe------------- #
        f.write(" unsafe\n {\n")
        # all_locs = ['0'] + [str(loc) for loc in locations] + ['1dt']
        for loc in all_locs:
            f.write(f"  l{loc}\n  {{\n")
            conditions = []
            for j in range(n):
                conditions.extend([
                    f"    x{j+1} >= {safex[1,j]}",
                    f"    x{j+1} <= {safex[0,j]}\n"
                ])
            f.write("    " + "  ".join(conditions) + "\n  }\n")
        f.write(" }\n")

    print(f".model file written to {modelfile}")
    return modelfile