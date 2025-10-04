import os
import subprocess
from polytope import Polytope
from system_desc import system_desc
from generate_model import generate_model 
from partition_safex import partition_safex
from generate_sscd_model import generate_sscd_model
from verify_model import verify_model
from verify_model_parallel import verify_model_parallel
import numpy as np
from multiprocessing import Pool
from utils.config import load_input_cfg

# --------------------- Read input config and set params--------------------- #
curdir = os.getcwd()
cfg = load_input_cfg(curdir)
fixed_step = cfg['fixed_step']
rem = cfg['rem']
cutoff = cfg['cutoff']
prec = cfg['prec']
orders = cfg['orders']
# run_multicore is set to 1 to run all verifications in parallel using GNU parallel
run_multicore = cfg['run_multicore'] 
# which_safety: set to 1 for location/sscd wise safety analysis,
#               set 2 for overall HA safety analysis (followed by sscd wise analysis), and  
#               set 0 for no safety analysis, only model generation
which_safety = cfg['which_safety']
Kmax = cfg['hyperperiod']
rootdir = cfg['system_dir']
# mmax = 5 # max misses
# parse and store variables from input_cfg.json to input in generate_*()

if __name__ == "__main__":
    # -------------------------- System Description -------------------------- #
    system, a, b, c, ad, bd, k, l, h, safex, grid_delta, dims, mdadt1, locationOrCt = system_desc('suspension_control')
    if isinstance(locationOrCt, list):
        locations = locationOrCt
        nl = len(locations) # number of locations other than l0 and l1's mdadt variant
        cm_bar = locations[-1].count('0') # number of zeroes in the longest string in the location sequence
        print(f"Initial analysis -> {nl} locations, Max consecutive miss (CM)={cm_bar},...")
    else:
        nl = locationOrCt
        cm_bar = locationOrCt - 1
        locations = [10**j for j in range(nl)]  # e.g. locn=4 -> [1, 10, 100, 1000]
        print(f"----Initial analysis -> {nl} locations, Max consecutive miss (CM)={cm_bar},...")


    # nl = 2 # number of locations other than l0 and l1's mdadt variant
    m_bar_max = Kmax - (Kmax // (cm_bar + 1)) if cm_bar > -1 else 0
    print(f"... Max weakly-hard misses (m_bar)={m_bar_max}----")

    # ---------------------Initialize the specification list and SG data structure------------------------ #
    spec_list = [{'X': [], 'm_bar': m, 'cm_bar': cm_bar, 'cm': mdadt1, 'K': Kmax} for m in range(m_bar_max + 1)]
    # store polytope objects in sg[m][location_zeroCt] = set of reachable partitions and initialise with empty polytopes
    sg_loc = {m: {i: [] for i in range(nl)} for m in range(m_bar_max + 1)} 
    # store polytope objects in sg[m][location_count] = set of reachable partitions and initialise with empty polytopes
    sg = {m: {nl: [] for j in range(nl)} for m in range(m_bar_max + 1)} 
    
    # tasks = []  # Store all tasks for parallel execution
    # make an nl sized array with -1 to store sscd models
    # loc_safety_models = [-1] * nl
    
    # ----------Create system directory if it doesn't exist----------- #
    if rootdir == '':
        rootdir = os.path.join(rootdir, 'models')
        os.makedirs(rootdir, exist_ok=True)
    system_dir = os.path.join(rootdir, system)
    os.makedirs(system_dir, exist_ok=True)

    # for inductive safety analysis
    verify_ct = 2
    
    # -------------------------- Partition Safe Space -------------------------- #
    partitions, count = partition_safex(safex, grid_delta, dims)
    print(f"----Partitioned safe space into {count} grid cells using Polytope----")
    # -------------------------- Generate all models --------------------------- #
    for m_j in range(m_bar_max, -1, -1): # from m_bar_max to 0
        p_ct = 0
        modelfilepaths = []
        loc_safety_models = ['' for i in range(nl)]
        cm_bar_j = 0
        # while safex_new is subset of safex
        # safex_new = safex.copy() # copy safex into safex_new such that changing safex_new does not change safex
        while verify_ct : # or (safex_new - safex):
            for partition in partitions:
                modelfilename = f'{system}_p{p_ct}_K{Kmax}_mbar{m_j}' # +.model or +_{sscd}.model
                modelfilepath = os.path.join(system_dir, modelfilename)
                # if user needs verification/reachability analysis
                if which_safety >= 0:
                    for loc in locations:
                        zct = str(loc).count('0')
                        loc_modelfilepath = generate_sscd_model(a, b, c, ad, bd, k, l, h, safex, partition, modelfilepath, mdadt1, zct, Kmax, m_j, fixed_step=fixed_step, rem=rem, cutoff=cutoff, prec=prec, orders=orders)
                        loc_safety_models[zct] = loc_modelfilepath
                        # ------------------------------ SSCD Verification ---------------------------- #
                        if not run_multicore and which_safety == 1 or which_safety == 3:
                            print(f"\n--- Verifying HA model for {p_ct}-th partition, {loc} sscd ---")
                            loc_results, loc_is_safe_status =  verify_model([loc_modelfilepath])
                            if loc_is_safe_status[0] == 1:
                                sg_loc[m_j][zct].append(partition)  # add Polytope object to list
                                print(f"Model {loc_modelfilepath} is SAFE, adding partition to sg_loc[{m_j}][{zct}]")
                                if loc != 1:
                                    cm_bar_j = cm_bar_j + 1
                            else:
                                print(f"Model {loc_modelfilepath} is UNSAFE, not adding partition to sg_loc[{m_j}][{zct}]" if loc_is_safe_status[0] == 0 else f"Model {loc_modelfilepath} verification INCOMPLETE, try changing config params, not adding partition to sg[{m_j}][{zct}]" if loc_is_safe_status[0] == -1 else "logging error, retry")
                                # limit number of consecutive misses
                                locations = locations[:locations.index(loc)]
                                break
                    if run_multicore and which_safety == 1 or which_safety == 3:
                        loc_results, loc_is_safe_status =  verify_model_parallel(loc_safety_models)
                        for loc in locations:
                            zct = str(loc).count('0')
                            loc_modelfilepath = loc_safety_models[zct]
                            if loc_is_safe_status[0] == 1:
                                sg_loc[m_j][zct].append(partition)  # add Polytope object to list
                                print(f"Model {loc_modelfilepath} is SAFE, adding partition to sg_loc[{m_j}][{zct}]")
                                if loc != 1:
                                    cm_bar_j = cm_bar_j + 1
                            else:
                                print(f"Model {loc_modelfilepath} is UNSAFE, not adding partition to sg_loc[{m_j}][{zct}]" if loc_is_safe_status[0] == 0 else f"Model {loc_modelfilepath} verification INCOMPLETE, try changing config params, not adding partition to sg[{m_j}][{zct}]" if loc_is_safe_status[0] == -1 else "logging error, retry")
                                # limit number of consecutive misses
                                locations = locations[:locations.index(loc)]
                                break
                # if which_safety >= 0:
                    if (which_safety == 3 and partition in sg_loc[m_j]) or which_safety == 2:
                        print(f"\n--- Generating HA model for partition {p_ct} and m_bar = {m_j} misses and sscds = {locations} ---")
                        # if m_j > nl, only consider locations with miss count <= m_j
                        if cm_bar_j < m_j:
                        # if nl-1 < m_j:
                            # loc_safety_models.append(generate_model(a, b, c, ad, bd, k, l, h, safex, partition, modelfile, locations, mdadt1=4, Kmax=Kmax, mmax=m_j))
                            modelfile = generate_model(a, b, c, ad, bd, k, l, h, safex, partition, modelfilepath, locations, mdadt1, Kmax, m_j, fixed_step=fixed_step, rem=rem, cutoff=cutoff, prec=prec, orders=orders)
                            modelfilepaths.append(modelfile)
                        else:
                            locations1 = [loc for loc in locations if m_j >= loc.count('0')]
                            # loc_safety_models.append(generate_model(a, b, c, ad, bd, k, l, h, safex, partition, modelfile, locations1, mdadt1=4, Kmax=Kmax, mmax=m_j))
                            modelfile = generate_model(a, b, c, ad, bd, k, l, h, safex, partition, modelfilepath, locations1, mdadt1, Kmax, m_j, fixed_step=fixed_step, rem=rem, cutoff=cutoff, prec=prec, orders=orders)
                            modelfilepaths.append(modelfile)
                        # tasks.append((wsl_path, modelfilename))
                        # -------------------------- HA Model Verification -------------------------- #
                        if not run_multicore and which_safety >= 2:
                            print(f"\n--- Verifying HA model for {p_ct}-th partition ---")
                            results, is_safe_status =  verify_model([modelfile])
                            if is_safe_status[0] == 1:
                                sg[m_j][nl].append(partition)  # add Polytope object to list
                                print(f"{modelfile} is SAFE, adding partition to sg[{m_j}][{nl}]")
                            else:
                                print(f"{modelfile} is UNSAFE, not adding partition to sg[{m_j}][{nl}]" if is_safe_status[0] == 0 else f"{modelfile} verification INCOMPLETE, try changing config params, not adding partition to sg[{m_j}][{nl}]" if is_safe_status[0] == -1 else "logging error, retry")
                p_ct = p_ct + 1
            if run_multicore:
                results, is_safe_status =  verify_model_parallel(modelfilepaths)
                for modelfile in modelfilepaths:
                    # idx = modelfilenames.index(modelfilename)
                    # modelfilename = safety_models[idx]
                    if is_safe_status[0] == 1:
                        sg_loc[m_j][cm_bar_j].append(partition)  # add Polytope object to list
                        print(f"Model {modelfile} is SAFE, adding partition to sg_loc[{m_j}][{cm_bar_j}]")
                    else:
                        print(f"Model {modelfile} is UNSAFE, not adding partition to sg_loc[{m_j}][{cm_bar_j}]" if is_safe_status[0] == 0 else f"Model {modelfile} verification INCOMPLETE, try changing config params, not adding partition to sg[{m_j}][{cm_bar_j}]" if is_safe_status[0] == -1 else "logging error, retry")

            # update partitions for second time reachability analysis
            partitions = list(sg[m_j][cm_bar_j]) # update partitions to only those that are safe
            print(f"------Updated safe partitions for m_bar={m_j}: {len(partitions)} with {sg[m_j][cm_bar_j]}------")
            verify_ct = verify_ct - 1

        # update spec list for this miss count
        spec_list.append({'X': sg[m_j][cm_bar_j], 'm_bar': m_j, 'cm_bar': cm_bar_j, 'cm': 0, 'K': Kmax})


    # spec_list = [{'X': sg[0][nl], 'm_bar': m_j, 'cm_bar': cm_bar, 'cm': 0, 'K': Kmax} for m_j in range(m_bar_max + 1)]

    print(f"-----------Final specification list---------------\n")
    for spec in spec_list:
        if spec.X != []:
            print(f"spec for {spec.m_bar}  - {spec}")
        else:
            print(f"no spec for {spec.m_bar}")
