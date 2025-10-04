import os
import subprocess
from polytope import Polytope
from utils.check_log import check_log
from system_desc import system_desc
from generate_model import generate_model 
from partition_safex import partition_safex
from generate_sscd_model import generate_sscd_model
import numpy as np
from multiprocessing import Pool
from functools import partial

run_multicore = 0 # set to 1 to run all verifications in parallel using GNU parallel
# which_safety: set to 1 to generate sscd models for location wise safety analysis, 2 for overall HA safety analysis, and  0 for no safety analysis only model generation
which_safety = 1

if __name__ == "__main__":
    # -------------------------- System Description -------------------------- #
    system, a, b, c, ad, bd, k, l, h, safex, grid_delta, dims, mdadt1, locationOrCt = system_desc('suspension_control')
    curdir = os.getcwd()
    # -------------------------- fixing parameters -------------------------- #
    Kmax = 8 # hyperperiod
    mmax = 5 # max misses
    if isinstance(locationOrCt, list):
        locations = locationOrCt
        nl = len(locations) # number of locations other than l0 and l1's mdadt variant
        cm_bar = locations[-1].count('0') # number of zeroes in the longest string in the location sequence
        print(f"Initial analysis -> {nl} locations, Max consecutive miss (CM)={cm_bar},...")
    else:
        nl = locationOrCt
        cm_bar = locationOrCt - 1
        locations = [10**j for j in range(nl)]  # e.g. locn=4 -> [1, 10, 100, 1000]
        print(f"Initial analysis -> {nl} locations, Max consecutive miss (CM)={cm_bar},...")


    # nl = 2 # number of locations other than l0 and l1's mdadt variant
    m_bar_max = Kmax - (Kmax // (cm_bar + 1)) if cm_bar > -1 else 0
    print(f"... Max weakly-hard misses (m_bar)={m_bar_max}")

    # ---------------------Initialize the specification list and SG data structure------------------------ #
    spec_list = [{'X': [], 'm_bar': m, 'cm_bar': cm_bar, 'cm': 0, 'K': Kmax} for m in range(m_bar_max + 1)]
    # store polytope objects in sg[m][location_zeroCt] = set of reachable partitions and initialise with empty polytopes
    sg_loc = {m: {i: [] for i in range(nl) for j in range(nl)} for m in range(m_bar_max + 1)} 
    # store polytope objects in sg[m][location_count] = set of reachable partitions and initialise with empty polytopes
    sg = {m: {nl: [] for j in range(nl)} for m in range(m_bar_max + 1)} 
    # initialise sg with empty sets
    # for m in range(m_bar_max + 1):
    #     sg[m][locations] = set()  # start with 0 misses at each location
    # --- loop for all possible miss counts --- #
    tasks = []  # Store all tasks for parallel execution
    # make an nl sized array with -1
    loc_safety_models = np.array([-1] * nl)
    
    # Create system directory if it doesn't exist
    system_dir = os.path.join(curdir, 'models', system)
    os.makedirs(system_dir, exist_ok=True)
    # Convert Windows path to WSL path and go to path, add to tasks
    wsl_path = system_dir.replace('D:', '/mnt/d').replace('\\', '/')
    os.chdir(system_dir) # change to system directory
    # Create log, verified model directories in system directory if they do not exist
    os.makedirs('logs', exist_ok=True)
    os.makedirs('verified_models', exist_ok=True)
    verify_ct = 1 # 2
    
    # -------------------------- Partition Safe Space -------------------------- #
    partitions, count = partition_safex(safex, grid_delta, dims)
    print(f"Partitioned safe space into {count} grid cells using Polytope.")
    # -------------------------- Generate all models --------------------------- #
    for m_j in range(m_bar_max, -1, -1): # from m_bar_max to 0
        p_ct = 0
        # while safex_new is subset of safex
        # safex_new = safex.copy() # copy safex into safex_new such that changing safex_new does not change safex
        while verify_ct : # or (safex_new - safex):
            for partition in partitions:
                modelfilename = f'{system}_p{p_ct}_K{Kmax}_mbar{m_j}' # +.model
                modelfile = os.path.join(system_dir, modelfilename)
                if which_safety == 1:
                    for loc in locations:
                        zct=loc.count('0')
                        loc_safety_models[zct] = generate_sscd_model(a, b, c, ad, bd, k, l, h, safex, partition, modelfile, locn=zct, mdadt1=mdadt1, Kmax=Kmax, mmax=m_j)
                print(f"\n--- Generating model for partition {p_ct} and m_bar = {m_j} misses ---")
                # if m_j > nl, only consider locations with miss count <= m_j
                if nl-1 < m_j:
                    loc_safety_models.append(generate_model(a, b, c, ad, bd, k, l, h, safex, partition, modelfile, locations, mdadt1=4, Kmax=Kmax, mmax=m_j))
                else:
                    locations1 = [loc for loc in locations if m_j >= loc.count('0')]
                    loc_safety_models.append(generate_model(a, b, c, ad, bd, k, l, h, safex, partition, modelfile, locations1, mdadt1=4, Kmax=Kmax, mmax=m_j))
                # tasks.append((wsl_path, modelfilename))
                p_ct = p_ct + 1
    # -------------------------- Model Verification -------------------------- #
                if not run_multicore and which_safety > 0:
                    # if location/sscd wise safety verification wnabled
                    if which_safety == 1 or which_safety == 3:
                        loc_modelfilename = f'{system}_p{p_ct}_K{Kmax}_mbar{m_j}' # +.model
                        for loc_modelfilename in loc_safety_models:
                            loc_id = loc_safety_models.index(loc_modelfilename)
                            print(f"\n--- Verifying model for location/sscd = {loc_modelfilename} and partition {p_ct} ---")
                            loc_modelfile = os.path.join(curdir, f'{system}', loc_modelfilename)    
                            loc_cmdlist =  ['wsl', 'bash', '-c', f'../../flowstar-2.1.0/flowstar < {loc_modelfilename}.model 2>&1 | tee {loc_modelfilename}.log']
                            try:
                                # result = subprocess.run(loc_cmdlist, capture_output=True, text=True, check=True)
                                loc_result = subprocess.Popen(loc_cmdlist, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1)
                            except subprocess.CalledProcessError as e:
                                print(f"Error during verification of {loc_modelfilename}.model: {e}")
                            loc_result.wait()  # Wait for the process to complete  
                            # print(f"Executing command: {' '.join(loc_cmdlist)}")
                            for line in loc_result.stdout:
                                print(line, end='')  # Print each line of output as it is received
                            
                            loc_returncode = loc_result.wait()  # Wait for the process to complete  
                            print(f"Executed command: {' '.join(loc_cmdlist)}")
                            if loc_returncode == 0:
                                print(f"Completed verification for {loc_modelfilename}.model")
                                print(f"Command Output:\n{result.stdout}")
                                print(f"Command Errors (if any):\n{result.stderr}")
                                loc_is_safe = check_log(log_file_path = os.path.join(system_dir, f'{loc_modelfilename}.log'))
                            else:
                                print(f"Verification failed for {loc_modelfilename}.model with return code {loc_result.returncode}")
                                print(f"Command Output:\n{loc_result.stdout}")
                                print(f"Command Errors (if any):\n{loc_result.stderr}")
                                continue  # exit the for loop if verification fails
                            if loc_is_safe:
                                # sg[m_j][len(locations)] = sg[m_j].get(len(locations), set()).union({partition})
                                sg[m_j][(loc_id,loc_id)].append(partition)  # add Polytope object to list
                                print(f"Model {loc_modelfilename} is SAFE, adding partition to sg[{m_j}][{(loc_id,loc_id)}]")
                            else:
                                msg = f"Model {loc_modelfilename} is UNSAFE, not adding partition to sg[{m_j}][{(loc_id,loc_id)}]" if loc_is_safe == 0 else f"Model {loc_modelfilename} verification INCOMPLETE, not adding partition to sg[{m_j}][{(loc_id,loc_id)}]"
                                print(msg)
                    if which_safety >= 2:   
                        print(f"\n--- Verifying model for partition {p_ct} and m_bar = {m_j} misses ---")
                        modelfilename = f'{system}_p{p_ct}_K{Kmax}_mbar{m_j}' # +.model
                        modelfile = os.path.join(curdir, f'{system}', modelfilename)
                        # parentdir = os.path.dirname(modelfile)
                        # cmdlist = ['wsl', 'bash', '-c', f'../../flowstar-2.1.0/flowstar < {modelfilename}.model 2>&1 | tee {modelfilename}.log']
                        cmdlist = ['wsl', 'bash', '-c', f'../../flowstar-2.1.0/flowstar < {modelfilename}.model 2>&1 | tee {modelfilename}.log']
                        print(f"Executing command: {' '.join(cmdlist)}")
                        try:
                            # result = subprocess.run(cmdlist, capture_output=True, text=True, check=True)
                            result = subprocess.Popen(cmdlist, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1)
                        except subprocess.CalledProcessError as e:
                            print(f"Error during verification of {modelfilename}.model: {e}")
                        for line in result.stdout:
                            print(line, end='')  # Print each line of output as it is received
                        result.wait()  # Wait for the process to complete  

                        if result.returncode == 0:
                            print(f"Completed verification for {modelfilename}.model")
                            print(f"Command Output:\n{result.stdout}")
                            print(f"Command Errors (if any):\n{result.stderr}")
                            is_safe = check_log(log_file_path = os.path.join(system_dir, f'{modelfilename}.log'))
                        else:
                            print(f"Verification failed for {modelfilename}.model with return code {result.returncode}")
                            print(f"Command Output:\n{result.stdout}")
                            print(f"Command Errors (if any):\n{result.stderr}")
                            break  # exit the for loop if verification fails
                        if is_safe:
                            # sg[m_j][len(locations)] = sg[m_j].get(len(locations), set()).union({partition})
                            sg[m_j][nl].append(partition)  # add Polytope object to list
                            print(f"Model {modelfilename} is SAFE, adding partition to sg[{m_j}][{nl}]")
                        else:
                            msg = f"Model {modelfilename} is UNSAFE, not adding partition to sg[{m_j}][{nl}]" if is_safe == 0 else f"Model {modelfilename} verification INCOMPLETE, not adding partition to sg[{m_j}][{nl}]"
                            print(msg)

            if run_multicore and which_safety > 0:
                # if location/sscd wise safety verification wnabled
                if which_safety == 1 or which_safety == 3:
                    loc_modelfilename = f'{system}_p{p_ct}_K{Kmax}_mbar{m_j}' # +.model
                    for loc_modelfilename in loc_safety_models:
                        loc_id = loc_safety_models.index(loc_modelfilename)
                        print(f"\n--- Verifying model for location/sscd = {loc_modelfilename} and partition {p_ct} ---")
                        loc_modelfile = os.path.join(curdir, f'{system}', loc_modelfilename)    
                        loc_cmdlist =  ['wsl', 'bash', '-c', f'../../flowstar-2.1.0/flowstar < {loc_modelfilename}.model 2>&1 | tee {loc_modelfilename}.log']
                        try:
                            # result = subprocess.run(loc_cmdlist, capture_output=True, text=True, check=True)
                            loc_result = subprocess.Popen(loc_cmdlist, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1)
                        except subprocess.CalledProcessError as e:
                            print(f"Error during verification of {loc_modelfilename}.model: {e}")
                        loc_result.wait()  # Wait for the process to complete  
                        # print(f"Executing command: {' '.join(loc_cmdlist)}")
                        for line in loc_result.stdout:
                            print(line, end='')  # Print each line of output as it is received
                        
                        loc_returncode = loc_result.wait()  # Wait for the process to complete  
                        print(f"Executed command: {' '.join(loc_cmdlist)}")
                        if loc_returncode == 0:
                            print(f"Completed verification for {loc_modelfilename}.model")
                            print(f"Command Output:\n{result.stdout}")
                            print(f"Command Errors (if any):\n{result.stderr}")
                            loc_is_safe = check_log(log_file_path = os.path.join(system_dir, f'{loc_modelfilename}.log'))
                        else:
                            print(f"Verification failed for {loc_modelfilename}.model with return code {loc_result.returncode}")
                            print(f"Command Output:\n{loc_result.stdout}")
                            print(f"Command Errors (if any):\n{loc_result.stderr}")
                            continue  # exit the for loop if verification fails
                        if loc_is_safe:
                            # sg[m_j][len(locations)] = sg[m_j].get(len(locations), set()).union({partition})
                            sg[m_j][(loc_id,loc_id)].append(partition)  # add Polytope object to list
                            print(f"Model {loc_modelfilename} is SAFE, adding partition to sg[{m_j}][{(loc_id,loc_id)}]")
                        else:
                            msg = f"Model {loc_modelfilename} is UNSAFE, not adding partition to sg[{m_j}][{(loc_id,loc_id)}]" if loc_is_safe == 0 else f"Model {loc_modelfilename} verification INCOMPLETE, not adding partition to sg[{m_j}][{(loc_id,loc_id)}]"
                            print(msg)
                if which_safety >= 2:
                    print(f"\n--- Starting parallel verifications for all partitions under m_bar = {m_j} misses ---")
                    cmdlist = ['wsl', 'bash' , '-c', 'ls *.model | taskset -c "0,1,2,3,4,5,6,7,8,9" parallel -j 9 --bar  "../../flowstar-2.1.0/flowstar < {{}} 2>&1 | tee {{}}.log" && mv *.log logs && mv *.model verified_models']
                    # print(f"Executing command: {' '.join(cmdlist)}")
                    try:
                        result = subprocess.run(cmdlist, capture_output=True, text=True, check=True, universal_newlines=True)
                    except subprocess.CalledProcessError as e:
                        print(f"Error during parallel verification: {e}")
                    if result.returncode == 0:
                        print(f"Completed all parallel verifications for m_bar = {m_j} misses")
                    for partition in partitions:
                        modelfilename = f'{system}_p{p_ct}_K{Kmax}_mbar{m_j}' # +.model
                        modelfile = os.path.join(system_dir, modelfilename)
                        is_safe = check_log(log_file_path = os.path.join(system_dir, f'{modelfilename}.log'))
                        if is_safe:
                            # sg[m_j][len(locations)] = sg[m_j].get(len(locations), set()).union({partition})
                            sg[m_j][len(locations)].append(partition)  # add Polytope object to list
                            print(f"Model {modelfilename} is SAFE, adding partition to sg[{m_j}][{len(locations)}]")
                        else:
                            msg = f"Model {modelfilename} is UNSAFE, not adding partition to sg[{m_j}][{len(locations)}]" if is_safe == 0 else f"Model {modelfilename} verification INCOMPLETE, not adding partition to sg[{m_j}][{len(locations)}]"
                            print(msg)

            partitions = list(sg[m_j][nl]) # update partitions to only those that are safe
            print(f"Updated safe partitions for m_bar={m_j}: {len(partitions)} with {sg[m_j][nl]}")
            verify_ct = verify_ct - 1

            result.wait()  # Wait for the process to complete  
            print(f"Completed all verifications for m_bar = {m_j} misses.")
            print(f"Moving all log files to logs directory and models to verified_models directory...")
            result = subprocess.run(['wsl', 'bash', '-c', 'mv *.log logs'], check=True)
            result = subprocess.run(['wsl', 'bash', '-c', 'mv *.model verified_models'], check=True)


    spec_list = [{'X': Polytope.union(sg[0][nl]), 'm_bar': m_j, 'cm_bar': cm_bar, 'cm': 0, 'K': Kmax} for m_j in range(m_bar_max + 1)]
    print(f"\nFinal specification list:")
    for spec in spec_list:
        print(f"  - {spec}")
