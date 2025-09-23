import os
import subprocess
from polytope import Polytope
from check_log import check_log
from system_desc import system_desc
from generate_model import generate_model 
from partition_safex import partition_safex
import numpy as np
from multiprocessing import Pool
from functools import partial

run_multicore = 0 # set to 1 to run all verifications in parallel using GNU parallel

if __name__ == "__main__":
    # -------------------------- System Description -------------------------- #
    system, a, b, c, ad, bd, k, l, h, safex, grid_delta, dims, mdadt1, locations = system_desc('suspension_control')
    curdir = os.getcwd()
    # -------------------------- fixing parameters -------------------------- #
    Kmax = 8 # hyperperiod
    mmax = 5 # max misses
    if isinstance(locations, list):
        nl = len(locations) # number of locations other than l0 and l1's mdadt variant
        cm_bar = nl - 1
        m_bar_max = Kmax - (Kmax // (cm_bar + 1)) if cm_bar > -1 else 0
        print(f"Initial analysis -> {nl} locations, Max consecutive miss (CM)={cm_bar},...")

    else:
        nl = locations
        cm_bar = len(locations) - 1
        
        print(f"Initial analysis -> {len(nl)} locations, Max consecutive miss (CM)={cm_bar},...")

    # nl = 2 # number of locations other than l0 and l1's mdadt variant
    m_bar_max = Kmax - (Kmax // (cm_bar + 1)) if cm_bar > -1 else 0
    print(f"... Max weakly-hard misses (m_bar)={m_bar_max}")

    # ---------------------Initialize the specification list and SG data structure------------------------ #
    spec_list = [{'X': [], 'm_bar': m, 'cm_bar': cm_bar, 'cm': 0, 'K': Kmax} for m in range(m_bar_max + 1)]
    # store polytope objects in sg[m][locations] = set of reachable partitions and initialise with empty polytopes
    sg = {m: {len(locations): [] for m in range(m_bar_max + 1)} for m in range(m_bar_max + 1)}  # sg[m][locations] = list of Polytope objects
    # # initialise sg with empty sets
    # for m in range(m_bar_max + 1):
    #     sg[m][locations] = set()  # start with 0 misses at each location
    # --- loop for all possible miss counts --- #
    tasks = []  # Store all tasks for parallel execution
    
    # Create system directory if it doesn't exist
    system_dir = os.path.join(curdir, 'models', system)
    os.makedirs(system_dir, exist_ok=True)
    # Convert Windows path to WSL path and go to path, add to tasks
    wsl_path = system_dir.replace('D:', '/mnt/d').replace('\\', '/')
    os.chdir(system_dir) # change to system directory
    # Create log, verified model directories in system directory if they do not exist
    os.makedirs('logs', exist_ok=True)
    os.makedirs('verified_models', exist_ok=True)
    verify_ct = 2
    
    # -------------------------- Partition Safe Space -------------------------- #
    partitions, count = partition_safex(safex, grid_delta, dims)
    print(f"Partitioned safe space into {str(count)} grid cells using Polytope.")
    # --- Generate all models --- #
    for m_j in range(m_bar_max + 1):
        p_ct = count
        # while safex_new is subset of safex
        # safex_new = safex.copy() # copy safex into safex_new such that changing safex_new does not change safex
        while verify_ct : # or (safex_new - safex):
            for partition in partitions:    
                modelfilename = f'{system}_p{p_ct}_K{Kmax}_mbar{m_j}' # +.model
                modelfile = os.path.join(system_dir, modelfilename)
                # parentdir = os.path.dirname(modelfile)
                print(f"\n--- Generating model for partition {p_ct} and m_bar = {m_j} misses ---")
                generate_model(a, b, c, ad, bd, k, l, h, safex, partition, modelfile,
                            mdadt1=4, locn=nl, Kmax=Kmax, mmax=m_j)
                # tasks.append((wsl_path, modelfilename))
                p_ct = p_ct - 1
        # # Run all verifications in parallel
        # print("\n--- Starting parallel verifications ---")
        # with Pool() as pool:
        #     # Execute all tasks in parallel
        #     results = pool.starmap(run_flowstar, tasks)
            
        # # Check results
        # if all(results):
        #     print("\nAll verifications completed successfully")
        # else:
        #     print("\nSome verifications failed")
        #     failed_tasks = [(i, tasks[i]) for i, success in enumerate(results) if not success]
        #     print("Failed verifications:")
        #     for i, (path, model) in failed_tasks:
        #         print(f"  - Task {i}: {model}")

        # --- Verify all models --- #
        # for m_j in range(m_bar_max + 1):
        #     for partition in partitions:
                if not run_multicore:
                    print(f"\n--- Verifying model for partition {p_ct} and m_bar = {m_j} misses ---")
                    modelfilename = f'{system}_p{p_ct}_K{Kmax}_mbar{m_j}' # +.model
                    modelfile = os.path.join(curdir, f'{system}', modelfilename)
                    parentdir = os.path.dirname(modelfile)
                    cmdlist = ['wsl', 'bash', '-c', f'../../flowstar-2.1.0/flowstar < {modelfilename}.model 2>&1 | tee {modelfilename}.log']
                    print(f"Executing command: {' '.join(cmdlist)}")
                    # result = subprocess.run(cmdlist, capture_output=True, text=True, check=True)
                    result = subprocess.Popen(cmdlist, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE )
                    print(f"Completed verification for {modelfilename}.model")
        
            if run_multicore:
                print(f"\n--- Starting parallel verifications for all partitions under m_bar = {m_j} misses ---")
                cmdlist =['wsl', 'bash' , '-c', 'ls *.model | taskset -c "0,1,2,3,4,5,6,7,8,9" parallel -j 9 --bar  "../../flowstar-2.1.0/flowstar < {{}} 2>&1 | tee {{}}.log"']
                print(f"Executing command: {' '.join(cmdlist)}")
                result = subprocess.run(cmdlist, capture_output=True, text=True, check=True)
        # # result = subprocess.run(['wsl', 'bash', '-c', f'cd /mnt/d/workspace/safe_weaklyHard_synth/models/{system} && ls && ../../flowstar-2.1.0/flowstar < {modelfilename}.model'], capture_output=True, text=True)
        # print("Command Output:")
        # print(result.stdout)
        # print("Command Errors (if any):")
        # print(result.stderr)
        # Move log file to logs directory
            
            # result = subprocess.run(['wsl', 'bash', '-c', 'mv *.log logs'], check=True)
            # result = subprocess.run(['wsl', 'bash', '-c', 'mv *.model verified_models'], check=True)
            
        
        # for m_j in range(m_bar_max + 1):
            for partition in partitions:
                modelfilename = f'{system}_p{p_ct}_K{Kmax}_mbar{m_j}' # +.model
                modelfile = os.path.join(system_dir, modelfilename)
                is_safe = check_log(log_file_path = os.path.join(system_dir, f'{modelfilename}.log'))
                if is_safe:
                    # sg[m_j][len(locations)] = sg[m_j].get(len(locations), set()).union({partition})
                    sg[m_j][len(locations)].append(partition)  # add Polytope object to list
                    print(f"Model {modelfilename} is SAFE, adding partition to sg[{m_j}][{len(locations)}]")

                partitions = list(sg[m_j][len(locations)]) # update partitions to only those that are safe
                verify_ct = verify_ct - 1

    
    spec_list = [{'X': Polytope.union(sg[0][len(locations)]), 'm_bar': m_j, 'cm_bar': cm_bar, 'cm': 0, 'K': Kmax} for m_j in range(m_bar_max + 1)]
    print(f"\nFinal specification list:")
    for spec in spec_list:
        print(f"  - {spec}")
