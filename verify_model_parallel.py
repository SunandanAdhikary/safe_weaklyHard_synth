import os
import subprocess
import shutil
import platform
from polytope import Polytope
from utils.check_log import check_log

def verify_model_parallel(modelfilepaths):
    '''
    parallelly runs verification/reacahbility analysis of all .model files in a folder using 
    flow star and outputs their safety statuses along with output messages and log them
    inputs:
        modelfilepaths: list of model file paths in a filder without .model extention
    outputs:
        is_safe_status: stores 0/1/-1 at i-th position of an array as the status of the i-th model file in modelfilepaths
                        1 if safe, 0 if unsafe, -1 is unknown
        reults: an array filled with output messages and errors for each run in the same order as the modelfiles
    '''
    system_dir = os.path.dirname(modelfilepaths[0]) # same as '\\'.join(modelfilepaths[0].split('\\')[:-2])+'\\'
    # Convert Windows path to WSL path and go to path, add to tasks
    if platform.system() != "Windows":
        system_dir = system_dir.replace('D:', '/mnt/d').replace('\\', '/')
    os.chdir(system_dir) # change to system directory
    # Create log, verified model directories in system directory if they do not exist
    os.makedirs('logs', exist_ok=True)
    logfilepath = os.path.join(system_dir, 'logs')
    os.makedirs('verified_models', exist_ok=True)
    verifiedfilepath = os.path.join(system_dir, 'verified_models')

    results = []
    is_safe_status = []
    # if not run_multicore:
        # wsl_path = system_dir.replace('D:', '/mnt/d').replace('\\', '/')
    # --------------------------------for multicore case--------------------------------- #
    # if run_multicore :
    print(f"\n--- Starting parallel verifications for all partitions ---")
    cmdlist = ['wsl', 'bash' , '-c', 'ls *.model | taskset -c "0,1,2,3,4,5,6,7,8,9" parallel -j 9 --bar  "../../flowstar-2.1.0/flowstar < {{}} 2>&1 | tee {{}}.log"'] # && mv *.log logs && mv *.model verified_models']
    # print(f"Executing command: {' '.join(cmdlist)}")
    if platform.system() != "Windows":
         cmdlist = ['bash' , '-c', 'ls *.model | taskset -c "0,1,2,3,4,5,6,7,8,9" parallel -j 9 --bar  "../../flowstar-2.1.0/flowstar < {{}} 2>&1 | tee {{}}.log"'] # && mv *.log logs && mv *.model verified_models']
    try:
        result = subprocess.run(cmdlist, capture_output=True, text=True, check=True, universal_newlines=True)
    except subprocess.CalledProcessError as e:
        print(f"Error during parallel verification: {e}")
    # for printing during execution of command
    for line in result.stdout:
        print(line, end='')  # Print each line of output as it is received
    ret_code = result.wait()  # Wait for the process to complete  
    if ret_code == 0:
        print(f"Completed all parallel verifications")
        print(f"Command Output:\n{result.stdout}")
        print(f"Command Errors (if any):\n{result.stderr}")
        results.append(result)
    else:
        print(f"Verification failed with return code {ret_code}")
        print(f"Command Output:\n{result.stdout}")
        print(f"Command Errors (if any):\n{result.stderr}")

    for modelfilepath in modelfilepaths:
        modelfilename = os.path.basename(modelfilepath).split('.')[0]
        # modelfilename = modelfilepath.split('//')[-1].split('.')[0]
        # modelfilename = modelfilename.split('.')[0]
        logfilename = f'{modelfilename}+.log'
        print(f"Verifying {modelfilename}.model-->{logfilename}")
        # ---------------------log analysis for both parallel and individual cases----------------------- #
        if ret_code == 0:
            is_safe = check_log(log_file_path = os.path.join(system_dir, f'{modelfilename}.log'))
            if is_safe:
                # safeguards[miss_ct][len(locations)] = safeguards[miss_ct].get(len(locations), set()).union({partition})
                # safeguards[miss_ct][nl].append(partition)  # add Polytope object to list
                print(f"Model {modelfilename} is SAFE")
                # move model and log files to designated folededrs once verificationis done
                shutil.move(modelfilepath, verifiedfilepath)
                shutil.move(logfilename, logfilepath)
            else:
                if is_safe != -3:
                    print(f"Model {modelfilename} is UNSAFE" if is_safe == 0 else f"Model {modelfilename} verification INCOMPLETE" if is_safe == -1 else "logging error, retry")
                    # move model and log files to designated folededrs once verificationis done
                    shutil.move(modelfilepath, verifiedfilepath)
                    shutil.move(logfilename, logfilepath)
                else:
                    print('error in log check')
        else:
            print(f"Verification failed for {modelfilename}.model with return code {ret_code}")
        is_safe_status.append(is_safe)

    return results, is_safe_status


# if __name__ == "__main__":
    # 