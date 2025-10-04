import os
import subprocess
import shutil
import platform
from utils.check_log import check_log
from utils.plot_polytope import print_polytope

def verify_model(modelfilepaths):
    '''
    sequentially runs verification.reacahbility analysis of eaxh .model file in a folder using 
    flow star and outputs their safety statuses along with output messages and log them
    inputs:
        modelfilepaths: list of model file paths in a filder without .model extention
    outputs:
        is_safe_status: stores 0/1/-1 at i-th position of an array as the status of the i-th model file in modelfilenames
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
    # modelfilepath = os.path.join(system_dir, modelfilename)
    results = []
    is_safe_status = []
    # if not run_multicore:
        # wsl_path = system_dir.replace('D:', '/mnt/d').replace('\\', '/')
    for modelfilepath in modelfilepaths:
        # ---------------------for each modelfile--------------------------- #
        modelfilename = os.path.basename(modelfilepath).split('.')[0]
        # modelfilename = modelfilepath.split('//')[-1].split('.')[0]
        # modelfilename = modelfilename.split('.')[0]
        logfilename = f'{modelfilename}'+'.log'
        print(f"Verifying {modelfilename}.model-->{logfilename}")
        cmdlist = ['wsl', 'bash', '-c', f'../../flowstar-2.1.0/flowstar < {modelfilename}.model 2>&1 | tee {modelfilename}.log']
        if platform.system() != "Windows":
            cmdlist = ['bash', '-c', f'../../flowstar-2.1.0/flowstar < {modelfilename}.model 2>&1 | tee {modelfilename}.log']
        try:
            result = subprocess.Popen(cmdlist, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1)
            for line in result.stdout:
                try:
                    decoded_line = line.decode()
                    print(decoded_line, end='')  # Print each line of output as it is received
                except UnicodeDecodeError:
                    print("Warning: Could not decode output line")
            ret_code = result.wait()  # Wait for the process to complete
        except subprocess.CalledProcessError as e:
            print(f"Error during verification of {modelfilename}.model: {e}")
            ret_code = e.returncode
        except Exception as e:
            print(f"Unexpected error during verification of {modelfilename}.model: {e}")
            ret_code = -1
        # --------log analysis for both parallel and individual cases------------- #
        if ret_code == 0:
            print(f"Completed verification for {modelfilename}.model")
            log_file_path = modelfilepath.replace('.model', '.log')
            is_safe = check_log(log_file_path = logfilename)
            if int(is_safe) > 0:
                print(f"Model {modelfilename} is SAFE")
                # move model and log files to designated folededrs once verificationis done
                shutil.move(modelfilepath, verifiedfilepath)
                shutil.move(logfilename, logfilepath)
            else:
                if is_safe != -3:
                    print(f"Model {modelfilename} is UNSAFE" if is_safe == 0 else f"Model {modelfilename} verification INCOMPLETE")
                    # move model and log files to designated folededrs once verificationis done
                    shutil.move(modelfilepath, verifiedfilepath)
                    shutil.move(logfilename, logfilepath)
                else:
                    print('error in log check')
        else:
            print(f"Verification failed for {modelfilename}.model with return code {ret_code}")
            print(f"Command Output:\n{result.stdout}")
            print(f"Command Errors (if any):\n{result.stderr}")
            is_safe = -1
        
        results.append(result)
        is_safe_status.append(is_safe)
        
    
    return results, is_safe_status


# if __name__ == "__main__":
    # 