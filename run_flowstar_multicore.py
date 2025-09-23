import subprocess
import os
from multiprocessing import Pool
from functools import partial

def run_flowstar(wsl_path, modelfilename):
    """Run flowstar for a single model file"""
    # First verify the files exist
    check_cmd = f'if [ ! -f "{modelfilename}.model" ]; then echo "Model file not found"; exit 1; fi; if [ ! -x "../../flowstar-2.1.0/flowstar" ]; then echo "Flowstar executable not found or not executable"; exit 1; fi'
    
    # Build the complete command with error handling
    cmd = f'''
    cd {wsl_path} && \\
    {check_cmd} && \\
    echo "Running flowstar for {modelfilename}" && \\
    ../../flowstar-2.1.0/flowstar < {modelfilename}.model
    '''
    
    try:
        # Run with shell=True to properly handle the complex command
        result = subprocess.run(['wsl', 'bash', '-c', cmd], 
                             capture_output=True, 
                             text=True, 
                             check=False)  # Don't raise exception immediately
        
        print(f"Command exit code: {result.returncode}")
        print("Command Output:")
        print(result.stdout)
        
        if result.stderr:
            print("Command Errors:")
            print(result.stderr)
            
        if result.returncode != 0:
            print(f"Error: flowstar exited with code {result.returncode}")
            return False
            
        print(f"Completed verification for {modelfilename}")
        return True
        
    except Exception as e:
        print(f"Error executing flowstar for {modelfilename}")
        print(f"Exception: {str(e)}")

        return False
