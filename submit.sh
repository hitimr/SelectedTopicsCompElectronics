#!/bin/bash
#SBATCH --job-name=CUDA_Test    # Job name
#SBATCH --mail-type=BEGIN,END,FAIL          # Mail events (NONE, BEGIN, END, FAIL, ALL)
#SBATCH --mail-user=hitimr@gmail.com     # Where to send mail	
#SBATCH --ntasks=1                    # Run on a single CPU
#SBATCH --time=00:10:00               # Time limit hrs:min:sec
#SBATCH --output=serial_test_%j.log   # Standard output and error log
#SBATCH --nodes=1


nvcc -v