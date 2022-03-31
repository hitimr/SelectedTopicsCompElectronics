from pathlib import Path
import sys
import pandas as pd
import subprocess
import json



def load_globals():
    proj_root_dir = str(Path(__file__).parent.parent) + "/"
    with open(proj_root_dir + "globals.json") as json_file:
        data = json.load(json_file)

    for path in data["paths"]:
        data["paths"][path] = proj_root_dir + data["paths"][path]
         
    return data

def run_command(command):
    return subprocess.run(command.split(sep=" "), check=True)

def run_benchmark(outfile, args="", print_output=True, verbose=True):
    executable = globals["paths"]["executable"]
    cmd = str(f"{executable} {args}")
    print(cmd)
    if(verbose):
        print(f"Running benchmark with {args}")

    if(print_output):
        subprocess.run(cmd.split(sep=" "), check=True)
    else:
        subprocess.run(cmd.split(sep=" "), check=True,
                       stdout=subprocess.DEVNULL)
    return load_df(outfile)

def load_df(file_name):
    df = pd.read_csv(file_name, sep=";")
    df = df.set_index(["n_rays","kernel"])
 
    # if raycount is low some values for n_rays may appear multiple times
    df = df.groupby(["n_rays", "kernel"]).mean()
    return df 



globals = load_globals()

'--ray_offset 0.2     --omp_n_threads 8     --p_rays_start 12     --p_rays_end 18     --n_bench 8 --outfile /home/hiti/Workspace/SelectedTopicsCompElectronics/build/2022-03-31_23-47-30_Ryzen7_3700X_GTX_970.csv'