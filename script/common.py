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


def run_benchmark(args="", print_output=True, verbose=True):
    output_file = globals["paths"]["outfile_timings"] 
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
    return load_df(output_file)

def load_df(file_name):
    df = pd.read_csv(file_name, sep=";")
    df = df.set_index(["n_rays","kernel"])
 
    # if raycount is low some values for n_rays may appear multiple times
    df = df.groupby(["n_rays", "kernel"]).mean()
    return df 


globals = load_globals()