import numpy as np
import pandas as pd
import datetime

import common



def to_str_args(args):
    str_args = ""
    for key in args:
        if args[key] == "":
            str_args += f"--{key} "
        else:
            str_args += f"--{key} {args[key]} "
    return str_args


if __name__ == "__main__":
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    gpu_short_name = "Tesla_T4"
    cpu_short_name = "Xeon_6248"
    load_factor = 1
    outfile = common.globals["paths"]['out_dir'] + f"{timestamp}_{'gpu_short_name'}_loadfactor_{load_factor}.csv"

    args = {
        "outfile" : outfile,
        "p_rays_start": 6,
        "p_rays_end": 27,
        "n_bench": 128,
        "cpu_price": 3290.0,
        "cpu_power": 150.0,
        "omp_n_threads": 40,
        "gpu_price": 2500.0,
        "gpu_power": 70.0,
        "gpu_grid_size": 256,
        "gpu_block_size": 256,
        # "skip-openvdb" : "",
        # "skip-nanovdb-cpu": "",
        "skip-checks": "",
        "gpu_load_factor": load_factor,
        "shuffle-rays": "",
        "inner_sphere_offset": 2

    }




    str_args = to_str_args(args) 
    str_args

    common.run_benchmark(outfile, str_args, print_output=True, verbose=True)
