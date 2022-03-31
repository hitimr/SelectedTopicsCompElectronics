import socket
import numpy as np
import pandas as pd
import platform
import logging
import cpuinfo
import matplotlib.pyplot as plt
import multiprocessing
import os
import datetime

import common


def get_CPU_data(cpu_name):
    if(cpu_name == "AMD Ryzen 7 3700X 8-Core Processor"):
        return {
            "cpu_price": 350.0,
            "cpu_power": 65.0,
            "n_threads": 8,
            "cpu_short_name": "Ryzen7_3700X"
        }

    if(cpu_name == "Intel(R) Xeon(R) Gold 6248 CPU @ 2.50GHz"):
        return {
            "cpu_price": 3290.0,
            "cpu_power": 150.0,
            "n_threads": 40,
            "cpu_short_name": "Xeon_6248"
        }

    else:
        logging.warning(f"Unknown CPU {cpu_name}")
        return {}


def get_GPU_data(gpu_name):
    if(gpu_name == "NVIDIA GeForce GTX 970"):
        return {
            "gpu_price": 260.0,
            "gpu_power": 500.0,
            "gpu_grid_size": 256,
            "gpu_block_size": 256,
            "gpu_short_name": "GTX_970"
        }
    if(gpu_name == "Tesla T4"):
        return {
            "gpu_price": 2500.0,
            "gpu_power": 70.0,
            "gpu_grid_size": 256,
            "gpu_block_size": 256,
            "gpu_short_name": "Tesla_T4"
        }


def is_cluster():
    if(multiprocessing.cpu_count() > 20):
        return True
    else:
        return False


def get_system_information():
    cpu_name = cpuinfo.get_cpu_info()["brand_raw"]
    gpu_name = os.popen("nvidia-smi --query-gpu=name --format=csv").read().split("\n")[1]

    data = {
        "host_name": socket.gethostname(),
        "cpu_name": cpu_name,
        "gpu_name": gpu_name
    }

    data.update(get_CPU_data(cpu_name))
    data.update(get_GPU_data(gpu_name))
    return data


# Assemble arguments based on system
sys_info = get_system_information()
if(is_cluster()):
    # Cluster
    args = {
        "p_rays_start": 12,
        "p_rays_end": 20,
        "n_bench": 64,
    }
else:
    # Home PC
    args = {
        "p_rays_start": 12,
        "p_rays_end": 18,
        "n_bench": 8
    }

# Filename for output
args.update(sys_info)
timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
outfile = common.globals["paths"]['out_dir'] + f"{timestamp}_{args['cpu_short_name']}_{args['gpu_short_name']}.csv"


# launch benchmark
df = common.run_benchmark(
    outfile,
    args=f"--ray_offset 0.2 \
    --outfile {outfile} \
    --omp_n_threads {args['n_threads']} \
    --p_rays_start {args['p_rays_start']} \
    --p_rays_end {args['p_rays_end']} \
    --n_bench {args['n_bench']}",
    print_output=True)
