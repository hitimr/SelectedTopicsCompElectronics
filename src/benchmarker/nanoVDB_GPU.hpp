#pragma once
#include <nanovdb/util/GridHandle.h>
#include <nanovdb/util/HostBuffer.h>

void run_nanoVDB_GPU(nanovdb::GridHandle<nanovdb::HostBuffer> &handle, size_t n_rays);