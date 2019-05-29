/*
 * Copyright (C) 2017 ETH Zurich, University of Bologna and GreenWaves Technologies
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 * Authors: Eric Flamand, GreenWaves Technologies (eric.flamand@greenwaves-technologies.com)
 *          Germain Haugou, ETH (germain.haugou@iis.ee.ethz.ch)
 */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "gap_l1_malloc.h"

#ifdef FEATURE_CLUSTER

/*******************************************************************************
 * Variables, macros, structures,... definition
 ******************************************************************************/

static malloc_t __l1_malloc_cluster; /*!< Cluster memory allocator. */

/*******************************************************************************
 * Function definition
 ******************************************************************************/

void *L1_Malloc(int32_t size)
{
    return __malloc(&__l1_malloc_cluster, size);
}

void L1_MallocFree(void *_chunk, int32_t size)
{
    __malloc_free(&__l1_malloc_cluster, _chunk, size);
}

void *L1_MallocAlign(int32_t size, int32_t align)
{
    return __malloc_align(&__l1_malloc_cluster, size, align);
}

void L1_MallocInit()
{
    __malloc_init(&__l1_malloc_cluster, (void *) &__heapsram_start, (int32_t) &__heapsram_size);
}

#endif
