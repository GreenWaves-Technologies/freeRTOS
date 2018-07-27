/*
 * Copyright (C) 2018 ETH Zurich and University of Bologna
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Copyright (C) 2018 GreenWaves Technologies
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gap_fs.h"

#ifdef __FREERTOS__
#include "FreeRTOS.h"
#define malloc pvPortMalloc
#define free   vPortFree
#endif

/*******************************************************************************
 * Definitions and variables
 ******************************************************************************/
static int  __fs_read_from_cache( fs_file_t *file, uint32_t addr, uint32_t size, uint32_t buffer );
static int  __fs_read_block     ( fs_file_t *file, uint32_t addr, uint32_t size, uint32_t buffer );
static int  __fs_read_cached    ( fs_file_t *file, uint32_t addr, uint32_t size, uint32_t buffer );
static int  __fs_read           ( fs_file_t *file, uint32_t addr, uint32_t size, uint32_t buffer );
static void __fs_try_read       ( fs_file_t *file );
/*******************************************************************************
 * APIs definition
 ******************************************************************************/

void fs_config_default( fs_config_t *fs_config )
{
    fs_config->fs_type = GAP_FS_READ_ONLY;
}

fs_handle_t *fs_mount( uint8_t device, fs_config_t *fs_config )
{
    if( fs_config == NULL )
        return NULL;

    fs_handle_t *fs = ( fs_handle_t * ) malloc( sizeof( fs_handle_t ) );
    if( fs == NULL )
        return NULL;
    fs->fs_size = 0;
    fs->fs_l2_offset = 0;
    fs->fs_l2_size = 0;
    fs->fs_info = NULL;
    fs->fs_cache_addr = 0;
    fs->fs_cache = NULL;
    fs->fs_cache = ( uint8_t * ) malloc( sizeof( FS_READ_THRESHOLD_BLOCK_FULL ) );
    if( fs->fs_cache == NULL )
        return NULL;
    fs->fs_device = device;

    uint32_t fs_size = 0;
    uint32_t fs_offset = 0;
    switch( fs->fs_device )
    {
    case fs_HYPER :
        /* Initialize device. */
        HYPERBUS_IO_Init( uHYPERBUS_Flash );
        /* Get offset of filesystem header */
        HYPERBUS_IO_Read( 0x00, 4, &(fs->fs_l2_offset) );
        /* Get header size. */
        HYPERBUS_IO_Read( fs->fs_l2_offset, 4, &(fs->fs_l2_size) );

        fs_size = ( ( fs->fs_l2_size + 3 ) & ~0x03 );
        fs_offset = fs->fs_l2_offset;
        fs->fs_info = ( uint32_t * ) malloc( fs_size );
        if( fs->fs_info == NULL )
            return NULL;
        HYPERBUS_IO_Read( fs_offset + 4, fs_size, fs->fs_info );
        break;
    default:
        return NULL;
    }

    return fs;
}

void fs_unmount( fs_handle_t *fs )
{
    if( fs != NULL )
    {
        if( fs->fs_cache != NULL )
            free( fs->fs_cache );
        if( fs->fs_info != NULL )
            free( fs->fs_info );
        switch( fs->fs_device )
        {
        case fs_HYPER :
            HYPERBUS_IO_Deinit();
        default :
            break;
        }
        free( fs );
    }
}


fs_file_t *fs_open( fs_handle_t *fs_handle, const char *file, uint8_t mode )
{
    uint32_t *fs_info = fs_handle->fs_info;
    uint32_t nb = *fs_info++;

    fs_desc_t *desc = NULL;
    uint32_t i = 0;

    for( i=0; i<nb; i++ )
    {
        desc = ( fs_desc_t * ) fs_info;

        if( strcmp( desc->name, file ) == 0 )
        {
            fs_file_t *file = ( fs_file_t * ) FC_Malloc( sizeof( fs_file_t ) );
            if( file == NULL )
                return NULL;
            file->offset = 0;
            file->size = desc->size;
            file->addr = desc->addr;
            file->fs = fs_handle;
            sprintf(file->name,"%s", desc->name);
            return file;
        }
        fs_info = ( uint32_t * )( ( uint32_t ) fs_info + sizeof( fs_desc_t ) + desc->path_size );
    }
    /* No file found. */
    return NULL;
}

void fs_close( fs_file_t *file )
{
    if( file != NULL )
        FC_MallocFree( file, sizeof( fs_file_t ) );
}

int fs_read( fs_file_t *file, uint32_t size, void *buffer )
{
    if( file == NULL )
        return IO_FILE_ERROR;
    if( buffer == NULL )
        return IO_READ_ERROR;

    uint32_t readBytes = size;
    if( file->offset + size > file->size )
        readBytes = file->size - file->offset;
    file->pending_buffer = ( uint32_t ) buffer;
    file->pending_size = readBytes;
    file->pending_addr = file->addr + file->offset;
    file->offset += readBytes;
    __fs_try_read( file );
    return readBytes;
}

int fs_seek( fs_file_t *file, int offset )
{
    if( file == NULL )
        return IO_FILE_ERROR;
    if( ( offset < 0 ) || ( offset > ( int ) file->size ) )
        return IO_SEEK_ERROR;
    file->offset = offset;
    return IO_OK;
}

/*******************************************/
/*******************************************/

static void __fs_try_read( fs_file_t *file )
{
    while( file->pending_size )
    {
        uint32_t size = __fs_read( file, file->pending_addr, file->pending_size, file->pending_buffer );
        file->pending_addr += size;
        file->pending_buffer += size;
        file->pending_size -= size;
    }
}

static int __fs_read( fs_file_t *file, uint32_t addr, uint32_t size, uint32_t buffer )
{
    /* Access without cache. */
    if( ( size <= FS_READ_THRESHOLD ) || ( ( addr & 0x3 ) != ( buffer & 0x3 ) ) )
        return __fs_read_cached( file, addr, size, buffer );

    /* Access to cache because of a HIT. */
    if( ( size <= FS_READ_THRESHOLD_BLOCK_FULL ) &&
        ( addr >= file->fs->fs_cache_addr ) &&
        ( ( addr + size ) < ( file->fs->fs_cache_addr +  FS_READ_THRESHOLD_BLOCK_FULL ) ) )
        return __fs_read_from_cache( file, addr, size, buffer );

    /* Otherwise, transfer to cache, then read from cache. */
    uint32_t prefix_size = addr & 0x03;
    uint32_t read_size = 0;
    if( prefix_size )
    {
        prefix_size = 4 - prefix_size;
        read_size = __fs_read_cached( file, addr, size, buffer );
        addr += prefix_size;
        buffer += prefix_size;
        size -= prefix_size;
    }

    uint32_t block_size = size & ~0x03;
    __fs_read_block( file, addr, block_size, buffer );

    return block_size;
}

static int __fs_read_cached( fs_file_t *file, uint32_t addr, uint32_t size, uint32_t buffer )
{
    if( size > ( FS_READ_THRESHOLD_BLOCK_FULL - ( addr & 0x03 ) ) )
        size = FS_READ_THRESHOLD_BLOCK_FULL - ( addr & 0x03 );

    if( ( addr < file->fs->fs_cache_addr ) ||
        ( ( addr + size ) > ( file->fs->fs_cache_addr + FS_READ_THRESHOLD_BLOCK_FULL ) ) )
    {
        file->fs->fs_cache_addr = addr & ~0x03;
        __fs_read_block( file, file->fs->fs_cache_addr, FS_READ_THRESHOLD_BLOCK_FULL, ( uint32_t ) file->fs->fs_cache );
        return 0;
    }

    return __fs_read_from_cache( file, addr, size, buffer );
}


static int __fs_read_block( fs_file_t *file, uint32_t addr, uint32_t size, uint32_t buffer )
{
    switch( file->fs->fs_device )
    {
    case fs_HYPER :
        return HYPERBUS_IO_Read( addr, size, ( void * ) buffer );

    default:
        return 0;
    }
}

static int __fs_read_from_cache( fs_file_t *file, uint32_t addr, uint32_t size, uint32_t buffer )
{
    memcpy( ( void * ) buffer, &file->fs->fs_cache[ addr - file->fs->fs_cache_addr ], size );
    return size;
}