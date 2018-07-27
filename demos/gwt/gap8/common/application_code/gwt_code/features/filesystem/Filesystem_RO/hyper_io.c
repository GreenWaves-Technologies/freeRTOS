/******************************************************************************************
 *
 * Copyright (c) 2018 , GreenWaves Technologies, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of GreenWaves Technologies, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************************/
#include "hyper_io.h"

#ifdef __FREERTOS__
#include "FreeRTOS.h"
#define malloc pvPortMalloc
#define free   vPortFree
#endif

static void HYPERBUS_Pin( PinName dq0, PinName dq1, PinName dq2, PinName dq3,
                          PinName dq4, PinName dq5, PinName dq6, PinName dq7,
                          PinName ck, PinName ckn, PinName rwds, PinName csn0, PinName csn1 )
{
    /* pin out the hyperbus pins */
    pinmap_pinout(ck,   PinMap_HYPERBUS_CLK);
    pinmap_pinout(ckn,  PinMap_HYPERBUS_CLKN);
    pinmap_pinout(rwds, PinMap_HYPERBUS_RWDS);
    pinmap_pinout(csn0, PinMap_HYPERBUS_CSN0);
    pinmap_pinout(csn1, PinMap_HYPERBUS_CSN1);
    pinmap_pinout(dq0,  PinMap_HYPERBUS_DQ0);
    pinmap_pinout(dq1,  PinMap_HYPERBUS_DQ1);
    pinmap_pinout(dq2,  PinMap_HYPERBUS_DQ2);
    pinmap_pinout(dq3,  PinMap_HYPERBUS_DQ3);
    pinmap_pinout(dq4,  PinMap_HYPERBUS_DQ4);
    pinmap_pinout(dq5,  PinMap_HYPERBUS_DQ5);
    pinmap_pinout(dq6,  PinMap_HYPERBUS_DQ6);
    pinmap_pinout(dq7,  PinMap_HYPERBUS_DQ7);

    hyperbus_master_config_t  masterConfig;

    HYPERBUS_MasterGetDefaultConfig( &masterConfig );
    HYPERBUS_MasterInit( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterConfig, SystemCoreClock );
}

static void HYPERBUS_Configure( hyperbus_transfer_t *masterXfer )
{
    /* Set VCR to 5 delay cycles */
    for ( uint32_t i = 0; i < 4; i++ )
    {
        masterXfer->txData = &VCR_Seq[i].data;
        masterXfer->txDataSize = 2;
        masterXfer->rxData = 0;
        masterXfer->rxDataSize = 0;
        masterXfer->configFlags = 32;
        masterXfer->addr = ( VCR_Seq[i].addr << 1 );
        masterXfer->device = uHYPERBUS_Flash;
        masterXfer->reg_access = uHYPERBUS_Mem_Access;

        HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, masterXfer );
    }
}

int HYPERBUS_IO_Init()
{
    /* Hyperbus Flash Default latency is 5 clock, can not change */
    uint32_t latency = 0;

    masterXfer = ( hyperbus_transfer_t * ) malloc( sizeof( hyperbus_transfer_t ) );

    HYPERBUS_Pin( HYPERBUS_DQ0, HYPERBUS_DQ1, HYPERBUS_DQ2, HYPERBUS_DQ3,
                  HYPERBUS_DQ4, HYPERBUS_DQ5, HYPERBUS_DQ6, HYPERBUS_DQ7,
                  HYPERBUS_CLK, HYPERBUS_CLKN, HYPERBUS_RWDS, HYPERBUS_CSN0, HYPERBUS_CSN1 );

    latency = 0;
    /* Config memory access timing for TX and RX*/
    HYPERBUS_SetTiming( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, 4, 4, 4, latency, 0, uHYPERBUS_Flash );
    HYPERBUS_SetTiming( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, 4, 4, 4, latency, 1, uHYPERBUS_Flash );
    HYPERBUS_Configure( masterXfer );

    return 0;
}

void HYPERBUS_IO_Deinit( void )
{
    HYPERBUS_MasterDeInit( (HYPERBUS_Type *) HYPERBUS_BASE_PTRS );

    free( masterXfer );
}

int HYPERBUS_IO_Erase( uint32_t addr )
{
    for ( uint32_t i = 0; i < 5; i++ )
    {
        masterXfer->txData      = &Erase_Seq[i].data;
        masterXfer->txDataSize  = 2;
        masterXfer->rxData      = 0;
        masterXfer->rxDataSize  = 0;
        masterXfer->configFlags = 32;
        masterXfer->addr        = Erase_Seq[i].addr << 1;
        masterXfer->device      = uHYPERBUS_Flash;
        masterXfer->reg_access  = uHYPERBUS_Mem_Access;

        HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, masterXfer );
    }

    masterXfer->txData      = &Erase_Seq[5].data;
    masterXfer->txDataSize  = sizeof( uint16_t );
    masterXfer->rxData      = 0;
    masterXfer->rxDataSize  = 0;
    masterXfer->configFlags = 32;
    masterXfer->addr        = addr;
    masterXfer->device      = uHYPERBUS_Flash;
    masterXfer->reg_access  = uHYPERBUS_Mem_Access;

    HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, masterXfer );

    return 0;
}

int HYPERBUS_IO_Write( uint32_t addr, uint32_t size, void *buf )
{
    /* Write to Buffer command sequence */
    for( uint32_t i = 0; i < 3; i++ )
    {
        masterXfer->txData      = &WP_Seq[i].data;
        masterXfer->txDataSize  = 2;
        masterXfer->rxData      = 0;
        masterXfer->rxDataSize  = 0;
        masterXfer->configFlags = 32;
        masterXfer->addr        = WP_Seq[i].addr << 1;
        masterXfer->device      = uHYPERBUS_Flash;
        masterXfer->reg_access  = uHYPERBUS_Mem_Access;

        HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, masterXfer );
    }

    /* Word Program */
    masterXfer->txData      = buf;
    masterXfer->txDataSize  = size;
    masterXfer->rxData      = 0;
    masterXfer->rxDataSize  = 0;
    masterXfer->configFlags = 32;
    masterXfer->addr        = addr;
    masterXfer->device      = uHYPERBUS_Flash;
    masterXfer->reg_access  = uHYPERBUS_Mem_Access;

    HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, masterXfer );

    return 0;
}

int HYPERBUS_IO_Read( uint32_t addr, uint32_t size, void *buf )
{
    masterXfer->txData      = 0;
    masterXfer->txDataSize  = 0;
    masterXfer->rxData      = buf;
    masterXfer->rxDataSize  = size;
    masterXfer->configFlags = 32;
    masterXfer->addr        = addr;
    masterXfer->device      = uHYPERBUS_Flash;
    masterXfer->reg_access  = uHYPERBUS_Mem_Access;

    HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, masterXfer );

    return 0;
}

int HYPERBUS_IO_Sync( void )
{
    uint16_t reg;
    hyperbus_transfer_t masterXferWrite, masterXferRead;

    write_val                   = Reg_Seq.data;
    masterXferWrite.txData      = ( uint16_t * ) &write_val;
    masterXferWrite.txDataSize  = 2;
    masterXferWrite.rxData      = 0;
    masterXferWrite.rxDataSize  = 0;
    masterXferWrite.configFlags = 32;
    masterXferWrite.addr        = Reg_Seq.addr << 1;
    masterXferWrite.device      = uHYPERBUS_Flash;
    masterXferWrite.reg_access  = uHYPERBUS_Mem_Access;

    masterXferRead.txData      = 0;
    masterXferRead.txDataSize  = 0;
    masterXferRead.rxData      = ( uint16_t * ) &read_val;
    masterXferRead.rxDataSize  = 2;
    masterXferRead.configFlags = 32;
    masterXferRead.addr        = 0;
    masterXferRead.device      = uHYPERBUS_Flash;
    masterXferRead.reg_access  = uHYPERBUS_Mem_Access;

    /* Wait the end of process
     * Status Register (SR)
     * bit 4 -> program status bit, 0-success ; 1-failure
     * bit 5 -> erase status bit,   0-success ; 1-failure
     * bit 7 -> device ready bit,   0-busy    ; 1-ready
     */
    do {
        HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterXferWrite );
        HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterXferRead );
        reg = ( ( read_val >> 16 ) & 0xffff );
    } while( !( reg & ( 1 << DEVICE_READY_OFFSET ) ) );

    return 0;
}
