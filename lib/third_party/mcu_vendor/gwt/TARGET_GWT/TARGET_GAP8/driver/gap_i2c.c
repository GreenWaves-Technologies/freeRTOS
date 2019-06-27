/*
 * Copyright (c) 2018, GreenWaves Technologies, Inc.
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
 */

#include "gap_i2c.h"

/*******************************************************************************
 * Variables, macros, structures,... definition
 ******************************************************************************/

/* Pointers to I2C bases for each instance. */
static I2C_Type *const s_i2cBases[] = I2C_BASE_PTRS;

/* I2C transfer command sequence array. */
static uint8_t s_command_sequence[64];

/*******************************************************************************
 * Function definition
 ******************************************************************************/

uint32_t I2C_GetInstance(I2C_Type *base)
{
    uint32_t instance;
    /* Find the instance index from base address mapping. */
    for (instance = 0; instance < ARRAY_SIZE(s_i2cBases); instance++)
    {
        if (s_i2cBases[instance] == base)
            break;
    }
    assert(instance < ARRAY_SIZE(s_i2cBases));
    return instance;
}

status_t I2C_Init(I2C_Type *base, const i2c_config_t *Config, uint32_t srcClock_Hz)
{
    assert(Config);

    /* Enable I2C UDMA clock. */
    UDMA_Init((UDMA_Type *)base);

    /* Configure baud rate. */
    return I2C_SetBaudRate(base, Config->baudRate_Bps, srcClock_Hz);
}

void I2C_Deinit(I2C_Type *base)
{
    /* Disable I2C UDMA clock. */
    UDMA_Deinit((UDMA_Type *)base);
}

void I2C_GetDefaultConfig(i2c_config_t *Config)
{
    assert(Config);
    /* Default baud rate at 100kbps. */
    Config->baudRate_Bps  = 100000;
}

status_t I2C_SetBaudRate(I2C_Type *base, uint32_t baudRate_Bps, uint32_t srcClock_Hz)
{
    uint32_t index = 0;
    uint32_t clkDivider = (srcClock_Hz >> 2) / baudRate_Bps;

    s_command_sequence[index++] = I2C_CMD_CFG;
    s_command_sequence[index++] = (clkDivider >> 8) & 0xFF;
    s_command_sequence[index++] = (clkDivider) & 0xFF;
    return I2C_TransferBlocking(base, (const uint8_t*)&s_command_sequence, index,
                                NULL, 0);
}

status_t I2C_Start(I2C_Type *base)
{
    uint32_t index = 0;

    s_command_sequence[index++] = I2C_CMD_START;

    return I2C_TransferBlocking(base, (const uint8_t*)&s_command_sequence, index,
                                NULL, 0);
}

status_t I2C_Stop(I2C_Type *base)
{
    uint32_t index = 0;

    s_command_sequence[index++] = I2C_CMD_STOP;
    s_command_sequence[index++] = I2C_CMD_WAIT;
    s_command_sequence[index++] = 0xFF;

    return I2C_TransferBlocking(base, (const uint8_t*)&s_command_sequence, index,
                                NULL, 0);
}

status_t I2C_ByteWrite(I2C_Type *base, uint8_t data)
{
    uint32_t index = 0;

    s_command_sequence[index++] = I2C_CMD_WR;
    s_command_sequence[index++] = data;

    return I2C_TransferBlocking(base, (const uint8_t*)&s_command_sequence, index,
                                NULL, 0);
}

status_t I2C_Write(I2C_Type *base, uint32_t address, const char *data, uint32_t length, uint32_t stop)
{
    status_t status = 0;
    uint32_t index = 0;
    int32_t len = length;

    s_command_sequence[index++] = I2C_CMD_START;
    s_command_sequence[index++] = I2C_CMD_WR;
    s_command_sequence[index++] = address | 0x0;

    while (len > 0)
    {
        /* UDMA capacity : 128kB/transfer. */
        uint32_t size  = (len > 0x1ffff) ? 0x1ffff : len;
        s_command_sequence[index++] = I2C_CMD_RPT;
        s_command_sequence[index++] = size;
        s_command_sequence[index++] = I2C_CMD_WR;

        /* Transfer sequence header with written data repeately (maximum 128 kB) by udma. */
        status = I2C_TransferBlocking(base, (const uint8_t*)&s_command_sequence, index,
                                      NULL, 0);

        if (status != uStatus_Success)
        {
            return uI2C_Error;
        }

        /* Transfer data section. */
        status = I2C_TransferBlocking(base, data, size, NULL, 0);
        if (status != uStatus_Success)
        {
            return uI2C_Error;
        }

        /* Stop command. */
        if ((len < 0x1ffff) && stop)
        {
            index = 0;
            s_command_sequence[index++] = I2C_CMD_STOP;
            s_command_sequence[index++] = I2C_CMD_WAIT;
            s_command_sequence[index++] = 0xFF;

            status = I2C_TransferBlocking(base, (const uint8_t*)&s_command_sequence, index,
                                          NULL, 0);
        }
        index = 0;
        len  -= 0x1ffff;
        data += 0x1ffff;
    }

    return status;
}

status_t I2C_Read(I2C_Type *base, uint32_t address, char *data, uint32_t length, uint32_t stop)
{
    status_t status;
    uint32_t index = 0;
    int32_t len = length;

    s_command_sequence[index++] = I2C_CMD_START;
    s_command_sequence[index++] = I2C_CMD_WR;
    s_command_sequence[index++] = address | 0x1;

    while (len > 0)
    {
        /* UDMA capacity : 128kB/transfer. */
        uint32_t size  = (len > 0x1ffff) ? 0x1ffff : len;
        s_command_sequence[index++] = I2C_CMD_RPT;
        s_command_sequence[index++] = size - 1;
        s_command_sequence[index++] = I2C_CMD_RD_ACK;
        s_command_sequence[index++] = I2C_CMD_RD_NACK;

        /* Stop command. */
        if ((size < 0x1ffff) && stop)
        {
            s_command_sequence[index++] = I2C_CMD_STOP;
            s_command_sequence[index++] = I2C_CMD_WAIT;
            s_command_sequence[index++] = 0xFF;
        }

        /* Transfer header with receive buffer. */
        status = I2C_TransferBlocking(base, (const uint8_t*)&s_command_sequence, index,
                                      data, size);
        if (status != uStatus_Success)
        {
            return uI2C_Error;
        }
        index = 0;
        len  -= 0x1ffff;
        data += 0x1ffff;
    }

    return status;
}

static status_t I2C_TransferBlocking(I2C_Type *base, const void *tx_buffer, uint32_t tx_length,
                                     void *rx_buffer, uint32_t rx_length)
{
    status_t status = uStatus_NoTransferInProgress;
    udma_req_info_t info;

    if (rx_length)
    {
        info.dataAddr    = (uint32_t) rx_buffer;
        info.dataSize    = (uint32_t) rx_length;
        info.configFlags = UDMA_CFG_EN(1);
        info.isTx        = 0;
        status = UDMA_BlockTransfer((UDMA_Type *)base, &info, UDMA_NO_WAIT);
    }

    if (tx_length)
    {
        info.dataAddr    = (uint32_t) tx_buffer;
        info.dataSize    = (uint32_t) tx_length;
        info.configFlags = UDMA_CFG_EN(1);
        info.isTx        = 1;

        if(rx_length)
            status = UDMA_BlockTransfer((UDMA_Type *)base, &info, UDMA_WAIT_RX);
        else
            status = UDMA_BlockTransfer((UDMA_Type *)base, &info, UDMA_WAIT);
    }
    return status;
}

/*! @brief Structure passed to the End of Transmission callback. */
typedef struct
{
    I2C_Type *base;       /*!< I2C base pointer. */
    i2c_handle_t *handle; /*!< Handler passed by the user, it will be called by the EOT. */
} cb_arg;

/*! @brief End Of Transmission callback.
 *
 * This function is called when an asynchronous transfer is completed.
 * It executes the user callback function along with its parameters.
 */
static void I2C_stop(void *arg)
{
    uint32_t index = 0;
    cb_arg *cb = (cb_arg *) arg;
    I2C_Type *base = (I2C_Type *) cb->base;
    i2c_handle_t *eot = NULL;

    s_command_sequence[index++] = I2C_CMD_STOP;
    s_command_sequence[index++] = I2C_CMD_WAIT;
    s_command_sequence[index++] = 0xFF;

    eot->transfer.txData      = s_command_sequence;
    eot->transfer.rxData      = NULL;
    eot->transfer.txDataSize  = index;
    eot->transfer.rxDataSize  = 0;
    eot->transfer.configFlags = 0;
    I2C_TransferNonBlocking(base, eot);
}

status_t I2C_Write_Async(I2C_Type *base, uint32_t address, const char *data,
                         uint32_t length, i2c_handle_t *handle)
{
    status_t status = uStatus_InvalidArgument;
    uint32_t index = 0;
    /* UDMA capacity : 128kB/transfer. */
    if (length <= 0x1ffff)
    {
        cb_arg cb = { .base = base, .handle = handle };
        i2c_handle_t eot;
        I2C_CreateHandler(base, &eot, I2C_stop, &cb);

        s_command_sequence[index++] = I2C_CMD_START;
        s_command_sequence[index++] = I2C_CMD_WR;
        s_command_sequence[index++] = address | 0x0;
        s_command_sequence[index++] = I2C_CMD_RPT;
        s_command_sequence[index++] = length;
        s_command_sequence[index++] = I2C_CMD_WR;

        /* SoT + EoT in callback. */
        eot.transfer.txData      = s_command_sequence;
        eot.transfer.rxData      = NULL;
        eot.transfer.txDataSize  = index;
        eot.transfer.rxDataSize  = 0;
        eot.transfer.configFlags = 0;
        status = I2C_TransferNonBlocking(base, &eot);
        if (status != uStatus_Success)
        {
            return status;
        }

        /* Data + CallbackUser in callback. */
        handle->transfer.txData      = (uint8_t*)data;
        handle->transfer.rxData      = NULL;
        handle->transfer.txDataSize  = length;
        handle->transfer.rxDataSize  = 0;
        handle->transfer.configFlags = 0;
        status = I2C_TransferNonBlocking(base, handle);
    }

    return status;
}

status_t I2C_Read_Async(I2C_Type *base, uint32_t address, char *data,
                        uint32_t length, i2c_handle_t *handle)
{
    status_t status = uStatus_InvalidArgument;
    uint32_t index = 0;
    /* UDMA capacity : 128kB/transfer. */
    if (length <= 0x1ffff)
    {
        s_command_sequence[index++] = I2C_CMD_START;
        s_command_sequence[index++] = I2C_CMD_WR;
        s_command_sequence[index++] = address | 0x1;
        s_command_sequence[index++] = I2C_CMD_RPT;
        s_command_sequence[index++] = length - 1;
        s_command_sequence[index++] = I2C_CMD_RD_ACK;
        s_command_sequence[index++] = I2C_CMD_RD_NACK;

        /* SoT + CallbackUser in callback. */
        handle->transfer.txData      = s_command_sequence;
        handle->transfer.rxData      = (uint8_t *)data;
        handle->transfer.txDataSize  = index;
        handle->transfer.rxDataSize  = length;
        handle->transfer.configFlags = 0;
        status = I2C_TransferNonBlocking(base, handle);
    }

    return status;
}

static status_t I2C_TransferNonBlocking(I2C_Type *base, i2c_handle_t *handle)
{
    status_t status = uStatus_NoTransferInProgress;
    handle->state = uI2C_Busy;

    if (handle->transfer.rxDataSize)
    {
        udma_req_t *udma_req       = UDMA_FindAvailableRequest();
        udma_req->info.dataAddr    = UDMA_SADDR_ADDR(handle->transfer.rxData);
        udma_req->info.dataSize    = UDMA_SIZE_SIZE(handle->transfer.rxDataSize);
        udma_req->info.configFlags = UDMA_CFG_EN(1);
        udma_req->info.isTx        = 0;
        udma_req->info.channelId   = UDMA_EVENT_I2C0_RX + (handle->module << 1);
        udma_req->info.ctrl        = UDMA_CTRL_DUAL_RX;
        udma_req->info.task        = (uint32_t)handle;
        udma_req->info.repeat.size = 0;

        status = UDMA_SendRequest((UDMA_Type *)base, udma_req);
        if (status != 1)
        {
            return uStatus_I2C_Error;
        }
    }

    if (handle->transfer.txDataSize)
    {
        udma_req_t *udma_req       = UDMA_FindAvailableRequest();
        udma_req->info.dataAddr    = UDMA_SADDR_ADDR(handle->transfer.txData);
        udma_req->info.dataSize    = UDMA_SIZE_SIZE(handle->transfer.txDataSize);
        udma_req->info.configFlags = UDMA_CFG_EN(1);
        udma_req->info.isTx        = 1;
        udma_req->info.channelId   = UDMA_EVENT_I2C0_TX + (handle->module << 1);
        udma_req->info.repeat.size = 0;
        if (handle->transfer.rxDataSize)
        {
            udma_req->info.ctrl = UDMA_CTRL_DUAL_TX;
            udma_req->info.task = 0;
        }
        else
        {
            udma_req->info.ctrl = UDMA_CTRL_NORMAL;
            udma_req->info.task = (uint32_t)handle;
        }
        status = UDMA_SendRequest((UDMA_Type *)base, udma_req);
    }
    return ((status == 1) ? uStatus_Success : uStatus_I2C_Error);
}

void I2C_CreateHandler(I2C_Type *base, i2c_handle_t *handle,
                       i2c_transfer_callback_t callback, void *userData)
{
    assert(handle);

    handle->state    = uI2C_Idle;
    handle->callback = callback;
    handle->userData = userData;
    handle->module   = I2C_GetInstance(base);
}

void I2C_IRQHandler(void *arg)
{
    i2c_handle_t *handle = (i2c_handle_t *) arg;
    handle->state = uI2C_Idle;
    if (handle->callback != NULL)
    {
        handle->callback(handle->userData);
    }
}
