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

#include "gap_sai.h"

#define CK_DIV_DEFAULT 25

/*******************************************************************************
 * Variables, macros, structures,... definitions
 ******************************************************************************/

/*! @brief Pointers to sai bases for each instance. */
static I2S_Type *const s_saiBases[] = I2S_BASE_PTRS;

/* Array of GPIO peripheral base address. */
static PORT_Type *const port_addrs[] = PORT_BASE_PTRS;

extern const PinMap PinMap_I2S_SCK[];
extern const PinMap PinMap_I2S_SDI[];
extern const PinMap PinMap_I2S_WS[];
extern const PinMap PinMap_HYPERBUS_CLK[];

/*******************************************************************************
 * Function definition
 ******************************************************************************/

uint32_t SAI_GetInstance(I2S_Type *base)
{
    uint32_t instance = 0;
    for (instance = 0; instance < ARRAY_SIZE(s_saiBases); instance++)
    {
        if (s_saiBases[instance] == base)
            break;
    }
    assert(instance < ARRAY_SIZE(s_saiBases));
    return instance;
}

void SAI_Init(I2S_Type *base, PinName sdi, PinName ws, PinName sck)
{
    pinmap_pinout(sck, PinMap_I2S_SCK);
    pinmap_pinout(ws, PinMap_I2S_WS);
    pinmap_pinout(sdi, PinMap_I2S_SDI);
    /* Pin config in case I2S1 is used with alternate function 3. */
    if(sdi == I2S1_SDI_B13)
    {
        pinmap_pinout(HYPERBUS_CLK, PinMap_HYPERBUS_CLK);
    }

    /* UDMA I2S device on */
    UDMA_Init((UDMA_Type *)base);
}

void SAI_Deinit(I2S_Type *base)
{
    /* UDMA I2S device off */
    UDMA_Deinit((UDMA_Type *)base);
}

void SAI_ExternalBitsWord(I2S_Type *base, uint8_t wordLength)
{
    base->EXT = I2S_EXT_BITS_WORD(wordLength - 1);
}

void SAI_ClockConfig(I2S_Type *base, char ch_id, uint8_t wordLength, uint8_t en, uint16_t div)
{
    if(ch_id)
        base->CFG_CLKGEN1 = I2S_CFG_CLKGEN1_BITS_WORD(wordLength - 1) |
                            I2S_CFG_CLKGEN1_CLK_EN(en) |
                            I2S_CFG_CLKGEN1_CLK_DIV(div);
    else
        base->CFG_CLKGEN0 = I2S_CFG_CLKGEN0_BITS_WORD(wordLength - 1) |
                            I2S_CFG_CLKGEN0_CLK_EN(en) |
                            I2S_CFG_CLKGEN0_CLK_DIV(div);
}

void SAI_ModeConfig(I2S_Type *base, uint8_t ch_id, uint8_t lsb_first, uint8_t pdm_filt_en,
                    uint8_t pdm_en, uint8_t use_ddr, sai_clk_mode_t clk_mode)
{
    if(ch_id)
        base->CHMODE |= I2S_CHMODE_CH1_SNAP_CAM(0) |
                        I2S_CHMODE_CH1_LSB_FIRST(lsb_first) |
                        I2S_CHMODE_CH1_PDM_USEFILTER(pdm_filt_en) |
                        I2S_CHMODE_CH1_PDM_EN(pdm_en) |
                        I2S_CHMODE_CH1_USEDDR(use_ddr) |
                        I2S_CHMODE_CH1_MODE(clk_mode);
    else
        base->CHMODE |= I2S_CHMODE_CH0_SNAP_CAM(0) |
                        I2S_CHMODE_CH0_LSB_FIRST(lsb_first) |
                        I2S_CHMODE_CH0_PDM_USEFILTER(pdm_filt_en) |
                        I2S_CHMODE_CH0_PDM_EN(pdm_en) |
                        I2S_CHMODE_CH0_USEDDR(use_ddr) |
                        I2S_CHMODE_CH0_MODE(clk_mode);
}

void SAI_FilterConfig(I2S_Type *base, char ch_id, uint16_t decimation, uint16_t shift)
{
    if(ch_id)
        base->FILT_CH1 = I2S_FILT_CH1_DECIMATION(decimation) | I2S_FILT_CH1_SHIFT(shift);
    else
        base->FILT_CH0 = I2S_FILT_CH0_DECIMATION(decimation) | I2S_FILT_CH0_SHIFT(shift);
}

status_t SAI_TransferReceiveBlocking(I2S_Type *base, sai_transfer_t *xfer)
{
    udma_req_info_t info;

    info.dataAddr    = (uint32_t) xfer->rxData;
    info.dataSize    = (uint32_t) xfer->rxDataSize;
    info.isTx        = xfer->channel;
    info.configFlags = UDMA_CFG_DATA_SIZE(xfer->configFlags) | UDMA_CFG_EN(1);

    return UDMA_BlockTransfer((UDMA_Type *)base, &info, UDMA_WAIT);
}

status_t SAI_TransferReceiveNonBlocking(I2S_Type *base, sai_transfer_t *xfer, sai_handle_t *handle)
{
    status_t status = uStatus_NoTransferInProgress;
    handle->state = uSAI_Busy;

    udma_req_t *udma_req = UDMA_FindAvailableRequest();
    udma_req->info.dataAddr    = (uint32_t) xfer->rxData;
    udma_req->info.dataSize    = (uint32_t) xfer->rxDataSize;
    udma_req->info.isTx        = xfer->channel;
    udma_req->info.channelId   = UDMA_EVENT_SAI_CH0 + xfer->channel;
    udma_req->info.task        = (uint32_t)handle;
    udma_req->info.configFlags = UDMA_CFG_DATA_SIZE(xfer->configFlags) | UDMA_CFG_EN(1);
    udma_req->info.ctrl        = UDMA_CTRL_NORMAL;
    udma_req->info.repeat.size = 0;

    status = UDMA_SendRequest((UDMA_Type *)base, udma_req);

    return ((status == 1) ? uStatus_Success : uStatus_SAI_Error);
}

void SAI_CreateHandler(I2S_Type *base, sai_handle_t *handle,
                       sai_transfer_callback_t callback, void *userData)
{
    assert(handle);

    handle->state    = uSAI_Idle;
    handle->callback = callback;
    handle->userData = userData;
}

void SAI_IRQHandler(void *arg)
{
    sai_handle_t *handle = (sai_handle_t *) arg;
    handle->state = uSAI_Idle;
    if(handle->callback != NULL)
        handle->callback(handle->userData);
}
