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

#ifndef _GAP_SAI_H_
#define _GAP_SAI_H_

#include "gap_common.h"
#include "gap_udma.h"
#include "pinmap.h"

/*!
 * @addtogroup SAI
 * @{
 */

/*******************************************************************************
 * Variables, macros, structures,... definitions
 ******************************************************************************/

/*! @brief SAI module status. */
enum _sai_status_t
{
    uStatus_SAI_Idle  = MAKE_STATUS(uStatusGroup_SAI, 0), /*!< SAI module idle. */
    uStatus_SAI_Busy  = MAKE_STATUS(uStatusGroup_SAI, 1), /*!< SAI module busy with a transfer. */
    uStatus_SAI_Error = MAKE_STATUS(uStatusGroup_SAI, 2)  /*!< Error received during transfer. */
};

/*! @brief SAI transfer state. */
enum _sai_transfer_status
{
    uSAI_Idle = 0x0,            /*!< Nothing in the receiver. */
    uSAI_Busy,                  /*!< Transfer queue not finished. */
    uSAI_Error                  /*!< Transfer error. */
};

/*! @brief SAI channel clock config. */
typedef enum _sai_clk_mode
{
    uSAI_CLK0_INT_WS = 0,       /*!< Clock | WS : Clock Generator 0. */
    uSAI_CLK1_INT_WS,           /*!< Clock | WS : Clock Generator 1. */
    uSAI_EXT_CLK_INT_WS,        /*!< Clock : External clock | WS : Clock Generator 0. */
    uSAI_EXT_CLK_EXT_WS         /*!< Clock | WS : External clock. */
} sai_clk_mode_t;

/*! @brief SAI Chip Select(csn) configuration. */
typedef enum _sai_chan
{
    uSAI_Channel0 = 0U, /*!< Module I2S0. */
    uSAI_Channel1 = 1U  /*!< Module I2S1. */
} sai_chan_t;

/*! @brief Audio sample rate. */
typedef enum _sai_sample_rate
{
    uSAI_Freq_8   = 8000,
    uSAI_Freq_11  = 11025,
    uSAI_Freq_16  = 16000,
    uSAI_Freq_22  = 22050,
    uSAI_Freq_32  = 32000,
    uSAI_Freq_44  = 44100,
    uSAI_Freq_48  = 48000,
    uSAI_Freq_96  = 96000,
    uSAI_Freq_192 = 192000
} sai_sample_rate_t;

/*! @brief SAI clock generator word width. */
typedef enum _sai_word_width
{
    uSAI_Word_8  = 0, /*!< 8 bits. */
    uSAI_Word_16 = 1, /*!< 16 bits. */
    uSAI_Word_32 = 2  /*!< 32 bits. */
} sai_word_width_t;

/*! @brief SAI transfer structure. */
typedef struct _sai_transfer
{
    uint8_t         *rxData;      /*!< Receive buffer. */
    volatile size_t  rxDataSize;  /*!< RX Transfer bytes. */
    uint8_t          configFlags; /*!< Transfer configuration flags. */
    sai_chan_t       channel;     /*!< SAI channel used for the transfer. */
} sai_transfer_t;

/*!
 * @brief Completion callback function pointer type.
 *
 * When an asynchronous transfer is completed, the handler calls this callback function.
 *
 * @param userData  Parameter passed to the callback function by the user.
 */
typedef void (*sai_transfer_callback_t)(void *userData);

/*!
 * @brief SAI handler structure.
 *
 * This structure holds information to handle events from UDMA upon asynchronous transfers completion.
 * When asynchronous transfers are used, this structure should be filled.
 */
typedef struct _sai_handle_t
{
    sai_transfer_callback_t  callback; /*!< A callback function called when the transfer is finished. */
    void                    *userData; /*!< A callback parameter passed to the callback function. */
    sai_transfer_t           transfer; /*!< SAI transfer. */
    uint8_t                  state;    /*!< A transfer state maintained during transfer. */
} sai_handle_t;


/*******************************************************************************
 * APIs
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*__cplusplus*/

/*!
 * @name SAI module configuration.
 * @{
 */

/*!
 * @brief Get instance number of a SAI module.
 *
 * @param base           SAI base pointer.
 *
 * @return Instance number of the SAI module.
 */
uint32_t SAI_GetInstance(I2S_Type *base);

/*!
 * @brief Initialize a SAI module.
 *
 * This function initializes a SAI module.
 *
 * @param base           SAI base pointer.
 * @param sdi            I2S serial data pin number.
 * @param ws             I2S word select pin number.
 * @param sck            I2S serial clock pin number.
 */
void SAI_Init(I2S_Type *base, PinName sdi, PinName ws, PinName sck);

/*!
 * @brief Release a SAI module.
 *
 * @param base           SAI base pointer.
 */
void SAI_Deinit(I2S_Type *base);

/*!
 * @brief Configure SAI module for the external clock.
 *
 * This function configures the I2S module in case the external clock is used
 * instead of the internal clocks.
 * When the external clock and internal WS are used, the wordLength parameter indicates
 * after how many bits the WS is toggled. The value is (num bits – 1).
 *
 * @param base           SAI base pointer.
 * @param wordLength     External clock word length.
 */
void SAI_ExternalBitsWordConfig(I2S_Type *base, uint8_t wordLength);

/*!
 * @brief Configure SAI module's internal clock.
 *
 * This function configures the internal clock of the I2S module.
 * This clock is used to generate the Word Select(WS) signal.
 * It can also be used as the clock by a channel instead of an external clock.
 *
 * @param base           SAI base pointer.
 * @param ch_id          ID of the channel to configure.
 * @param wordLength     Internal clock word length.
 * @param div            Clock divider.
 * @param en             Clock enable.
 */
void SAI_ClockConfig(I2S_Type *base, char ch_id, uint8_t wordLength, uint8_t en, uint16_t div);

/*!
 * @brief Configure SAI module's channel mode.
 *
 * This function configures the channel mode of the I2S module.
 *
 * @param base           SAI base pointer.
 * @param ch_id          ID of the channel to configure.
 * @param lsb_first      Word serialization : LSB or MSB.
 * @param pdm_filt_en    Enable PDM Filter.
 * @param pdm_en         Enable PDM.
 * @param use_ddr        Double data rate.
 * @param clk_mode       Channel clock modes(sai_clk_mode_t).
 */
void SAI_ModeConfig(I2S_Type *base, uint8_t ch_id, uint8_t lsb_first, uint8_t pdm_filt_en,
                    uint8_t pdm_en, uint8_t use_ddr, sai_clk_mode_t clk_mode);

/*!
 * @brief Configure SAI module filter.
 *
 * This function confiures the SAI module filter.
 * The channel ID is required to set one the two available channels of the I2S module.
 * The decimation and shift parameters are used to set the PDM filter.
 *
 * @param base           SAI base pointer.
 * @param ch_id          ID of the channel to configure.
 * @param decimation     PDM filter decimation value.
 * @param shift          PDM filter shift value.
 */
void SAI_FilterConfig(I2S_Type *base, char ch_id, uint16_t decimation, uint16_t shift);

/*! @} */

/*!
 * @name Synchronous operations(blocking functions).
 * @{
 */

/*!
 * @brief Blocking transfer from a SAI module.
 *
 * This function reads data from a SAI module into a buffer. This function uses blocking API.
 *
 * @param base           SAI base pointer.
 * @param xfer           Pointer to sai_transfer_t structure.
 *
 * @retval uStatus_Success if the operation is successful, an error otherwise.
 */
status_t SAI_TransferReceiveBlocking(I2S_Type *base, sai_transfer_t *xfer);

/*! @} */

/*!
 * @name Asynchronous operations(non blocking functions).
 * @{
 */

/*!
 * @brief Non blocking transfer from a SAI module.
 *
 * This function is used for non blocking transactions using UDMA.
 * Once the UDMA, called for the transfer operations, is configured, this function returns.
 *
 * @note Calling the API returns immediately after transfer initiates. When all
 * data is transferred, the callback function is called.
 *
 * @note This API returns immediately after the transfer initiates.
 *       Call the SAI_RxGetTransferStatusIRQ to poll the transfer status
 *       and check whether the transfer is finished.
 *       If the return status is not ustatus_SAI_Busy, the transfer is finished.
 *
 * @param                base SAI base pointer
 * @param xfer           Pointer to the sai_transfer_t structure.
 * @param handle         Pointer to the sai_handle_t structure.
 *
 * @retval ustatus_Success    Successfully started the data reception.
 * @retval uStatus_SAI_Error  Transfer error.
 */
status_t SAI_TransferReceiveNonBlocking(I2S_Type *base, sai_transfer_t *xfer, sai_handle_t *handle);

/*! @} */

/*!
 * @name IRQ Handler.
 * @{
 */

/*!
 * @brief Initializes the SAI Rx handle.
 *
 * This function initializes the Rx handle for the SAI Rx transactional APIs. Call
 * this function once to get the handle initialized.
 *
 * @param base           SAI base pointer.
 * @param handle         Pointer to sai_handle_t structure.
 * @param callback       Callback function.
 * @param userData       Parameter passed to the callback function.
 */
void SAI_CreateHandler(I2S_Type *base, sai_handle_t *handle,
                       sai_transfer_callback_t callback, void *userData);

/*!
 * @brief SAI IRQ handler.
 *
 * This function is called when a non blocking transfer is completed.
 * When called, the callback function previously defined is executed.
 *
 * @param arg            Pointer to sai_handle_t structure.
 *
 */
void SAI_IRQHandler(void *arg);

/*! @} */

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

/*! @} */

#endif /* _GAP_SAI_H_ */
