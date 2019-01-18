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

#ifndef _GAP_I2C_H_
#define _GAP_I2C_H_

#include "gap_udma.h"

/*!
 * @addtogroup I2C
 * @{
 */

/*******************************************************************************
 * Variables, macros, structures,... definitions
 ******************************************************************************/

/*! @brief I2C module status. */
enum _i2c_status
{
    uStatus_I2C_Idle  = MAKE_STATUS(uStatusGroup_I2C, 0), /*!< I2C module idle. */
    uStatus_I2C_Busy  = MAKE_STATUS(uStatusGroup_I2C, 1), /*!< I2C module busy with a transfer. */
    uStatus_I2C_Error = MAKE_STATUS(uStatusGroup_I2C, 2)  /*!< I2C module error. */
};

/*! @brief I2C transfer state. */
enum _i2c_transfer_states
{
    uI2C_Idle = 0x0U,           /*!< Nothing in the transmitter/receiver. */
    uI2C_Busy,                  /*!< Transfer queue is not finished. */
    uI2C_Error                  /*!< Transfer error. */
};

/*! @brief I2C module instances. */
typedef enum _i2c_module
{
    uI2C_Module0 = 0, /*!< I2C0. */
    uI2C_Module1 = 1  /*!< I2C1. */
} i2c_module_t;

/*! @brief I2C configuration structure.*/
typedef struct _i2c_config
{
    uint32_t baudRate_Bps;     /*!< Baud rate configuration of I2C peripheral. */
} i2c_config_t;

/*!
 * @brief I2C transfer structure.
 *
 * This structure holds information on a transfer such as source buffer, size of the data,...
 */
typedef struct _i2c_transfer
{
    uint8_t         *txData;      /*!< Send buffer. */
    uint8_t         *rxData;      /*!< Receive buffer. */
    volatile size_t  txDataSize;  /*!< TX Transfer bytes. */
    volatile size_t  rxDataSize;  /*!< RX Transfer bytes. */
    uint32_t         configFlags; /*!< Transfer configuration flags. */
} i2c_transfer_t;

/*!
 * @brief Completion callback function pointer type.
 *
 * When an asynchronous transfer is completed, the handler calls this callback function.
 *
 * @param userData       Parameter passed to the callback function by the user.
 */
typedef void (*i2c_transfer_callback_t)(void *userData);

/*!
 * @brief I2C handler structure.
 *
 * This structure holds information to handle events from UDMA upon asynchronous transfers completion.
 * When asynchronous transfers are used, this structure should be filled.
 */
typedef struct _i2c_handle
{
    i2c_transfer_callback_t  callback;     /*!< A callback function called when the transfer is finished. */
    void                    *userData;     /*!< A callback parameter passed to the callback function. */
    i2c_transfer_t           transfer;     /*!< I2C transfer. */
    uint8_t                  state;        /*!< A transfer state maintained during transfer. */
    i2c_module_t             module;       /*!< Module used for the transfer. */
} i2c_handle_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @name I2C module configuration.
 * @{
 */

/*!
 * @brief Get instance number of a I2C module.
 *
 * @param base           I2C base pointer.
 *
 * @return Instance number of the I2C module.
 */
uint32_t I2C_GetInstance(I2C_Type *base);

/*!
 * @brief Initialize a I2C module.
 *
 * This function initializes a I2C module. This is an use case example.
 *  @code
 *   i2c_config_t  Config;
 *   Config.baudRate_Bps = 100000;
 *   I2C_Init(base, &Config, srcClock_Hz);
 *  @endcode
 *
 * @param base           I2C base pointer.
 * @param Config         Pointer to i2c_config_t structure.
 * @param srcClock_Hz    Module source input clock in Hertz.
 *
 * @return Return uStatus_Success if initialization is successful, an error otherwise.
 */
status_t I2C_Init(I2C_Type *base, const i2c_config_t *Config, uint32_t srcClock_Hz);

/*!
 * @brief Release a I2C module.
 *
 * Call this API to gate the I2C clock.
 * The I2C module can't work unless the I2C_Init() function is called.
 *
 * @param base           I2C base pointer.
 */
void I2C_Deinit(I2C_Type *base);

/*!
 * @brief Set the I2C configuration structure to default values.
 *
 * The purpose of this API is to get the configuration structure initialized for use in the I2C_Configure().
 * Use the initialized structure unchanged in the I2C_Configure() or modify
 * the structure before calling the I2C_Configure().
 * This is an example.
 * @code
 * i2c_config_t config;
 * I2C_GetDefaultConfig(&config);
 * @endcode
 *
 * @param Config         Pointer to the configuration structure.
*/
void I2C_GetDefaultConfig(i2c_config_t *Config);

/*!
 * @brief Set the I2C transfer baud rate.
 *
 * @param base           I2C base pointer.
 * @param baudRate_Bps   Baud rate value in bps.
 * @param srcClock_Hz    Source clock.
 *
 * @return Return uStatus_Success if configuration is successful, an error otherwise.
 */
status_t I2C_SetBaudRate(I2C_Type *base, uint32_t baudRate_Bps, uint32_t srcClock_Hz);

/*!
 * @brief Generate a start signal.
 *
 * @param base           I2C base pointer.
 *
 * @return Return uStatus_Success if the command is successfully sent, an error otherwise.
 */
status_t I2C_Start(I2C_Type *base);

/*!
 * @brief Generate a stop signal.
 *
 * @param base           I2C base pointer.
 *
 * @return Return uStatus_Success if the command is successfully sent, an error otherwise.
 */
status_t I2C_Stop(I2C_Type *base);

/*!
 * @brief Generate a wait signal.
 *
 * @param base           I2C base pointer.
 * @param ncycles        Number of cycles to wait.
 *
 * @return Return uStatus_Success if the command is successfully sent, an error otherwise.
 */
status_t I2C_Wait(I2C_Type *base, uint8_t ncycles);

/* @} */

/*!
 * @name Synchronous operations(blocking functions).
 * @{
 */

/*!
 * @brief Write a byte to a I2C slave peripheral.
 *
 * This function is used to send a byte of data to a I2C slave peripheral. It uses blocking API.
 *
 * @param base           I2C base pointer.
 * @param address        Address of the target.
 * @param data           Byte to write.
 * @param stop           Stop command at the end of the transaction.
 *
 * @return Return uStatus_Success if the write operation is successful, an error otherwise.
 */
status_t I2C_ByteWrite(I2C_Type *base, uint32_t address, uint8_t data, uint8_t stop);

/*!
 * @brief Read a byte from a I2C slave peripheral.
 *
 * This function is used to read a byte of data from a I2C slave peripheral. It uses blocking API.
 *
 * @param base           I2C base pointer.
 * @param address        Address of the target.
 * @param data           Buffer to copy data.
 * @param stop           Stop command at the end of the transaction.
 *
 * @return Return uStatus_Success if the read operation is successful, an error otherwise.
 */
status_t I2C_ByteRead(I2C_Type *base, uint32_t address, uint8_t *data, uint8_t stop);

/*!
 * @brief Write data to a I2C slave peripheral.
 *
 * This function writes data from a buffer to a I2C slave peripheral. This function uses blocking API.
 *
 * @param base           I2C base pointer.
 * @param address        Address of the target.
 * @param data           Pointer to the buffer to write.
 * @param length         Size of the buffer.
 * @param stop           Stop command at the end of the transaction.
 *
 * @return Return uStatus_Success if the write operation is successful, an error otherwise.
 */
status_t I2C_Write(I2C_Type *base, uint32_t address, const char *data, uint32_t length, uint32_t stop);

/*!
 * @brief Read data from a I2C slave peripheral.
 *
 * This function reads data from a I2C slave peripheral into a buffer. This function uses blocking API.
 *
 * @param base           I2C base pointer.
 * @param address        Address of the target.
 * @param data           Pointer to the buffer to copy data.
 * @param length         Size of the buffer.
 * @param stop           Stop command at the end of the transaction.
 *
 * @return Return uStatus_Success if the read operation is successful, an error otherwise.
 */
status_t I2C_Read(I2C_Type *base, uint32_t address, char *data, uint32_t length, uint32_t stop);

/*!
 * @brief Blocking transfer on the I2C bus.
 *
 * This function is used for blocking transactions. This function does not return until end of a transaction.
 *
 * @note The API does not return until the transfer succeeds or fails due
 * to arbitration lost or receiving a NAK.
 *
 * @param base           I2C base pointer.
 * @param tx_buffer      Udma tx buffer base address.
 * @param tx_length      Udma tx buffer size.
 * @param rx_buffer      Udma rx buffer base address.
 * @param rx_length      Udma rx buffer size.
 *
 * @return Return uStatus_Success if the transfer is successful, an error otherwise.
 */
static status_t I2C_TransferBlocking(I2C_Type *base, const void *tx_buffer, uint32_t tx_length,
                                     void *rx_buffer, uint32_t rx_length);

/* @} */

/*!
 * @name Asynchronous operations(non blocking functions).
 * @{
 */

/*!
 * @brief Write data to a I2C slave peripheral.
 *
 * This function writes data from a buffer to a I2C slave peripheral. This function uses non blocking API.
 *
 * @param base           I2C base pointer.
 * @param address        Address of the target.
 * @param data           Pointer to the buffer to write.
 * @param length         Size of the buffer.
 * @param stop           Stop command at the end of the transaction.
 * @param handle         Pointer to i2c_handle_t structure.
 *
 * @return Return uStatus_Success if the write operation is successful, an error otherwise.
 */
status_t I2C_Write_Async(I2C_Type *base, uint32_t address, const char *data,
                         uint32_t length, i2c_handle_t *handle);

/*!
 * @brief Read data from a I2C slave peripheral.
 *
 * This function reads data from a I2C slave peripheral into a buffer. This function uses non blocking API.
 *
 * @param base           I2C base pointer.
 * @param address        Address of the target.
 * @param data           Pointer to the buffer to copy data.
 * @param length         Size of the buffer.
 * @param stop           Stop command at the end of the transaction.
 * @param handle         Pointer to i2c_handle_t structure.
 *
 * @return Return uStatus_Success if the read operation is successful, an error otherwise.
 */
status_t I2C_Read_Async(I2C_Type *base, uint32_t address, char *data,
                        uint32_t length, i2c_handle_t *handle);

/*!
 * @brief Non blocking transfer on the I2C bus.
 *
 * This function is used for non blocking transactions using UDMA.
 * Once the UDMA, called for the transfer operations, is configured, this function returns.
 *
 * @note Calling the API returns immediately after transfer initiates. When all
 * data is transferred, the callback function is called.
 *
 * @param base           I2C base pointer.
 * @param handle         Pointer to i2c_handle_t structure.
 *
 * @retval uStatus_Success   Successfully started the data transmission.
 * @retval uStatus_I2C_Error Transfer error.
 */
static status_t I2C_TransferNonBlocking(I2C_Type *base, i2c_handle_t *handle);

/* @} */

/*!
 * @name IRQ Handler.
 * @{
 */

/*!
 * @brief Initialize the I2C IRQ handler.
 *
 * This function creates a IRQ handler for I2C non blocking operations.
 * The callback function passed to this function is called when transaction is done.
 *
 * @param base           I2C base pointer.
 * @param handle         Pointer to i2c_handle_t structure.
 * @param callback       Callback function.
 * @param userData       Parameter passed to the callback function.
 */
void I2C_CreateHandler(I2C_Type *base, i2c_handle_t *handle,
                       i2c_transfer_callback_t callback, void *userData);

/*!
 * @brief I2C IRQ handler.
 *
 * This function is called when a non blocking transfer is completed.
 * When called, the callback function previously defined is executed.
 *
 * @param arg            Callback function.
 */
void I2C_IRQ_Handler(void *arg);

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /*_GAP_I2C_H_*/
