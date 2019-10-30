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

#ifndef __PI_DRIVERS_I2S_H__
#define __PI_DRIVERS_I2S_H__

#include "pmsis/pmsis_types.h"

/**
* @ingroup groupDrivers
*/


/**
 * @defgroup I2S I2S
 *
 * The I2S driver provides support for recording data from external microphones.
 */

/**
 * @addtogroup I2S
 * @{
 */

/**@{*/

#define PI_I2S_CHMODE_LSB_OFFSET        4
#define PI_I2S_CHMODE_PDM_FILTER_OFFSET 8
#define PI_I2S_CHMODE_PDM_ENA_OFFSET    12
#define PI_I2S_CHMODE_DDR_ENA_OFFSET    16
#define PI_I2S_CHMODE_WS_CLK_OFFSET     24

typedef enum
{
    PI_I2S_CHMODE_MSB                   = (0 << PI_I2S_CHMODE_LSB_OFFSET),
    PI_I2S_CHMODE_LSB                   = (1 << PI_I2S_CHMODE_LSB_OFFSET),
    PI_I2S_CHMODE_PDM_FILT_DIS          = (0 << PI_I2S_CHMODE_PDM_FILTER_OFFSET),
    PI_I2S_CHMODE_PDM_FILT_ENA          = (1 << PI_I2S_CHMODE_PDM_FILTER_OFFSET),
    PI_I2S_CHMODE_PDM_DIS               = (0 << PI_I2S_CHMODE_PDM_ENA_OFFSET),
    PI_I2S_CHMODE_PDM_ENA               = (1 << PI_I2S_CHMODE_PDM_ENA_OFFSET),
    PI_I2S_CHMODE_DDR_DIS               = (0 << PI_I2S_CHMODE_DDR_ENA_OFFSET),
    PI_I2S_CHMODE_DDR_ENA               = (1 << PI_I2S_CHMODE_DDR_ENA_OFFSET),
    PI_I2S_CHMODE_WS_CLK_CLKGEN_0       = (0 << PI_I2S_CHMODE_PDM_ENA_OFFSET),
    PI_I2S_CHMODE_WS_CLK_CLKGEN_1       = (1 << PI_I2S_CHMODE_PDM_ENA_OFFSET),
    PI_I2S_CHMODE_WS_CLK_EXT_CLK_INT_WS = (2 << PI_I2S_CHMODE_PDM_ENA_OFFSET),
    PI_I2S_CHMODE_WS_CLK_EXT_CLK_EXT_WS = (3 << PI_I2S_CHMODE_PDM_ENA_OFFSET)
} pi_i2s_chmode_e;

/**
 * \struct pi_i2s_conf_t
 * \brief I2S master configuration structure.
 *
 * This structure is used to pass the desired I2S configuration to the runtime
 * when opening a device.
 */
typedef struct pi_i2s_conf
{
    pi_device_e device;         /*!< Device type. */
    uint8_t i2s_id; /*!< Specifies on which I2C interface the device is
                      connected. */
    uint16_t conf;   /*!< Maximum baudrate for the I2C bitstream which
      can be used with the opened device . */
} pi_i2c_conf_t;

/** \brief Initialize an I2C configuration with default values.
 *
 * This function can be called to get default values for all parameters before
 * setting some of them. The structure containing the configuration must be
 * kept alive until the I2C device is opened.
 *
 * \param conf A pointer to the I2C configuration.
 */
void pi_i2c_conf_init(pi_i2c_conf_t *conf);

/** \brief Open an I2C device.
 *
 * This function must be called before the Hyperbus device can be used.
 * It will do all the needed configuration to make it usable and initialize
 * the handle used to refer to this opened device when calling other functions.
 *
 * \param device    A pointer to the device structure of the device to open.
 *   This structure is allocated by the called and must be kept alive until the
 *   device is closed.
 * \return          0 if the operation is successfull, -1 if there was an error.
 */
int pi_i2c_open(struct pi_device *device);

/** \brief Close an opened I2C device.
 *
 * This function can be called to close an opened I2C device once it is
 * not needed anymore, in order to free all allocated resources. Once this
 * function is called, the device is not accessible anymore and must be opened
 * again before being used.
 *
 * \param device    The device structure of the device to close.
 */
void pi_i2c_close (struct pi_device *device);

/** \brief Dynamically change the device configuration.
 *
 * This function can be called to change part of the device configuration after
 * it has been opened.
 *
 * \param device  A pointer to the structure describing the device.
 * \param cmd     The command which specifies which parameters of the driver to
 * modify and for some of them also their values. The command must be one of
 * those defined in pi_i2c_ioctl_e.
 * \param arg       An additional value which is required for some parameters
 * when they are set.
 */
void pi_i2c_ioctl(struct pi_device *device, uint32_t cmd, void *arg);

/** \brief Enqueue a burst read copy from the I2C (from I2C device to chip).
 *
 * This function can be used to read at least 1 byte of data from the I2C
 * device. The copy will make a synchronous transfer between the I2C and one of
 * the chip memory.
 * The caller is blocked until the transfer is finished.
 * Depending on the chip, there may be some restrictions on the memory which
 * can be used. Check the chip-specific documentation for more details.
 *
 * \param device  A pointer to the structure describing the device.
 * \param rx_buff The address in the chip where the received data must be
 *   written.
 * \param length  The size in bytes of the copy.
 * \param flags  Specify additional transfer behaviors like start and stop bits
 *   management.
 */
void pi_i2c_read(struct pi_device *device, uint8_t *rx_buff, int length,
  pi_i2c_xfer_flags_e flags);


/** \brief Enqueue an asynchronous burst read copy from the I2C (from I2C
 * device to chip).
 *
 * This function can be used to read at least 1 byte of data from the I2C
 * device.
 * The copy will make an asynchronous transfer between the I2C and one of the
 * chip memory.
 * A task must be specified in order to specify how the caller should be
 * notified when the transfer is finished.
 * Depending on the chip, there may be some restrictions on the memory which
 * can be used. Check the chip-specific documentation for more details.
 *
 * \param device  A pointer to the structure describing the device.
 * \param rx_buff  The address in the chip where the received data must be
 *   written.
 * \param length   The size in bytes of the copy.
 * \param flags  Specify additional transfer behaviors like start and stop
 *   bits management.
 * \param task        The task used to notify the end of transfer.
   See the documentation of pi_task_t for more details.
 */
void pi_i2c_read_async(struct pi_device *device, uint8_t *rx_buff, int length,
  pi_i2c_xfer_flags_e flags, pi_task_t *task);


//!@}

/**
 * @}
 */




#endif  /* __PI_DRIVERS_I2S_H__ */
