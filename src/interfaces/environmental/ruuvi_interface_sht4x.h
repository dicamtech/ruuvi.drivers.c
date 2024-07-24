#ifndef RUUVI_INTERFACE_SHT4XEXT_H
#define RUUVI_INTERFACE_SHT4XEXT_H
#include "ruuvi_driver_error.h"
#include "ruuvi_driver_sensor.h"

/**
 * @addtogroup Environmental
 */
/*@{*/
/**
 * @defgroup SHT4XEXT SHT4XEXT Interface
 * @brief Implement @ref rd_sensor_t functions on SHT4XEXT
 *
 * The implementation supports taking single-samples and a pseudo-continuous mode
 * by taking a new sample when data is polled in continuous mode
 */
/*@}*/
/**
 * @addtogroup SHT4XEXT
 */
/*@{*/
/**
 * @file ruuvi_interface_SHT4XEXT.h
 * @author Ibrahim Hroob <ibrahim.hroub7@gmail.com>
 * @date 2024-07-13
 * @copyright Dicam Technology Ltd, license BSD-3-Clause.
 *
 * Interface for SHT4XEXT basic usage. The underlying platform must provide
 * functions for I2C access.
 *
 * Testing the interface with @ref ruuvi_driver_sensor_test.h
 *
 * @code{.c}
 *  RD_ERROR_CHECK(err_code, RD_ERROR_SELFTEST);
 *  rd_status_t err_code = RD_SUCCESS;
 *  rd_bus_t bus = RD_BUS_NONE;
 *  uint8_t handle = 0;
 *  rd_sensor_init_fp init = ri_SHT4XEXT_init;
 *  bus = RD_BUS_I2C;
 *  handle = RUUVI_BOARD_RUUVI_BOARD_SHT4XEXT_I2C_ADDRESS;
 *  err_code = test_sensor_init(init, bus, handle);
 *  err_code = test_sensor_setup(init, bus, handle);
 *  err_code = test_sensor_modes(init, bus, handle);
 *  RD_ERROR_CHECK(err_code, RD_ERROR_SELFTEST);
 * @endcode
 */

// Maximum power up time is 1000us, please check section 3.1 Timings in https://sensirion.com/media/documents/33FD6951/662A593A/HT_DS_Datasheet_SHT4XEXT.pdf
#define RI_SHT4XEXT_WAKEUP_US (1000U) //!< Time from wakeup cmd to rdy.

/** @brief @ref rd_sensor_init_fp */
rd_status_t ri_SHT4XEXT_init (rd_sensor_t *
                           environmental_sensor, rd_bus_t bus, uint8_t handle);
/** @brief @ref rd_sensor_init_fp */
rd_status_t ri_SHT4XEXT_uninit (rd_sensor_t *
                             environmental_sensor, rd_bus_t bus, uint8_t handle);
/** @brief @ref rd_sensor_setup_fp */
rd_status_t ri_SHT4XEXT_samplerate_set (uint8_t * samplerate);
/** @brief @ref rd_sensor_setup_fp */
rd_status_t ri_SHT4XEXT_samplerate_get (uint8_t * samplerate);
/** @brief @ref rd_sensor_setup_fp */
rd_status_t ri_SHT4XEXT_resolution_set (uint8_t * resolution);
/** @brief @ref rd_sensor_setup_fp */
rd_status_t ri_SHT4XEXT_resolution_get (uint8_t * resolution);
/** @brief @ref rd_sensor_setup_fp */
rd_status_t ri_SHT4XEXT_scale_set (uint8_t * scale);
/** @brief @ref rd_sensor_setup_fp */
rd_status_t ri_SHT4XEXT_scale_get (uint8_t * scale);
/** @brief @ref rd_sensor_setup_fp */
rd_status_t ri_SHT4XEXT_dsp_set (uint8_t * dsp, uint8_t * parameter);
rd_status_t ri_SHT4XEXT_dsp_get (uint8_t * dsp, uint8_t * parameter);
/** @brief @ref rd_sensor_setup_fp */
rd_status_t ri_SHT4XEXT_mode_set (uint8_t *);
/** @brief @ref rd_sensor_setup_fp */
rd_status_t ri_SHT4XEXT_mode_get (uint8_t *);
/** @brief @ref rd_sensor_data_fp */
rd_status_t ri_SHT4XEXT_data_get (rd_sensor_data_t * const
                               p_data);
/*@}*/
#endif