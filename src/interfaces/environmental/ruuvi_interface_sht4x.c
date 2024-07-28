/**
 * @file ruuvi_interface_sht4x.c
 * @copyright Dicam technology Ltd.
 * @author Ibrahim Hroob <ibrahim.hroub7@gmail.com>
 * @date 2019-08-10
 *
 * SHT4x temperature and humidity sensor driver.
 */

#include "ruuvi_driver_enabled_modules.h"
#if RI_SHT4XEXT_ENABLED || DOXYGEN
// Ruuvi headers
#include "ruuvi_driver_error.h"
#include "ruuvi_driver_sensor.h"
#include "ruuvi_interface_environmental.h"
#include "ruuvi_interface_gpio.h"
#include "ruuvi_interface_i2c.h"
#include "ruuvi_interface_sht4x.h"
#include "ruuvi_interface_rtc.h"
#include "ruuvi_interface_yield.h"

#include <string.h>

/**
 * @addtogroup SHT4XEXT
 */
/** @{ */

// Sensirion driver.
#include "sht4x/sht4x.h"

#define LOW_POWER_SLEEP_MS_MIN (1000U)
#define SHT4XEXT_PROBE_RETRIES_MAX (5U)

#define ENTER_LOW_POWER_MODE 1
#define EXIT_LOW_POWER_MODE 0

static inline uint32_t US_TO_MS_ROUNDUP (uint32_t us)
{
    return (us / 1000) + 2;
}

/** @brief Macro for checking "ignored" parameters NO_CHANGE, MIN, MAX, DEFAULT */
#define RETURN_SUCCESS_ON_VALID(param) do {\
            if(RD_SENSOR_CFG_DEFAULT   == param ||\
               RD_SENSOR_CFG_MIN       == param ||\
               RD_SENSOR_CFG_MAX       == param ||\
               RD_SENSOR_CFG_NO_CHANGE == param   \
             ) return RD_SUCCESS;\
           } while(0)

/** @brief Macro for checking that sensor is in sleep mode before configuration */
#define VERIFY_SENSOR_SLEEPS() do { \
          uint8_t MACRO_MODE = 0; \
          ri_SHT4XEXT_mode_get(&MACRO_MODE); \
          if(RD_SENSOR_CFG_SLEEP != MACRO_MODE) { return RD_ERROR_INVALID_STATE; } \
          } while(0)

static uint64_t m_tsample;           //!< Timestamp of sample.
static bool m_autorefresh;           //!< Flag to refresh data on data_get.
static int32_t m_temperature;        //!< Last measured temperature.
static int32_t m_humidity;           //!< Last measured humidity.
static bool m_is_init;               //!< Flag, is sensor init.
static const char m_sensor_name[] = "SHT4X"; //!< Human-readable name of the sensor.

#define STATUS_OK 0                  //!< SHT4 driver ok
#define STATUS_ERR_BAD_DATA (-1)     //!< SHT4 driver data invald
#define STATUS_CRC_FAIL (-2)         //!< SHT4 driver CRC error
#define STATUS_UNKNOWN_DEVICE (-3)   //!< Invalid WHOAMI
#define STATUS_WAKEUP_FAILED (-4)    //!< Device didn't wake up
#define STATUS_SLEEP_FAILED (-5)     //!< Device didn't go to sleep

/**
 * @brief Convert error from SHT4XEXT driver to appropriate NRF ERROR
 *
 * @param[in] rslt error code from SHT4XEXT driver
 * @return    Ruuvi error code corresponding to SHT4XEXT error code
 */
static rd_status_t SHT4XEXT_TO_RUUVI_ERROR (const int16_t rslt)
{
    if (STATUS_OK == rslt)                 { return RD_SUCCESS; }

    rd_status_t err_code = RD_ERROR_INTERNAL;

    if (STATUS_UNKNOWN_DEVICE == rslt)     { err_code = RD_ERROR_NOT_FOUND; }
    else if (STATUS_ERR_BAD_DATA == rslt)  { err_code = RD_ERROR_INVALID_DATA; }
    else if (STATUS_CRC_FAIL == rslt)      { err_code = RD_ERROR_INVALID_DATA; }
    else if (STATUS_WAKEUP_FAILED == rslt) { err_code = RD_ERROR_INTERNAL; }
    else if (STATUS_SLEEP_FAILED == rslt)  { err_code = RD_ERROR_INTERNAL; }

    return err_code;
}

rd_status_t ri_SHT4XEXT_init (rd_sensor_t * sensor, rd_bus_t bus, uint8_t handle)
{
    rd_status_t err_code = RD_SUCCESS;

    if (NULL == sensor)
    {
        err_code |= RD_ERROR_NULL;
    }
    else if (rd_sensor_is_init (sensor))
    {
        err_code |= RD_ERROR_INVALID_STATE;
    }
    else
    {
        rd_sensor_initialize (sensor);
        sensor->name = m_sensor_name;
        uint8_t retries = 0;

        switch (bus)
        {
            case RD_BUS_I2C:
                do
                {
                    err_code = SHT4XEXT_TO_RUUVI_ERROR (sht4x_probe());
                    retries++;
                } while ( (RD_ERROR_INVALID_DATA == err_code)

                          && (retries < SHT4XEXT_PROBE_RETRIES_MAX));

                break;

            default:
                err_code |=  RD_ERROR_INVALID_PARAM;
        }

        if (RD_SUCCESS != err_code)
        {
            err_code = RD_ERROR_NOT_FOUND;
        }
        else
        {
            // Sensirion driver delays high-power mode time in any case.
            // Explicitly entering low-power mode has no effect.
            sht4x_enable_low_power_mode (EXIT_LOW_POWER_MODE);
            sensor->init              = ri_SHT4XEXT_init;
            sensor->uninit            = ri_SHT4XEXT_uninit;
            sensor->samplerate_set    = ri_SHT4XEXT_samplerate_set;
            sensor->samplerate_get    = ri_SHT4XEXT_samplerate_get;
            sensor->resolution_set    = ri_SHT4XEXT_resolution_set;
            sensor->resolution_get    = ri_SHT4XEXT_resolution_get;
            sensor->scale_set         = ri_SHT4XEXT_scale_set;
            sensor->scale_get         = ri_SHT4XEXT_scale_get;
            sensor->dsp_set           = ri_SHT4XEXT_dsp_set;
            sensor->dsp_get           = ri_SHT4XEXT_dsp_get;
            sensor->mode_set          = ri_SHT4XEXT_mode_set;
            sensor->mode_get          = ri_SHT4XEXT_mode_get;
            sensor->data_get          = ri_SHT4XEXT_data_get;
            sensor->configuration_set = rd_sensor_configuration_set;
            sensor->configuration_get = rd_sensor_configuration_get;
            sensor->provides.datas.temperature_c = 1;
            sensor->provides.datas.humidity_rh = 1;
            sht4x_enable_low_power_mode(ENTER_LOW_POWER_MODE);
            m_tsample = RD_UINT64_INVALID;
            m_is_init = true;
        }
    }

    return err_code;
}

rd_status_t ri_SHT4XEXT_uninit (rd_sensor_t * sensor,
                             rd_bus_t bus, uint8_t handle)
{
    if (NULL == sensor) { return RD_ERROR_NULL; }

    rd_status_t err_code = RD_SUCCESS;
    sht4x_enable_low_power_mode (ENTER_LOW_POWER_MODE);
    rd_sensor_uninitialize (sensor);
    m_tsample = RD_UINT64_INVALID;
    m_temperature = RD_INT32_INVALID;
    m_humidity = RD_INT32_INVALID;
    m_is_init = false;
    m_autorefresh = false;
    return err_code;
}

rd_status_t ri_SHT4XEXT_samplerate_set (uint8_t * samplerate)
{
    if (NULL == samplerate) { return RD_ERROR_NULL; }

    VERIFY_SENSOR_SLEEPS();
    rd_status_t err_code = RD_SUCCESS;

    if (RD_SENSOR_CFG_DEFAULT == *samplerate)  { *samplerate = RD_SENSOR_CFG_DEFAULT; }
    else if (RD_SENSOR_CFG_NO_CHANGE == *samplerate) { *samplerate = RD_SENSOR_CFG_DEFAULT; }
    else if (RD_SENSOR_CFG_MIN == *samplerate) { *samplerate = RD_SENSOR_CFG_DEFAULT; }
    else if (RD_SENSOR_CFG_MAX == *samplerate) {*samplerate = RD_SENSOR_CFG_DEFAULT; }
    else { *samplerate = RD_SENSOR_ERR_NOT_SUPPORTED; err_code |= RD_ERROR_NOT_SUPPORTED; }

    return err_code;
}

rd_status_t ri_SHT4XEXT_samplerate_get (uint8_t * samplerate)
{
    if (NULL == samplerate) { return RD_ERROR_NULL; }

    *samplerate = RD_SENSOR_CFG_DEFAULT;
    return RD_SUCCESS;
}

rd_status_t ri_SHT4XEXT_resolution_set (uint8_t * resolution)
{
    if (NULL == resolution) { return RD_ERROR_NULL; }

    VERIFY_SENSOR_SLEEPS();
    uint8_t original = *resolution;
    *resolution = RD_SENSOR_CFG_DEFAULT;
    RETURN_SUCCESS_ON_VALID (original);
    return RD_ERROR_NOT_SUPPORTED;
}

rd_status_t ri_SHT4XEXT_resolution_get (uint8_t * resolution)
{
    if (NULL == resolution) { return RD_ERROR_NULL; }

    *resolution = RD_SENSOR_CFG_DEFAULT;
    return RD_SUCCESS;
}

rd_status_t ri_SHT4XEXT_scale_set (uint8_t * scale)
{
    if (NULL == scale) { return RD_ERROR_NULL; }

    VERIFY_SENSOR_SLEEPS();
    uint8_t original = *scale;
    *scale = RD_SENSOR_CFG_DEFAULT;
    RETURN_SUCCESS_ON_VALID (original);
    return RD_ERROR_NOT_SUPPORTED;
}

rd_status_t ri_SHT4XEXT_scale_get (uint8_t * scale)
{
    if (NULL == scale) { return RD_ERROR_NULL; }

    *scale = RD_SENSOR_CFG_DEFAULT;
    return RD_SUCCESS;
}

rd_status_t ri_SHT4XEXT_dsp_set (uint8_t * dsp, uint8_t * parameter)
{
    if (NULL == dsp || NULL == parameter) { return RD_ERROR_NULL; }

    VERIFY_SENSOR_SLEEPS();

    // Validate configuration
    if ( (RD_SENSOR_CFG_DEFAULT  != *parameter
            && RD_SENSOR_CFG_MIN   != *parameter
            && RD_SENSOR_CFG_MAX   != *parameter) ||
            (RD_SENSOR_DSP_LAST  != *dsp))
    {
        return RD_ERROR_NOT_SUPPORTED;
    }

    return RD_SUCCESS;
}

rd_status_t ri_SHT4XEXT_dsp_get (uint8_t * dsp, uint8_t * parameter)
{
    if (NULL == dsp || NULL == parameter) { return RD_ERROR_NULL; }

    // Only default is available
    *dsp       = RD_SENSOR_CFG_DEFAULT;
    *parameter = RD_SENSOR_CFG_DEFAULT;
    return RD_SUCCESS;
}

// Start single on command, mark autorefresh with continuous
rd_status_t ri_SHT4XEXT_mode_set (uint8_t * mode)
{
    rd_status_t err_code = RD_SUCCESS;

    if (NULL == mode)
    {
        err_code |= RD_ERROR_NULL;
    }
    // Enter low power mode by default and by explicit low power mode commmand
    else if ( (RD_SENSOR_CFG_SLEEP == *mode) || (RD_SENSOR_CFG_DEFAULT == *mode))
    {
        m_autorefresh = false;
        *mode = RD_SENSOR_CFG_SLEEP;
        sht4x_enable_low_power_mode(1);
    }
    else if (RD_SENSOR_CFG_SINGLE == *mode)
    {
        // Do nothing if sensor is in continuous mode
        uint8_t current_mode;
        ri_SHT4XEXT_mode_get (&current_mode);

        if (RD_SENSOR_CFG_CONTINUOUS == current_mode)
        {
            *mode = RD_SENSOR_CFG_CONTINUOUS;
            return RD_ERROR_INVALID_STATE;
        }

        // Enter sleep after measurement
        m_autorefresh = false;
        *mode = RD_SENSOR_CFG_SLEEP;
        m_tsample = rd_sensor_timestamp_get();
        sht4x_enable_low_power_mode(0);
        sensirion_sleep_usec (RI_SHT4XEXT_WAKEUP_US);
        err_code |= SHT4XEXT_TO_RUUVI_ERROR (sht4x_measure_blocking_read (&m_temperature,
                                          &m_humidity));
        sht4x_enable_low_power_mode(1);
        return err_code;
    }
    else if (RD_SENSOR_CFG_CONTINUOUS == *mode)
    {
        m_autorefresh = true;
        err_code |= RD_SUCCESS;
    }
    else
    {
        err_code |= RD_ERROR_INVALID_PARAM;
    }

    return err_code;
}

rd_status_t ri_SHT4XEXT_mode_get (uint8_t * mode)
{
    if (NULL == mode) { return RD_ERROR_NULL; }

    if (m_autorefresh)
    {
        *mode = RD_SENSOR_CFG_CONTINUOUS;
    }

    if (!m_autorefresh)
    {
        *mode = RD_SENSOR_CFG_SLEEP;
    }

    return RD_SUCCESS;
}

rd_status_t ri_SHT4XEXT_data_get (rd_sensor_data_t * const p_data)
{
    rd_status_t err_code = RD_SUCCESS;

    if (NULL == p_data)
    {
        err_code |= RD_ERROR_NULL;
    }
    else
    {
        if (m_autorefresh)
        {
            // Set autorefresh to false to take a single sample
            m_autorefresh = false;
            uint8_t mode = RD_SENSOR_CFG_SINGLE;
            err_code |= ri_SHT4XEXT_mode_set (&mode);
            // Restore autorefresh
            m_autorefresh = true;
        }

        if ( (RD_SUCCESS == err_code) && (RD_UINT64_INVALID != m_tsample))
        {
            rd_sensor_data_t d_environmental;
            rd_sensor_data_fields_t env_fields = {.bitfield = 0};
            float env_values[2];
            env_values[0] = m_humidity / 1000.0f;
            env_values[1] = m_temperature / 1000.0f;
            env_fields.datas.humidity_rh = 1;
            env_fields.datas.temperature_c = 1;
            d_environmental.data = env_values;
            d_environmental.valid  = env_fields;
            d_environmental.fields = env_fields;
            d_environmental.timestamp_ms = m_tsample;
            rd_sensor_data_populate (p_data,
                                     &d_environmental,
                                     p_data->fields);
        }
    }

    return err_code;
}

/** @} */

#endif // RI_SHT4XEXT_ENABLED || DOXYGEN
