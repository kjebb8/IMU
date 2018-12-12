#include <math.h>

#include "mpu9250.h"

#include "nrf_delay.h"
#include "nrf_log.h"

static volatile bool m_new_imu_data = false;

static uint8_t m_mpu_address = MPU9250_ADDRESS_AD0;
// static uint8_t m_ak8963_address = AK8963_ADDRESS;
//
// static accel_scale_t m_accel_scale = AFS_2G;
// static gyro_scale_t m_gyro_scale = GFS_250DPS;
// static mag_scale_t m_mag_scale = MFS_16BITS;
// static mag_mode_t m_mag_mode = M_8HZ;
//
// static mpu_result_t m_result; //Yaw, Pitch and Roll
// static float temperature;   // Stores the real internal chip temperature in Celsius
//
// static uint32_t output_rate_deltat = 0; // Used to control display output rate
// static uint32_t count = 0, sum_count = 0; // used to control display output rate
//
// static float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
// static uint32_t last_update = 0, first_update = 0; // used to calculate integration interval
// static uint32_t now = 0;        // used to calculate integration interval
//
// static int16_t accel_raw[3];  // Stores the 16-bit signed accelerometer sensor output
// static int16_t gyro_raw[3];   // Stores the 16-bit signed gyro sensor output
// static int16_t mag_raw[3];    // Stores the 16-bit signed magnetometer sensor output
// static int16_t temp_raw;      // Temperature raw count output
//
// static float accel_res, gyro_res, mag_res; // Scale resolutions per LSB for the sensors
//
// static float accel_x, accel_y, accel_z, // Variables to hold latest sensor data values
//              gyro_x, gyro_y, gyro_z,
//              mag_x, mag_y, mag_z;
//
// static float factory_mag_calibration[3] = {0, 0, 0}; // Factory mag calibration and mag bias
//
// static float accel_bias[3] = {0, 0, 0}, // Bias corrections for gyro, accelerometer, and magnetometer
//              gyro_bias[3]  = {0, 0, 0},
//              mag_bias[3]   = {0, 0, 0}, //Used for mag calibration
//              mag_scale[3]  = {0, 0, 0}; //Used for mag calibration

static float self_test[6];

void mpu9250_init(void)
{

}

bool mpu_new_data_available(void)
{
    return m_new_imu_data;
}

uint8_t mpu_who_am_i(void)
{
    uint8_t who_am_i;
    read_register_twim(m_mpu_address, WHO_AM_I_MPU9250, &who_am_i, sizeof(who_am_i));
    wait_for_xfer();
    return who_am_i;
}

void mpu_self_test(void)
{
    uint8_t raw_data[6] = {0};
    uint8_t self_test_raw[6] = {0};
    int32_t a_avg[3] = {0}, g_avg[3] = {0}, a_st_avg[3] = {0}, g_st_avg[3] = {0};
    float factory_trim[6];
    uint8_t FS = GFS_250DPS;
    int num_samples = 200;

    write_register_byte(m_mpu_address, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
    write_register_byte(m_mpu_address, IMU_CONFIG, 0x02);    // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    write_register_byte(m_mpu_address, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
    write_register_byte(m_mpu_address, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    write_register_byte(m_mpu_address, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g
    wait_for_xfer();

    for( int ii = 0; ii < num_samples; ii++) // get average current values of gyro and acclerometer
    {
      read_register_twim(m_mpu_address, ACCEL_XOUT_H, &raw_data[0], 6);    // Read the six raw data registers into data array
      wait_for_xfer();
      a_avg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]) ;   // Turn the MSB and LSB into a signed 16-bit value
      a_avg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]) ;
      a_avg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]) ;

      read_register_twim(m_mpu_address, GYRO_XOUT_H, &raw_data[0], 6);    // Read the six raw data registers sequentially into data array
      wait_for_xfer();
      g_avg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
      g_avg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]) ;
      g_avg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]) ;
    }

    // Get average of num_samples values and store as average current readings
    for (int ii =0; ii < 3; ii++)
    {
      a_avg[ii] /= num_samples;
      g_avg[ii] /= num_samples;
    }

    // Configure the accelerometer for self-test
    // Enable self test on all three axes and set accelerometer range to +/- 2 g
    write_register_byte(m_mpu_address, ACCEL_CONFIG, 0xE0);
    // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    write_register_byte(m_mpu_address, GYRO_CONFIG,  0xE0);
    wait_for_xfer();
    nrf_delay_ms(25);

    // Get average self-test values of gyro and acclerometer
    for (int ii = 0; ii < num_samples; ii++)
    {
      // Read the six raw data registers into data array
      read_register_twim(m_mpu_address, ACCEL_XOUT_H, &raw_data[0], 6);
      wait_for_xfer();
      a_st_avg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
      a_st_avg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
      a_st_avg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

      // Read the six raw data registers sequentially into data array
      read_register_twim(m_mpu_address, GYRO_XOUT_H, &raw_data[0], 6);
      wait_for_xfer();
      g_st_avg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
      g_st_avg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
      g_st_avg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
    }

    // Get average of num_samples values and store as average self-test readings
    for (int ii =0; ii < 3; ii++)
    {
      a_st_avg[ii] /= num_samples;
      g_st_avg[ii] /= num_samples;
    }

    // Configure the gyro and accelerometer for normal operation
    write_register_byte(m_mpu_address, ACCEL_CONFIG, 0x00);
    write_register_byte(m_mpu_address, GYRO_CONFIG,  0x00);
    wait_for_xfer();
    nrf_delay_ms(25);

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    // X-axis accel self-test results
    read_register_twim(m_mpu_address, SELF_TEST_X_ACCEL, &self_test_raw[0], 3);
    read_register_twim(m_mpu_address, SELF_TEST_X_GYRO, &self_test_raw[3], 3);
    wait_for_xfer();

    // Retrieve factory self-test value from self-test code reads
    // FT[Xa] factory trim calculation
    factory_trim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)self_test_raw[0] - 1.0) ));
    // FT[Ya] factory trim calculation
    factory_trim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)self_test_raw[1] - 1.0) ));
    // FT[Za] factory trim calculation
    factory_trim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)self_test_raw[2] - 1.0) ));
    // FT[Xg] factory trim calculation
    factory_trim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)self_test_raw[3] - 1.0) ));
    // FT[Yg] factory trim calculation
    factory_trim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)self_test_raw[4] - 1.0) ));
    // FT[Zg] factory trim calculation
    factory_trim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)self_test_raw[5] - 1.0) ));

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
    // of the Self-Test Response
    // To get percent, must multiply by 100
    for (int i = 0; i < 3; i++)
    {
      // Report percent differences
      self_test[i] = 100.0 * ((float)(a_st_avg[i] - a_avg[i])) / factory_trim[i] - 100.;
      // Report percent differences
      self_test[i+3] = 100.0 * ((float)(g_st_avg[i] - g_avg[i])) / factory_trim[i+3] - 100.;
    }

    NRF_LOG_RAW_INFO("\r\nx-axis self test: acceleration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(self_test[0]));
    NRF_LOG_RAW_INFO("\r\ny-axis self test: acceleration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(self_test[1]));
    NRF_LOG_RAW_INFO("\r\nx-axis self test: acceleration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(self_test[2]));
    NRF_LOG_RAW_INFO("\r\nx-axis self test: gyration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(self_test[3]));
    NRF_LOG_RAW_INFO("\r\ny-axis self test: gyration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(self_test[4]));
    NRF_LOG_RAW_INFO("\r\nz-axis self test: gyration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(self_test[5]));
}
