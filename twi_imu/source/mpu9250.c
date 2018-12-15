#include <math.h>

#include "mpu9250.h"
#include "mpu9250_support.h"
#include "twim_mpu.h"
#include "quaternion_filters.h"
#include "counter.h"

#include "nrf_delay.h"
#include "nrf_log.h"

#define M_PI 3.14159265358979323846
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

#define PRINT_VALUES

static volatile bool m_new_imu_data = false;

static uint8_t m_mpu_address = MPU9250_ADDRESS_AD0;
static uint8_t m_ak_address = AK8963_ADDRESS;

static accel_scale_t m_accel_scale = AFS_2G;
static gyro_scale_t m_gyro_scale = GFS_250DPS;
static mag_scale_t m_mag_scale = MFS_16BITS;
static mag_mode_t m_mag_mode = M_8HZ;

static float m_self_test[6];

static float m_accel_bias[3]       = {0, 0, 0}, // Bias corrections for gyro, accelerometer, and magnetometer
             m_gyro_bias[3]        = {0, 0, 0},
             m_mag_bias[3]         = {0, 0, 0}, //Used for mag calibration
             m_mag_scale_factor[3] = {0, 0, 0}; //Used for mag calibration

static float m_factory_mag_calibration[3] = {0, 0, 0}; // Factory mag calibration and mag bias

static float m_accel_res, m_gyro_res, m_mag_res; // Scale resolutions per LSB for the sensors

static mpu_result_t m_mpu_result; //Yaw, Pitch and Roll

static mpu_data_t m_accel_data, // Variables to hold latest sensor data values
                  m_gyro_data,
                  m_mag_data;

// static float m_temperature;   // Stores the real internal chip temperature in Celsius

static bool m_first_mag_data_acq = false;

static uint16_t m_counter_freq; //Hz

static float m_deltat = 0.0f;      // integration interval for both filter schemes
static uint32_t m_count_last = 0; // used to calculate integration interval
static uint32_t m_count_now = 0;    // used to calculate integration interval

#ifdef PRINT_VALUES
// Used to control display output rate
static uint32_t m_samples_since_print = 0;
static float m_time_since_print = 0.0f;
#endif

static void calculate_resolutions(void)
{
    switch (m_accel_scale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
        // 2-bit value:
        case AFS_2G:
            m_accel_res = 2.0f / 32768.0f;
            break;
        case AFS_4G:
            m_accel_res = 4.0f / 32768.0f;
            break;
        case AFS_8G:
            m_accel_res = 8.0f / 32768.0f;
            break;
        case AFS_16G:
            m_accel_res = 16.0f / 32768.0f;
            break;
    }

    switch (m_gyro_scale)
    {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
        // 2-bit value:
        case GFS_250DPS:
            m_gyro_res = 250.0f / 32768.0f;
            break;
        case GFS_500DPS:
            m_gyro_res = 500.0f / 32768.0f;
            break;
        case GFS_1000DPS:
            m_gyro_res = 1000.0f / 32768.0f;
            break;
        case GFS_2000DPS:
            m_gyro_res = 2000.0f / 32768.0f;
            break;
    }

    switch (m_mag_scale)
    {
        // Possible magnetometer scales (and their register bit settings) are:
        // 14 bit resolution (0) and 16 bit resolution (1)
        case MFS_14BITS:
            m_mag_res = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
            break;
        case MFS_16BITS:
            m_mag_res = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
            break;
    }
}

static void set_mag_bias(void)
{
    m_mag_bias[0] = 206.5;
    m_mag_bias[1] = -278;
    m_mag_bias[2] = 36.9;
}

static void set_mag_scale_factor(void)
{
    m_mag_scale_factor[0] = 0.985;
    m_mag_scale_factor[1] = 1.05;
    m_mag_scale_factor[2] = 0.945;
}

static void update_time(void)
{

    m_count_now = counter_get();

    // Set integration time by time elapsed since last filter update
    if(m_count_now >= m_count_last)
    {
        m_deltat = (float)(m_count_now - m_count_last) / m_counter_freq;
    }
    else //Counter overflow
    {
        m_deltat = (float)(0xFFFFFF - m_count_last + m_count_now) / m_counter_freq;
    }

    m_count_last = m_count_now;

#ifdef PRINT_VALUES
    m_time_since_print += m_deltat;
    m_samples_since_print++;
#endif
}

static void calculate_yaw_pitch_roll(void)
{
    const float * q = get_q();

    m_mpu_result.yaw   = RAD_TO_DEG * atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
                                            q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    m_mpu_result.pitch = RAD_TO_DEG * -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    m_mpu_result.roll  = RAD_TO_DEG * atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                                            q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
}

static void print_values(void)
{
    if(m_time_since_print > 0.5f)
    {
        NRF_LOG_RAW_INFO("\r\nYaw: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(m_mpu_result.yaw));
        NRF_LOG_RAW_INFO("\tPitch: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(m_mpu_result.pitch));
        NRF_LOG_RAW_INFO("\tRoll: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(m_mpu_result.roll));
        NRF_LOG_RAW_INFO("\tRate: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(m_samples_since_print / m_time_since_print));

        m_samples_since_print = 0;
        m_time_since_print = 0;
    }
}

uint8_t mpu_who_am_i(void)
{
    uint8_t who_am_i;
    twim_mpu_read_register(m_mpu_address, WHO_AM_I_MPU9250, &who_am_i, 1);
    return who_am_i;
}

uint8_t mpu_who_am_i_ak8963(void)
{
    uint8_t who_am_i;
    twim_mpu_read_register(m_ak_address, WHO_AM_I_AK8963, &who_am_i, 1);
    return who_am_i;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less
// deviation is a pass.
void mpu_self_test(void)
{
    uint8_t raw_data[6] = {0};
    uint8_t m_self_test_raw[6] = {0};
    int32_t a_avg[3] = {0}, g_avg[3] = {0}, a_st_avg[3] = {0}, g_st_avg[3] = {0};
    float factory_trim[6];
    uint8_t FS = GFS_250DPS;
    int num_samples = 200;

    twim_mpu_write_register_byte(m_mpu_address, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
    twim_mpu_write_register_byte(m_mpu_address, IMU_CONFIG, 0x02);    // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    twim_mpu_write_register_byte(m_mpu_address, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
    twim_mpu_write_register_byte(m_mpu_address, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    twim_mpu_write_register_byte(m_mpu_address, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

    for( int ii = 0; ii < num_samples; ii++) // get average current values of gyro and acclerometer
    {
        twim_mpu_read_register(m_mpu_address, ACCEL_XOUT_H, &raw_data[0], 6);    // Read the six raw data registers into data array
        a_avg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]) ;   // Turn the MSB and LSB into a signed 16-bit value
        a_avg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]) ;
        a_avg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]) ;

        twim_mpu_read_register(m_mpu_address, GYRO_XOUT_H, &raw_data[0], 6);    // Read the six raw data registers sequentially into data array
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
    twim_mpu_write_register_byte(m_mpu_address, ACCEL_CONFIG, 0xE0);
    // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    twim_mpu_write_register_byte(m_mpu_address, GYRO_CONFIG,  0xE0);
    nrf_delay_ms(25);

    // Get average self-test values of gyro and acclerometer
    for (int ii = 0; ii < num_samples; ii++)
    {
        // Read the six raw data registers into data array
        twim_mpu_read_register(m_mpu_address, ACCEL_XOUT_H, &raw_data[0], 6);
        a_st_avg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
        a_st_avg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        a_st_avg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

        // Read the six raw data registers sequentially into data array
        twim_mpu_read_register(m_mpu_address, GYRO_XOUT_H, &raw_data[0], 6);
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
    twim_mpu_write_register_byte(m_mpu_address, ACCEL_CONFIG, 0x00);
    twim_mpu_write_register_byte(m_mpu_address, GYRO_CONFIG,  0x00);
    nrf_delay_ms(25);

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    // X-axis accel self-test results
    twim_mpu_read_register(m_mpu_address, SELF_TEST_X_ACCEL, &m_self_test_raw[0], 3);
    twim_mpu_read_register(m_mpu_address, SELF_TEST_X_GYRO, &m_self_test_raw[3], 3);

    // Retrieve factory self-test value from self-test code reads
    // FT[Xa] factory trim calculation
    factory_trim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)m_self_test_raw[0] - 1.0) ));
    // FT[Ya] factory trim calculation
    factory_trim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)m_self_test_raw[1] - 1.0) ));
    // FT[Za] factory trim calculation
    factory_trim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)m_self_test_raw[2] - 1.0) ));
    // FT[Xg] factory trim calculation
    factory_trim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)m_self_test_raw[3] - 1.0) ));
    // FT[Yg] factory trim calculation
    factory_trim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)m_self_test_raw[4] - 1.0) ));
    // FT[Zg] factory trim calculation
    factory_trim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)m_self_test_raw[5] - 1.0) ));

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
    // of the Self-Test Response
    // To get percent, must multiply by 100
    for (int i = 0; i < 3; i++)
    {
        // Report percent differences
        m_self_test[i] = 100.0 * ((float)(a_st_avg[i] - a_avg[i])) / factory_trim[i] - 100.;
        // Report percent differences
        m_self_test[i+3] = 100.0 * ((float)(g_st_avg[i] - g_avg[i])) / factory_trim[i+3] - 100.;
    }

    NRF_LOG_RAW_INFO("x-axis self test: acceleration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(m_self_test[0]));
    NRF_LOG_RAW_INFO("\r\ny-axis self test: acceleration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(m_self_test[1]));
    NRF_LOG_RAW_INFO("\r\nx-axis self test: acceleration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(m_self_test[2]));
    NRF_LOG_RAW_INFO("\r\nx-axis self test: gyration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(m_self_test[3]));
    NRF_LOG_RAW_INFO("\r\ny-axis self test: gyration trim within: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(m_self_test[4]));
    NRF_LOG_RAW_INFO("\r\nz-axis self test: gyration trim within: " NRF_LOG_FLOAT_MARKER "%%\r\n", NRF_LOG_FLOAT(m_self_test[5]));
}

// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void mpu_calibrate(void)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias_raw[3]  = {0, 0, 0}, accel_bias_raw[3] = {0, 0, 0};

    // reset device
    // Write a one to bit 7 reset bit; toggle reset device
    twim_mpu_write_register_byte(m_mpu_address, PWR_MGMT_1, READ_FLAG);
    nrf_delay_ms(100);

    // get stable time source; Auto select clock source to be PLL gyroscope
    // reference if ready else use the internal oscillator, bits 2:0 = 001
    twim_mpu_write_register_byte(m_mpu_address, PWR_MGMT_1, 0x01);
    twim_mpu_write_register_byte(m_mpu_address, PWR_MGMT_2, 0x00);
    nrf_delay_ms(200);

    // Configure device for bias calculation
    // Disable all interrupts
    twim_mpu_write_register_byte(m_mpu_address, INT_ENABLE, 0x00);
    // Disable FIFO
    twim_mpu_write_register_byte(m_mpu_address, FIFO_EN, 0x00);
    // Turn on internal clock source
    twim_mpu_write_register_byte(m_mpu_address, PWR_MGMT_1, 0x00);
    // Disable I2C master
    twim_mpu_write_register_byte(m_mpu_address, I2C_MST_CTRL, 0x00);
    // Disable FIFO and I2C master modes
    twim_mpu_write_register_byte(m_mpu_address, USER_CTRL, 0x00);
    // Reset FIFO and DMP
    twim_mpu_write_register_byte(m_mpu_address, USER_CTRL, 0x0C);
    nrf_delay_ms(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    // Set low-pass filter to 188 Hz
    twim_mpu_write_register_byte(m_mpu_address, IMU_CONFIG, 0x01);
    // Set sample rate to 1 kHz
    twim_mpu_write_register_byte(m_mpu_address, SMPLRT_DIV, 0x00);
    // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    twim_mpu_write_register_byte(m_mpu_address, GYRO_CONFIG, 0x00);
    // Set accelerometer full-scale to 2 g, maximum sensitivity
    twim_mpu_write_register_byte(m_mpu_address, ACCEL_CONFIG, 0x00);

    uint16_t  gyro_sensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accel_sensitivity = 16384; // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    twim_mpu_write_register_byte(m_mpu_address, USER_CTRL, 0x40);  // Enable FIFO
    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
    // MPU-9150)
    twim_mpu_write_register_byte(m_mpu_address, FIFO_EN, 0x78);
    nrf_delay_ms(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    // Disable gyro and accelerometer sensors for FIFO
    twim_mpu_write_register_byte(m_mpu_address, FIFO_EN, 0x00);
    // Read FIFO sample count
    twim_mpu_read_register(m_mpu_address, FIFO_COUNTH, &data[0], 2);
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    // How many sets of full gyro and accelerometer data for averaging
    packet_count = fifo_count/12;

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        // Read data for averaging
        twim_mpu_read_register(m_mpu_address, FIFO_R_W, &data[0], 12);
        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

        // Sum individual signed 16-bit biases to get accumulated signed 32-bit
        // biases.
        accel_bias_raw[0] += (int32_t) accel_temp[0];
        accel_bias_raw[1] += (int32_t) accel_temp[1];
        accel_bias_raw[2] += (int32_t) accel_temp[2];
        gyro_bias_raw[0]  += (int32_t) gyro_temp[0];
        gyro_bias_raw[1]  += (int32_t) gyro_temp[1];
        gyro_bias_raw[2]  += (int32_t) gyro_temp[2];
    }
    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias_raw[0] /= (int32_t) packet_count;
    accel_bias_raw[1] /= (int32_t) packet_count;
    accel_bias_raw[2] /= (int32_t) packet_count;
    gyro_bias_raw[0]  /= (int32_t) packet_count;
    gyro_bias_raw[1]  /= (int32_t) packet_count;
    gyro_bias_raw[2]  /= (int32_t) packet_count;

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    if (accel_bias_raw[2] > 0L)
    {
        accel_bias_raw[2] -= (int32_t) accel_sensitivity;
    }
    else
    {
        accel_bias_raw[2] += (int32_t) accel_sensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers,
    // which are reset to zero upon device startup.
    // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
    // format.
    data[0] = (-gyro_bias_raw[0]/4  >> 8) & 0xFF;
    // Biases are additive, so change sign on calculated average gyro biases
    data[1] = (-gyro_bias_raw[0]/4)       & 0xFF;
    data[2] = (-gyro_bias_raw[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias_raw[1]/4)       & 0xFF;
    data[4] = (-gyro_bias_raw[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias_raw[2]/4)       & 0xFF;

    for(int i = 0; i < 6; i++)
    {
        NRF_LOG_RAW_INFO("gyro data[%d]: %d\r\n", i, data[i]);
    }

    // Push gyro biases to hardware registers
    twim_mpu_write_register_byte(m_mpu_address, XG_OFFSET_H, data[0]);
    twim_mpu_write_register_byte(m_mpu_address, XG_OFFSET_L, data[1]);
    twim_mpu_write_register_byte(m_mpu_address, YG_OFFSET_H, data[2]);
    twim_mpu_write_register_byte(m_mpu_address, YG_OFFSET_L, data[3]);
    twim_mpu_write_register_byte(m_mpu_address, ZG_OFFSET_H, data[4]);
    twim_mpu_write_register_byte(m_mpu_address, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    m_gyro_bias[0] = (float) gyro_bias_raw[0]/(float) gyro_sensitivity;
    m_gyro_bias[1] = (float) gyro_bias_raw[1]/(float) gyro_sensitivity;
    m_gyro_bias[2] = (float) gyro_bias_raw[2]/(float) gyro_sensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer
    // bias registers. These registers contain factory trim values which must be
    // added to the calculated accelerometer biases; on boot up these registers
    // will hold non-zero values. In addition, bit 0 of the lower byte must be
    // preserved since it is used for temperature compensation calculations.
    // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    // A place to hold the factory accelerometer trim biases
    int32_t m_accel_bias_reg[3] = {0, 0, 0};
    // Read factory accelerometer trim values
    twim_mpu_read_register(m_mpu_address, XA_OFFSET_H, &data[0], 2);
    m_accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    twim_mpu_read_register(m_mpu_address, YA_OFFSET_H, &data[0], 2);
    m_accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    twim_mpu_read_register(m_mpu_address, ZA_OFFSET_H, &data[0], 2);
    m_accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    // Define mask for temperature compensation bit 0 of lower byte of
    // accelerometer bias registers
    uint32_t mask = 1uL;
    // Define array to hold mask bit for each accelerometer bias axis
    uint8_t mask_bit[3] = {0, 0, 0};

    for (ii = 0; ii < 3; ii++)
    {
        // If temperature compensation bit is set, record that fact in mask_bit
        if ((m_accel_bias_reg[ii] & mask))
        {
            mask_bit[ii] = 0x01;
        }
    }

    // Construct total accelerometer bias, including calculated average
    // accelerometer bias from above
    // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
    // (16 g full scale)
    m_accel_bias_reg[0] -= (accel_bias_raw[0]/8);
    m_accel_bias_reg[1] -= (accel_bias_raw[1]/8);
    m_accel_bias_reg[2] -= (accel_bias_raw[2]/8);

    data[0] = (m_accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (m_accel_bias_reg[0])      & 0xFF;
    // preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[1] = data[1] | mask_bit[0];
    data[2] = (m_accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (m_accel_bias_reg[1])      & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[3] = data[3] | mask_bit[1];
    data[4] = (m_accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (m_accel_bias_reg[2])      & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[5] = data[5] | mask_bit[2];

    for(int i = 0; i < 6; i++)
    {
        NRF_LOG_RAW_INFO("accel data[%d]: %d\r\n", i, data[i]);
    }

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    twim_mpu_write_register_byte(m_mpu_address, XA_OFFSET_H, data[0]);
    twim_mpu_write_register_byte(m_mpu_address, XA_OFFSET_L, data[1]);
    twim_mpu_write_register_byte(m_mpu_address, YA_OFFSET_H, data[2]);
    twim_mpu_write_register_byte(m_mpu_address, YA_OFFSET_L, data[3]);
    twim_mpu_write_register_byte(m_mpu_address, ZA_OFFSET_H, data[4]);
    twim_mpu_write_register_byte(m_mpu_address, ZA_OFFSET_L, data[5]);

    // Output scaled accelerometer biases for display in the main program
    m_accel_bias[0] = (float)accel_bias_raw[0]/(float)accel_sensitivity;
    m_accel_bias[1] = (float)accel_bias_raw[1]/(float)accel_sensitivity;
    m_accel_bias[2] = (float)accel_bias_raw[2]/(float)accel_sensitivity;

    NRF_LOG_RAW_INFO("x-axis accel bias: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(m_accel_bias[0]));
    NRF_LOG_RAW_INFO("\r\ny-axis accel bias: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(m_accel_bias[1]));
    NRF_LOG_RAW_INFO("\r\nx-axis accel bias: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(m_accel_bias[2]));
    NRF_LOG_RAW_INFO("\r\nx-axis gyro bias: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(m_gyro_bias[0]));
    NRF_LOG_RAW_INFO("\r\ny-axis gyro bias: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(m_gyro_bias[1]));
    NRF_LOG_RAW_INFO("\r\nz-axis gyro bias: " NRF_LOG_FLOAT_MARKER "%%\r\n", NRF_LOG_FLOAT(m_gyro_bias[2]));
}

void mpu_init(void)
{
    // wake up device
    // Clear sleep mode bit (6), enable all sensors
    twim_mpu_write_register_byte(m_mpu_address, PWR_MGMT_1, 0x00);
    nrf_delay_ms(100); // Wait for all registers to reset

    // Get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    twim_mpu_write_register_byte(m_mpu_address, PWR_MGMT_1, 0x01);
    nrf_delay_ms(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
    // respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion
    // update rates cannot be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
    // 8 kHz, or 1 kHz
    twim_mpu_write_register_byte(m_mpu_address, IMU_CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG above.
    twim_mpu_write_register_byte(m_mpu_address, SMPLRT_DIV, 0x04);

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
    // left-shifted into positions 4:3

    // get current GYRO_CONFIG register value
    uint8_t c;
    twim_mpu_read_register(m_mpu_address, GYRO_CONFIG, &c, 1);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x03; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | m_gyro_scale << 3; // Set full scale range for the gyro
    // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
    // GYRO_CONFIG
    // c =| 0x00;
    // Write new GYRO_CONFIG value to register
    twim_mpu_write_register_byte(m_mpu_address, GYRO_CONFIG, c);

    // Set accelerometer full-scale range configuration
    // Get current ACCEL_CONFIG register value
    twim_mpu_read_register(m_mpu_address, ACCEL_CONFIG, &c, 1);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | m_accel_scale << 3; // Set full scale range for the accelerometer
    // Write new ACCEL_CONFIG register value
    twim_mpu_write_register_byte(m_mpu_address, ACCEL_CONFIG, c);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by
    // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
    // 1.13 kHz
    // Get current ACCEL_CONFIG2 register value
    twim_mpu_read_register(m_mpu_address, ACCEL_CONFIG2, &c, 1);
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    // Write new ACCEL_CONFIG2 register value
    twim_mpu_write_register_byte(m_mpu_address, ACCEL_CONFIG2, c);
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because
    // of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    // twim_mpu_write_register_byte(m_mpu_address, INT_PIN_CFG, 0x22);
    // twim_mpu_write_register_byte(m_mpu_address, INT_ENABLE, 0x00);
    twim_mpu_write_register_byte(m_mpu_address, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear
    twim_mpu_write_register_byte(m_mpu_address, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

    nrf_delay_ms(100);

    calculate_resolutions();

    m_counter_freq = 800;
    counter_init(m_counter_freq);
    counter_start();
}

void mpu_init_kj(void)
{
    // Clear sleep mode bit (6), enable all sensors
    twim_mpu_write_register_byte(m_mpu_address, PWR_MGMT_1, 0x00);
    nrf_delay_ms(100); // Wait for all registers to reset

    // Auto select clock source to be PLL gyroscope reference if ready else
    twim_mpu_write_register_byte(m_mpu_address, PWR_MGMT_1, 0x01);
    nrf_delay_ms(200);

    //10 Hz bandwidth, 17.85 ms delay, 1kHz internal sample rate
    twim_mpu_write_register_byte(m_mpu_address, IMU_CONFIG, 0x05);

    // Set sample rate = 50 Hz = gyroscope output rate/(1 + SMPLRT_DIV)
    twim_mpu_write_register_byte(m_mpu_address, SMPLRT_DIV, 0x13);

    uint8_t c;
    twim_mpu_read_register(m_mpu_address, GYRO_CONFIG, &c, 1);
    c = c & ~0x03; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | m_gyro_scale << 3; // Set full scale range for the gyro
    twim_mpu_write_register_byte(m_mpu_address, GYRO_CONFIG, c );

    twim_mpu_read_register(m_mpu_address, ACCEL_CONFIG, &c, 1);
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | m_accel_scale << 3; // Set full scale range for the accelerometer
    twim_mpu_write_register_byte(m_mpu_address, ACCEL_CONFIG, c);

    twim_mpu_read_register(m_mpu_address, ACCEL_CONFIG2, &c, 1);
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x05;  //10 Hz bandwidth, 35.7 ms delay, 1kHz internal sample rate
    twim_mpu_write_register_byte(m_mpu_address, ACCEL_CONFIG2, c);

    // twim_mpu_write_register_byte(m_mpu_address, INT_PIN_CFG, 0x22);
    // twim_mpu_write_register_byte(m_mpu_address, INT_ENABLE, 0x00);
    twim_mpu_write_register_byte(m_mpu_address, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear
    twim_mpu_write_register_byte(m_mpu_address, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

    nrf_delay_ms(100);

    calculate_resolutions();

    m_counter_freq = 200;
    counter_init(m_counter_freq);
    counter_start();
}

void mpu_init_ak8963(void)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t raw_data[3];  // x/y/z gyro calibration data stored here
    // TODO: Test this!! Likely doesn't work
    twim_mpu_write_register_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    nrf_delay_ms(10);

    twim_mpu_write_register_byte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    nrf_delay_ms(10);

    // Read the x-, y-, and z-axis calibration values
    twim_mpu_read_register(AK8963_ADDRESS, AK8963_ASAX, &raw_data[0], 3);

    // Return x-axis sensitivity adjustment values, etc.
    m_factory_mag_calibration[0] =  (float)(raw_data[0] - 128)/256. + 1.;
    m_factory_mag_calibration[1] =  (float)(raw_data[1] - 128)/256. + 1.;
    m_factory_mag_calibration[2] =  (float)(raw_data[2] - 128)/256. + 1.;

    twim_mpu_write_register_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    nrf_delay_ms(10);

    // Configure the magnetometer for continuous read and highest resolution.
    // Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
    // register, and enable continuous mode data acquisition Mmode (bits [3:0]),
    // 0010 for 8 Hz and 0110 for 100 Hz sample rates.

    // Set magnetometer data resolution and sample ODR
    twim_mpu_write_register_byte(AK8963_ADDRESS, AK8963_CNTL, m_mag_scale << 4 | m_mag_mode);

    set_mag_bias();
    set_mag_scale_factor();

    nrf_delay_ms(500);

    for (int i = 0; i < 3; i++)
    {
        NRF_LOG_RAW_INFO("fact mag cal: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(m_factory_mag_calibration[i]));
    }
}

bool mpu_new_data_int(void)
{
    return m_new_imu_data;
}

bool mpu_new_data_poll(void)
{
    uint8_t data_status;
    twim_mpu_read_register(m_mpu_address, INT_STATUS, &data_status, 1);
    return (data_status & 0x01);
}

const mpu_data_t * mpu_read_accel_data(void)
{
    uint8_t raw_data[6];  // x/y/z accel register data stored here
    int16_t volt_data[3];
    // Read the six raw data registers into data array
    twim_mpu_read_register(m_mpu_address, ACCEL_XOUT_H, &raw_data[0], 6);

    // Turn the MSB and LSB into a signed 16-bit value
    volt_data[0] = ((int16_t)raw_data[0] << 8) | raw_data[1];
    volt_data[1] = ((int16_t)raw_data[2] << 8) | raw_data[3];
    volt_data[2] = ((int16_t)raw_data[4] << 8) | raw_data[5];

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    m_accel_data.type = ACCEL,
    m_accel_data.x    = (float)volt_data[0] * m_accel_res - m_accel_bias[0];
    m_accel_data.y    = (float)volt_data[1] * m_accel_res - m_accel_bias[1];
    m_accel_data.z    = (float)volt_data[2] * m_accel_res - m_accel_bias[2];

    return &m_accel_data;
}

const mpu_data_t * mpu_read_gyro_data(void)
{
    uint8_t raw_data[6];  // x/y/z gyro register data stored here
    int16_t volt_data[3];
    // Read the six raw data registers into data array
    twim_mpu_read_register(m_mpu_address, GYRO_XOUT_H, &raw_data[0], 6);

    // Turn the MSB and LSB into a signed 16-bit value
    volt_data[0] = ((int16_t)raw_data[0] << 8) | raw_data[1];
    volt_data[1] = ((int16_t)raw_data[2] << 8) | raw_data[3];
    volt_data[2] = ((int16_t)raw_data[4] << 8) | raw_data[5];

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    m_gyro_data.type = GYRO,
    m_gyro_data.x    = (float)volt_data[0] * m_gyro_res - m_gyro_bias[0];
    m_gyro_data.y    = (float)volt_data[1] * m_gyro_res - m_gyro_bias[1];
    m_gyro_data.z    = (float)volt_data[2] * m_gyro_res - m_gyro_bias[2];

    return &m_gyro_data;
}

const mpu_data_t * mpu_read_mag_data(void)
{
    do
    {
        // Wait for magnetometer data ready bit to be set
        uint8_t st1;
        twim_mpu_read_register(AK8963_ADDRESS, AK8963_ST1, &st1, 1);
        if (st1 & 0x01)
        {
            // x/y/z mag register data, ST2 register stored here, must read ST2 at end
            // of data acquisition
            uint8_t raw_data[7];

            // Read the six raw data and ST2 registers sequentially into data array
            twim_mpu_read_register(AK8963_ADDRESS, AK8963_XOUT_L, &raw_data[0], 7);
            uint8_t st2 = raw_data[6]; // End data read by reading ST2 register

            // Check if magnetic sensor overflow set, if not then report data
            if (!(st2 & 0x08))
            {
                int16_t volt_data[3];
                // Turn the MSB and LSB into a signed 16-bit value
                // Data stored as little Endian
                volt_data[0] = ((int16_t)raw_data[1] << 8) | raw_data[0];
                volt_data[1] = ((int16_t)raw_data[3] << 8) | raw_data[2];
                volt_data[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];

                // Calculate the magnetometer values in milliGauss
                // Include factory calibration per data sheet and user environmental
                // corrections
                // Get actual magnetometer value, this depends on scale being set
                m_mag_data.type = MAG,
                m_mag_data.x = ( (float)volt_data[0] * m_mag_res * m_factory_mag_calibration[0] - m_mag_bias[0] ) * m_mag_scale_factor[0];
                m_mag_data.y = ( (float)volt_data[1] * m_mag_res * m_factory_mag_calibration[1] - m_mag_bias[1] ) * m_mag_scale_factor[1];
                m_mag_data.z = ( (float)volt_data[2] * m_mag_res * m_factory_mag_calibration[2] - m_mag_bias[2] ) * m_mag_scale_factor[2];

                m_first_mag_data_acq = true;
            }
        }
    } while(!m_first_mag_data_acq);

    return &m_mag_data;
}

void mpu_read_new_data(void)
{
    mpu_read_accel_data();
    mpu_read_gyro_data();
    mpu_read_mag_data();
}

void mpu_calculate_orientation(void)
{
    update_time();
    mahony_quaternion_update(m_accel_data.x, m_accel_data.y, m_accel_data.z,
                             m_gyro_data.x * DEG_TO_RAD, m_gyro_data.y * DEG_TO_RAD, m_gyro_data.z * DEG_TO_RAD,
                             m_mag_data.y, m_mag_data.x, -m_mag_data.z, m_deltat);
    calculate_yaw_pitch_roll();

#ifdef PRINT_VALUES
    print_values();
#endif
}

const mpu_result_t * mpu_get_current_orientation(void)
{
    return &m_mpu_result;
}
