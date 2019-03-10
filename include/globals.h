
#include <Arduino.h>
#include <Wire.h>

extern TwoWire HWire; 

//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer

extern uint8_t gyro_address;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
extern uint8_t baro_address;               //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
extern uint8_t compass_address;            //The I2C address of the HMC5883L is 0x1E in hexadecimal form.

extern uint8_t disable_throttle, flip32;
extern uint8_t error;
extern uint32_t loop_timer;
extern float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
extern float battery_voltage;
extern int16_t loop_counter;
extern uint8_t data, start, warning;
extern int16_t acc_axis[4], gyro_axis[4], temperature;
extern int32_t gyro_axis_cal[4], acc_axis_cal[4];
extern int32_t cal_int;
extern int32_t channel_1_start, channel_1;
extern int32_t channel_2_start, channel_2;
extern int32_t channel_3_start, channel_3;
extern int32_t channel_4_start, channel_4;
extern int32_t channel_5_start, channel_5;
extern int32_t channel_6_start, channel_6;
extern int32_t measured_time, measured_time_start;
extern uint8_t channel_select_counter;

//Barometer variables.
extern uint16_t C[7];
extern uint8_t barometer_counter, temperature_counter;
extern int64_t OFF, OFF_C2, SENS, SENS_C1, P;
extern uint32_t raw_pressure, raw_temperature, temp;
extern float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
extern float ground_pressure, altutude_hold_pressure;
extern int32_t dT, dT_C5;

//Compass_variables.
extern int16_t compass_x, compass_y, compass_z;

void gyro_signalen(void);
void red_led(int8_t level);
void green_led(int8_t level);
void blue_led(int8_t level);
void check_barometer(void);
void check_battery_voltage(void);
void read_data();
void check_compass(void);
void check_gps(void);
void check_imu_angles(void);
void check_motor_vibrations(void);
void i2c_scanner(void);
void handler_channel_1(void);
void print_intro(void);
void read_gyro_values(void);
void reading_receiver_signals(void);
void test_leds(void);
void timer_setup(void);