/*
Supports firmware for the UVic Formula Hybrid custom throttle body.
Ted Alley
July 2016
*/

#ifndef THROTTLEBODYCONTROLLER_H
#define THROTTLEBODYCONTROLLER_H

#define DEBUG 1
#define CLOCK_MULT_FOR_PWM 1

#define BM0 (1<<0)
#define BM1 (1<<1)
#define BM2 (1<<2)
#define BM3 (1<<3)
#define BM4 (1<<4)
#define BM5 (1<<5)
#define BM6 (1<<6)
#define BM7 (1<<7)
#define BM8 (1<<8)
#define BM9 (1<<9)
#define BM10 (1<<10)
#define BM11 (1<<11)
#define BM12 (1<<12)
#define BM13 (1<<13)
#define BM14 (1<<14)
#define BM15 (1<<15)

#define SPI_CAN_CS 9
#define SPI_HALL_CS 10
#define CURRENT_SENS 1
#define VOLTAGE_SENS 0
#define TEMP_SENS 2
#define MOTOR_OPEN_PIN 5
#define MOTOR_CLOSE_PIN 4

#define HALL_AVERAGE_SIZE 8      //number of readings to take into account over 1ms. Makes reading more accurate.
#define HALL_ZERO_READING_COUNT 1

#define PID_EXECUTION_INTERVAL 5  //number of ms between changes in PID controller
#define CAN_SEND_INTERVAL 10
#define VALIDITY_CHECK_INTERVAL 100

#define HALL_GET_ANGLE 0x3FFF
#define HALL_GET_ERROR 0x0001
#define HALL_ZERO_ANGLE_HIGH 0x0016
#define HALL_ZERO_ANGLE_LOW 0x0017
#define HALL_NOP_COMMAND 0

#define CAN_DIAG_MSG_ADDRESS 0x103
#define CAN_THROTTLE_MSG_ADDRESS 45
#define CAN_FEEDBACK_MSG_ADDRESS 0x101

#define SPI_HALL_SELECT() PORTB &= ~(0b00000100)        //digitalWrite(SPI_HALL_CS, LOW)
#define SPI_HALL_DESELECT() PORTB |= (0b00000100)       //digitalWrite(SPI_HALL_CS, HIGH)

SPISettings SPI_SETTINGS_HALL(8000000, MSBFIRST, SPI_MODE1);
SPISettings SPI_SETTINGS_CAN(8000000, MSBFIRST, SPI_MODE0);

#define CONTROLLER_KP 0.04    //2 * (pi/2)/100
#define CONTROLLER_KI 0.08    // 1.5 * (pi/2)/100
#define CONTROLLER_KD 0.0002    // 0.006 * (pi/2)/100

#define CONTROLLER_I_TERM_MAX 0.05
#define CONTROLLER_I_TERM_MIN -0.05
#define CONTROLLER_I_TERM_RESET_THRESH_PERCENT -5

#define CONTROLLER_D_TERM_MAX 0.017
#define CONTROLLER_D_TERM_MIN 0

#define CONTROLLER_EFFORT_MAX 1
#define CONTROLLER_EFFORT_MIN 0

#define THERMISTOR_B_CONSTANT 3400.0
#define THERMISTOR_CALIB_DEGC 3.0

#endif // THROTTLEBODYCONTROLLER_H

