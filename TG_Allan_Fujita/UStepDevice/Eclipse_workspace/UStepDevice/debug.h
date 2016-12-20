/*
 * debug.h
 *
 *  Created on: May 18, 2015
 *      Author: andre
 */

#ifndef DEBUG_H_
#define DEBUG_H_

// Includes
#include <stdio.h>

// Color changing special characters
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_RESET   "\x1b[0m"

// Enable/Disable debug prints
#define DEBUG_PRINT_ENABLED   1
#define WARNING_PRINT_ENABLED 1
#define ERROR_PRINT_ENABLED   1

// Debug print function
#if DEBUG_PRINT_ENABLED
#define Debug printf
#else
#define Debug(format, args...) ((void)0)
#endif

// Warning print function
#if WARNING_PRINT_ENABLED
#define Warn(format, ...) printf(ANSI_COLOR_YELLOW format ANSI_COLOR_RESET, ##__VA_ARGS__)
#else
#define Warn(format, args...) ((void)0)
#endif

// Error print function
#if ERROR_PRINT_ENABLED
#define Error(format, ...) printf(ANSI_COLOR_RED format ANSI_COLOR_RESET, ##__VA_ARGS__)
#else
#define Error(format, ...) ((void)0)
#endif

// Error codes
#define ERR_MALLOC                  -1
#define ERR_MOTOR_NOT_CONFIGURED    -11
#define ERR_DEVICE_NOT_INITIALIZED  -12
#define ERR_DEVICE_NOT_CALIBRATED   -13
#define ERR_TIME_CALC_INVALID       -14
#define ERR_INVALID_ROTATION_RAMP   -15
#define ERR_INVALID_MOTOR_SPEED     -21
#define ERR_SPEED_TOO_SMALL         -22
#define ERR_SPEED_TOO_HIGH          -23
#define ERR_INSERT_POS_TOO_HIGH     -24
#define ERR_INSERT_POS_TOO_LOW      -25
#define ERR_INVALID_MOTOR_CODE      -31
#define ERR_GPIO_INIT_FAIL          -101
#define ERR_GPIO_WAVE_CREATE_FAIL   -102
#define ERR_WAVES_NOT_PRESENT       -103

#define ERR_ADDR_INFO_CREATE_FAIL   -201
#define ERR_SOCKET_CREATE_FAIL      -202
#define ERR_SOCKET_BIND_FAIL        -203
#define ERR_SOCKET_LISTEN_FAIL      -204
#define ERR_SOCKET_ACCEPT_FAIL      -205
#define ERR_START_TCPIP_FAIL        -206
#define ERR_CLIENT_CONNECT_FAIL     -207
#define ERR_CLIENT_MSG_RECEIVE_FAIL -208

#endif /* DEBUG_H_ */
