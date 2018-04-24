/*
 * bno055.h
 *
 *  Created on: Apr 10, 2018
 *      Author: lee7670
 */

#ifndef BNO055_H_
#define BNO055_H_

#define BNO055_ID 		 0xA0

#define DEV_ADD          (0x28<<1)
#define REG_CHIP_ID      0x00
#define UNIT_SELECT_ADD  0x3B
#define UNIT_SELECT_DATA 0x68
#define OPR_MODE_ADD     0x3D
#define OPR_MODE_DATA    0xFC
#define EUL_X_ADD        0x1A
#define EUL_Y_ADD        0x1C
#define EUL_Z_ADD        0x1E
#define LIA_X_ADD        0x28
#define LIA_Y_ADD        0x2A
#define LIA_Z_ADD        0x2C
//Definitions for operating mode
#define OPERATION_MODE_CONFIG        0x00
#define OPERATION_MODE_ACCONLY       0x01
#define OPERATION_MODE_MAGONLY       0x02
#define OPERATION_MODE_GYRONLY       0x03
#define OPERATION_MODE_ACCMAG        0x04
#define OPERATION_MODE_ACCGYRO       0x05
#define OPERATION_MODE_MAGGYRO       0x06
#define OPERATION_MODE_AMG           0x07
#define OPERATION_MODE_IMUPLUS       0x08
#define OPERATION_MODE_COMPASS       0x09
#define OPERATION_MODE_M4G           0x0A
#define OPERATION_MODE_NDOF_FMC_OFF  0x0B
#define OPERATION_MODE_NDOF          0x0C
#define BNO055_EULER_H_LSB_ADDR 	 0x1A
#define BNO055_TEMP_ADDR			 0x34
#define BNO055_PAGE_ID_ADDR			 0x07
#define BNO055_SYS_TRIGGER_ADDR		 0x3F
#define POWER_MODE_NORMAL            0X00
#define POWER_MODE_LOWPOWER          0X01
#define POWER_MODE_SUSPEND           0X02
#define BNO055_PWR_MODE_ADDR		 0x3E
#endif /* BNO055_H_ */
