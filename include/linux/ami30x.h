/* include/linux/ami304.h - AMI304 compass driver
 *
 * Copyright (C) 2009 AMIT Technology Inc.
 * Author: Kyle Chen <sw-support@amit-inc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Definitions for ami304 compass chip.
 */
#ifndef AMI304_H
#define AMI304_H

#ifndef _WIN32 
#include <linux/ioctl.h>
//#include <asm-arm/arch/regs-gpio.h>
#endif

//#define AMI304_I2C_ADDRESS 			0x0E  //new Addr=0x0E(Low), old Addr=0x0F(High)
#define AMI304_I2C_ADDRESS 			0x0F  //AMI306 daughter-board

/* AMI304 Internal Register Address  (Please refer to AMI304 Specifications) */
#define AMI304_REG_INFO				0x0D
#define AMI304_REG_WIA				0x0F//Who I am
#define AMI304_REG_CTRL1			0x1B
#define AMI304_REG_CTRL2			0x1C
#define AMI304_REG_CTRL3			0x1D
#define AMI304_REG_CTRL4			0x5C
#define AMI304_REG_DATAXL			0x10
#define AMI304_REG_DATAXH   		0x11
#define AMI304_REG_DATAYL			0x12
#define AMI304_REG_DATAYH			0x13
#define AMI304_REG_DATAZL	      	0x14
#define AMI304_REG_DATAZH			0x15
#define AMI304_REG_GAIN_PARAX		0x9C
#define AMI304_REG_GAIN_PARAY		0x9E
#define AMI304_REG_GAIN_PARAZ		0xA0

/* AMI304 Control Bit  (Please refer to AMI304 Specifications) */
#define AMI304_CTRL1_PC1_ACTIVE		0x80
#define AMI304_CTRL1_PC1_STANDBY	0x00
#define AMI304_CTRL1_FS1_NORMAL		0x00 //Normal
#define AMI304_CTRL1_FS1_FORCE		0x02 //Force
#define AMI304_CTRL1_ODR1			0x10 //20SPS(20HZ)
#define AMI304_CTRL2_DREN			0x08
#define AMI304_CTRL2_DRP			0x04
#define AMI304_CTRL3_FORCE_BIT		0x40
#define AMI304_CTRL3_B0_LO  		0x10
#define AMI304_CTRL4_COMPASS_MODE	0x00
#define AMI304_CTRL4_LOWSPEED_MODE	0x00
#define AMI306_CTRL4_GYROSCOPE_MODE 0x7E
#define AMI306_CTRL4_HIGHSPEED_MODE 0xA0
#define AMI304_WIA_VALUE			0x47
#define AMI306_WIA_VALUE			0x46

/* IOCTLs for ami304 misc. device library */
#define AMI304IO						   0x83
#define AMI304_IOCTL_SET_INIT              		_IO(AMI304IO, 0x01)
#define AMI304_IOCTL_SET_STANDBY				_IO(AMI304IO, 0x02)
#define AMI304_IOCTL_READ_CHIPINFO         		_IOR(AMI304IO, 0x03, int)
#define AMI304_IOCTL_READ_SENSORDATA       		_IOR(AMI304IO, 0x04, char)
#define AMI304_IOCTL_READ_POSTUREDATA      		_IOR(AMI304IO, 0x05, int)
#define AMI304_IOCTL_WRITE_POSTUREDATA     		_IOW(AMI304IO, 0x06, int)
#define AMI304_IOCTL_READ_CALIDATA         		_IOR(AMI304IO, 0x07, int)
#define AMI304_IOCTL_WRITE_CALIDATA        		_IOW(AMI304IO, 0x08, int)
#define AMI304_IOCTL_READ_GYRODATA         		_IOR(AMI304IO, 0x09, int)
#define AMI304_IOCTL_WRITE_GYRODATA        		_IOW(AMI304IO, 0x0A, int)
#define AMI304_IOCTL_READ_PEDODATA         		_IOR(AMI304IO, 0x0B, long)
#define AMI304_IOCTL_WRITE_PEDODATA        		_IOW(AMI304IO, 0x0C, long)
#define AMI304_IOCTL_READ_PEDOPARAM        		_IOR(AMI304IO, 0x0D, int)
#define AMI304_IOCTL_WRITE_PEDOPARAM       		_IOW(AMI304IO, 0x0E, int)
#define AMI304_IOCTL_READ_CONTROL          		_IOR(AMI304IO, 0x0F, int)
#define AMI304_IOCTL_WRITE_CONTROL         		_IOW(AMI304IO, 0x10, int)
#define AMI304_IOCTL_WRITE_MODE            		_IOW(AMI304IO, 0x11, int)
#define AMI304_IOCTL_WRITE_REPORT          		_IO(AMI304IO, 0x12)
#define AMI304_IOCTL_READ_WIA			   		_IOR(AMI304IO, 0x13, int)
#define AMI304_IOCTL_READ_AXISINTERFERENCE		_IOR(AMI304IO, 0x14, int)
#define AMI304_IOCTL_GET_DIRPOLARITY			_IOR(AMI304IO, 0x15, int)
#define AMI304_IOCTL_READ_ROTATION_VECTOR		_IOR(AMI304IO, 0x16, int)
#define AMI304_IOCTL_WRITE_ROTATION_VECTOR		_IOW(AMI304IO, 0x17, int)
#define AMI304_IOCTL_READ_LINEAR_ACCEL			_IOR(AMI304IO, 0x18, int)
#define AMI304_IOCTL_WRITE_LINEAR_ACCEL			_IOW(AMI304IO, 0x19, int)
#define AMI304_IOCTL_READ_GRAVITY				_IOR(AMI304IO, 0x1A, int)
#define AMI304_IOCTL_WRITE_GRAVITY				_IOW(AMI304IO, 0x1B, int)
#define AMI304_IOCTL_READ_SECURITY				_IOR(AMI304IO, 0x1C, int)
#define AMI304_IOCTL_WRITE_I2CDATA				_IOW(AMI304IO, 0x1D, int)
#define AMI304_IOCTL_WRITE_I2CADDR				_IOW(AMI304IO, 0x1E, int)
#define AMI304_IOCTL_READ_I2CDATA				_IOR(AMI304IO, 0x1F, int)

/* IOCTLs for AMI304 middleware misc. device library */
#define AMI304DAEIO						   0x84
#define AMI304DAE_IOCTL_SET_INIT				_IO(AMI304DAEIO, 0x01)
#define AMI304DAE_IOCTL_SET_STANDBY				_IO(AMI304DAEIO, 0x02)
#define AMI304DAE_IOCTL_GET_SENSORDATA     		_IOR(AMI304DAEIO, 0x03, char)
#define AMI304DAE_IOCTL_SET_POSTURE        		_IOW(AMI304DAEIO, 0x04, int)
#define AMI304DAE_IOCTL_SET_CALIDATA       		_IOW(AMI304DAEIO, 0x05, int)
#define AMI304DAE_IOCTL_SET_GYRODATA       		_IOW(AMI304DAEIO, 0x06, int)
#define AMI304DAE_IOCTL_SET_PEDODATA       		_IOW(AMI304DAEIO, 0x07, long)
#define AMI304DAE_IOCTL_GET_PEDOPARAM      		_IOR(AMI304DAEIO, 0x08, int)
#define AMI304DAE_IOCTL_SET_PEDOPARAM      		_IOR(AMI304DAEIO, 0x09, int)
#define AMI304DAE_IOCTL_SET_CONTROL        		_IOW(AMI304DAEIO, 0x0A, int)
#define AMI304DAE_IOCTL_GET_CONTROL        		_IOR(AMI304DAEIO, 0x0B, int)
#define AMI304DAE_IOCTL_SET_MODE           		_IOW(AMI304DAEIO, 0x0C, int)
#define AMI304DAE_IOCTL_SET_REPORT         		_IO(AMI304DAEIO, 0x0D)
#define AMI304DAE_IOCTL_GET_WIA			   		_IOR(AMI304DAEIO, 0x0E, int)
#define AMI304DAE_IOCTL_GET_AXISINTERFERENCE	_IOR(AMI304DAEIO, 0x0F, int)
#define AMI304DAE_IOCTL_GET_DIRPOLARITY			_IOR(AMI304DAEIO, 0x10, int)
#define AMI304DAE_IOCTL_SET_ROTATION_VECTOR		_IOW(AMI304DAEIO, 0x11, int)
#define AMI304DAE_IOCTL_SET_LINEAR_ACCEL		_IOW(AMI304DAEIO, 0x12, int)
#define AMI304DAE_IOCTL_SET_GRAVITY				_IOW(AMI304DAEIO, 0x13, int)
#define AMI304DAE_IOCTL_GET_SECURITY			_IOR(AMI304DAEIO, 0x14, int)
#define AMI304DAE_IOCTL_SET_I2CDATA				_IOW(AMI304DAEIO, 0x15, int)
#define AMI304DAE_IOCTL_SET_I2CADDR				_IOW(AMI304DAEIO, 0x16, int)
#define AMI304DAE_IOCTL_GET_I2CDATA				_IOR(AMI304DAEIO, 0x17, int)

/* IOCTLs for AMI304 HAL misc. device library */
#define AMI304HALIO						   0x85
#define AMI304HAL_IOCTL_GET_SENSORDATA     _IOR(AMI304HALIO, 0x01, char)
#define AMI304HAL_IOCTL_GET_POSTURE        _IOR(AMI304HALIO, 0x02, int)
#define AMI304HAL_IOCTL_GET_CALIDATA       _IOR(AMI304HALIO, 0x03, int)
#define AMI304HAL_IOCTL_GET_GYRODATA       _IOR(AMI304HALIO, 0x04, int)
#define AMI304HAL_IOCTL_GET_PEDODATA       _IOR(AMI304HALIO, 0x05, long)
#define AMI304HAL_IOCTL_GET_PEDOPARAM      _IOR(AMI304HALIO, 0x06, int)
#define AMI304HAL_IOCTL_SET_PEDOPARAM      _IOW(AMI304HALIO, 0x07, int)
#define AMI304HAL_IOCTL_GET_CONTROL        _IOR(AMI304HALIO, 0x08, int)
#define AMI304HAL_IOCTL_SET_CONTROL        _IOW(AMI304HALIO, 0x09, int)
#define AMI304HAL_IOCTL_GET_WIA			   _IOR(AMI304HALIO, 0x0A, int)
#define AMI304HAL_IOCTL_GET_ROTATION_VECTOR	 _IOR(AMI304HALIO, 0x0B, int)
#define AMI304HAL_IOCTL_GET_LINEAR_ACCEL	 _IOR(AMI304HALIO, 0x0C, int)
#define AMI304HAL_IOCTL_GET_GRAVITY			 _IOR(AMI304HALIO, 0x0D, int)


#define AMI304_CHIPSET						0
#define AMI306_CHIPSET						1
#define AMI304_BUFSIZE						256
#define AMI304_I2C_BUFSIZE					32
#define AMI30x_AXIS_INTERFERENCE			127
#define AMI304_NORMAL_MODE					0
#define AMI304_FORCE_MODE					1
#define AMI304_IRQ							IRQ_EINT9
#define AMI30x_DEFAULT_POLLING_TIME 		200
#define AMI304_REPORT_EN_COMPASS    		1
#define AMI304_REPORT_EN_GYROSCOPE  		2
#define AMI304_POSTURE_ACCURACY_RATE        100
#define AMI304_QUATERNION_ACCURACY_RATE     1000000
#define AMI304_ACCELEROMETER_SENSITIVITY	2 //mean +/- 2G
#define AMI304_MEGNETIC_SENSITIVITY			3 //mean +/- 3G

/* Define items in Control-Byte */
#define AMI304_CB_LENGTH			10
#define AMI304_CB_LOOPDELAY			 0
#define AMI304_CB_RUN				 1
#define AMI304_CB_ACCCALI			 2
#define AMI304_CB_MAGCALI			 3
#define AMI304_CB_ACTIVESENSORS		 4
#define AMI304_CB_PD_RESET			 5
#define AMI304_CB_PD_EN_PARAM		 6
#define AMI304_CB_GYROCALI			 7
#define AMI304_CB_ALGORITHMLOG		 8
#define AMI304_CB_UNDEFINE_1		 9

/* Define items in DirPolar-Byte */
#define AMI304_DP_LENGTH			 6
#define AMI304_DP_ACC_DIR			 0
#define AMI304_DP_ACC_POLARITY		 1
#define AMI304_DP_MAG_DIR			 2
#define AMI304_DP_MAG_POLARITY		 3
#define AMI304_DP_GYRO_DIR			 4
#define AMI304_DP_GYRO_POLARITY		 5

/* Pedometer Parameters */
#define AMI304_PD_LENGTH			10
#define AMI304_PD_PRARM_IIR1		 0
#define AMI304_PD_PRARM_IIR2		 1
#define AMI304_PD_PRARM_IIR3		 2
#define AMI304_PD_PRARM_IIR4		 3
#define AMI304_PD_PRARM_TH1			 4
#define AMI304_PD_PRARM_TH2			 5
#define AMI304_PD_PRARM_TH3			 6
#define AMI304_PD_PRARM_TH4			 7
#define AMI304_PD_UNDEFINE_1		 8
#define AMI304_PD_UNDEFINE_2		 9

/* Define AMIT Sensor Type */
#define AMIT_ACCELEROMETER_SENSOR	    0
#define AMIT_MAGNETIC_FIELD_SENSOR		1
#define AMIT_ORIENTATION_SENSOR			2
#define AMIT_ROTATION_VECTOR			3
#define AMIT_LINEAR_ACCELERATION		4
#define AMIT_GRAVITY					5
#define AMIT_GYROSCOPE_SENSOR	        6
#define AMIT_PEDOMETER_SENSOR	        9

#define AMIT_BIT_ACCELEROMETER			(1<<AMIT_ACCELEROMETER_SENSOR)
#define AMIT_BIT_MAGNETIC_FIELD			(1<<AMIT_MAGNETIC_FIELD_SENSOR)
#define AMIT_BIT_ORIENTATION		    (1<<AMIT_ORIENTATION_SENSOR)
#define AMIT_BIT_ROTATION_VECTOR		(1<<AMIT_ROTATION_VECTOR)
#define AMIT_BIT_LINEAR_ACCELERATION	(1<<AMIT_LINEAR_ACCELERATION)
#define AMIT_BIT_GRAVITY				(1<<AMIT_GRAVITY)
#define AMIT_BIT_GYROSCOPE				(1<<AMIT_GYROSCOPE_SENSOR)
#define AMIT_BIT_PEDOMETER				(1<<AMIT_PEDOMETER_SENSOR)

#ifdef __KERNEL__
struct ami304_platform_data {
	int acc_dir;
	int acc_polarity;		
	int gyro_dir;
	int gyro_polarity;
	int mag_dir;
	int mag_polarity;
};
#endif

#endif
