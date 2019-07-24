/* drivers/i2c/chips/ami304.c - AMI304 compass driver
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/ami30x.h>
#include <linux/kobject.h>
#include <linux/poll.h> 
#include <linux/time.h>
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>

#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

struct notifier_block e_compass_fb_notif;
#endif
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
#include <linux/platform_device.h>
#include <linux/of_gpio.h>

static int ecompass_status = 0;
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"

#define AMI304_DRV_NAME         "ami304"
#define DRIVER_VERSION          "1.1.7.34"

#define ACC_DIR        6
#define ACC_POLARITY   7
#define MAG_DIR        6
#define MAG_POLARITY   7		
#define GYRO_DIR       6
#define GYRO_POLARITY  7

static struct i2c_client *ami304_i2c_client = NULL;

struct _ami304_data {
    rwlock_t lock;
    int chipset;
    int mode;
    int i2c_read_addr;
    int i2c_read_len;	
    volatile int updated;
} ami304_data;

typedef struct {
    int x;
    int y;
    int z;
}ami304_vec_t;

typedef struct _ami304_posture_t{
    int yaw;
    int roll;
    int pitch;
}ami304_posture_t;

typedef struct _ami30x_posture_t {
    union {
    	struct _ami304_posture_t p;
    	int data[3];
    };	
}ami30x_posture_t, *pami30x_posture_t;

typedef struct {
    unsigned long pedo_step;
    unsigned long pedo_time;
    int pedo_stat;
}ami304_pedo_t;

struct _ami304mid_data {
    rwlock_t datalock;
    rwlock_t ctrllock;  
    int controldata[AMI304_CB_LENGTH];  
    int dirpolarity[AMI304_DP_LENGTH];
    int pedometerparam[AMI304_PD_LENGTH];
    ami30x_posture_t pos;
    ami304_vec_t nm;
    ami304_vec_t na;
    ami304_vec_t gyro;
    ami304_pedo_t pedo; 
    ami304_vec_t linear_accel;
    ami304_vec_t gravity;
    int	rotationvector[4]; 		
    int status;
} ami304mid_data;

struct ami304_i2c_data {
    struct input_dev *input_dev_compass;
    struct input_dev *input_dev_gyroscope;	
    struct i2c_client *client;
#ifdef __KERNEL__    
    struct ami304_platform_data *pdata;
#endif
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#elif defined(CONFIG_FB)

#endif
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"

};

static atomic_t dev_open_count;
static atomic_t daemon_open_count;
//static atomic_t hal_open_count;

static atomic_t o_status;
static atomic_t a_status;
static atomic_t m_status;
static atomic_t g_status;
static atomic_t rv_status;
static atomic_t la_status;
static atomic_t gv_status;

static atomic_t off_status;
static atomic_t off_status_hal;

static int AMI304_I2C_Read(int reg_addr, int buf_len, int *buf)
{
	int res = 0;
	u8  readdata[AMI304_I2C_BUFSIZE];
	u8  regaddr;
	
	memset(readdata, 0x00, sizeof(readdata));

    if (!ami304_i2c_client)
    {
        *buf = 0;
        return -2;
    }
	
	regaddr = (u8)reg_addr;

	res = i2c_master_send(ami304_i2c_client, &regaddr, 1);
	if (res <= 0) {
		printk(KERN_ERR "%s AMI304_I2c_Read error res = %d\n", __func__, res);
		return res;
	}
	udelay(20);
	res = i2c_master_recv(ami304_i2c_client, readdata, buf_len);
	if (res <= 0) {
		printk(KERN_ERR "%s AMI304_I2c_Read error res = %d\n", __func__, res);
		return res;
	}
	
	memcpy(buf,(int *)readdata,buf_len);

	return 0;
}

static int AMI304_I2C_Write(int reg_addr, int buf_len, int *buf)
{
	int res = 0;
	u8 databuf[AMI304_I2C_BUFSIZE];
	
	memset(databuf, 0x00, sizeof(databuf));

	if ( (buf_len+2) > AMI304_I2C_BUFSIZE)
		return -EINVAL;

    if (!ami304_i2c_client)
    {
        return -2;
    }	

    databuf[0] = (u8)reg_addr;
    memcpy(&databuf[1],(u8 *)buf,(buf_len-1));		

	res = i2c_master_send(ami304_i2c_client, databuf, buf_len);	
	if (res <= 0)
		printk(KERN_ERR "%s AMI304_I2c_Write error res = %d\n", __func__, res);

	return 0;
}

static int AMI304_Chipset_StandBy(void)
{
    u8 databuf[10];
    int  res = 0;
    u8 ctrl;
    u8 regaddr;

    regaddr = AMI304_REG_CTRL1;
    res = i2c_master_send(ami304_i2c_client, &regaddr, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_StandBy;  
    res = i2c_master_recv(ami304_i2c_client, &ctrl, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_StandBy;

    databuf[0] = AMI304_REG_CTRL1;
    ctrl &= ~AMI304_CTRL1_PC1_ACTIVE;//0x80
    databuf[1] = ctrl;
    res = i2c_master_send(ami304_i2c_client, databuf, 2);
    if (res <= 0) 
        goto exit_AMI304_Chipset_StandBy;

    return 0; 
 
exit_AMI304_Chipset_StandBy: 
    if (res <= 0) 
    {
        printk(KERN_ERR "Fail to standby chipset(I2C error): ret value=%d\n", res);
    }              
    return res;
}

static int AMI304_Chipset_Active(void)
{
    u8 databuf[10];
    int  res = 0;
    u8 ctrl;
    u8 regaddr;

    regaddr = AMI304_REG_CTRL1;
    res = i2c_master_send(ami304_i2c_client, &regaddr, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Active;  
    res = i2c_master_recv(ami304_i2c_client, &ctrl, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Active;		
	
    databuf[0] = AMI304_REG_CTRL1;
	ctrl |= AMI304_CTRL1_PC1_ACTIVE;//0x80
	databuf[1] = ctrl;
    res = i2c_master_send(ami304_i2c_client, databuf, 2);
    if (res <= 0) 
  		goto exit_AMI304_Chipset_Active;
  	
  	return 0;	 
    
exit_AMI304_Chipset_Active:
    if (res <= 0)
    {
        printk(KERN_ERR "Fail to init chipset(I2C error): ret value=%d\n", res);
    } 
    return res;
}

static int AMI304_Chipset_Init(int mode, int chipset)
{
    u8 databuf[10];
    u8 regaddr;
    u8 ctrl1, ctrl2, ctrl3;
    unsigned char ctrl4[2];
    int res = 0;
    
    regaddr = AMI304_REG_CTRL1;
    res = i2c_master_send(ami304_i2c_client, &regaddr, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;    
    res = i2c_master_recv(ami304_i2c_client, &ctrl1, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;

    regaddr = AMI304_REG_CTRL2;
    res = i2c_master_send(ami304_i2c_client, &regaddr, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;    
    res = i2c_master_recv(ami304_i2c_client, &ctrl2, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;    
    
    regaddr = AMI304_REG_CTRL3;
    res = i2c_master_send(ami304_i2c_client, &regaddr, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;    
    res = i2c_master_recv(ami304_i2c_client, &ctrl3, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;          

    regaddr = AMI304_REG_CTRL4; //2 bytes
    res = i2c_master_send(ami304_i2c_client, &regaddr, 1);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;    
    res = i2c_master_recv(ami304_i2c_client, &(ctrl4[0]), 2);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;    
    
    databuf[0] = AMI304_REG_CTRL1;
    if( mode==AMI304_FORCE_MODE )
    {
        databuf[1] = ctrl1 | AMI304_CTRL1_PC1_ACTIVE | AMI304_CTRL1_FS1_FORCE;
        write_lock(&ami304_data.lock);
        ami304_data.mode = AMI304_FORCE_MODE;
        write_unlock(&ami304_data.lock);            
    }
    else    
    {
        databuf[1] = ctrl1 | AMI304_CTRL1_PC1_ACTIVE | AMI304_CTRL1_FS1_NORMAL | AMI304_CTRL1_ODR1;
        write_lock(&ami304_data.lock);
        ami304_data.mode = AMI304_NORMAL_MODE;
        write_unlock(&ami304_data.lock);            
    }
    res = i2c_master_send(ami304_i2c_client, databuf, 2);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;         
    
    databuf[0] = AMI304_REG_CTRL2;
    databuf[1] = ctrl2 | AMI304_CTRL2_DREN | AMI304_CTRL2_DRP;
    res = i2c_master_send(ami304_i2c_client, databuf, 2);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;         
    
    databuf[0] = AMI304_REG_CTRL3;//set to 1 at B0_LO bit
    databuf[1] = ctrl3 | AMI304_CTRL3_B0_LO;
    res = i2c_master_send(ami304_i2c_client, databuf, 2);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;    

    databuf[0] = AMI304_REG_CTRL3;//set to 0 at B0_LO bit
    ctrl3 &= ~AMI304_CTRL3_B0_LO;
    databuf[1] = ctrl3;
    res = i2c_master_send(ami304_i2c_client, databuf, 2);	
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;    
	
    databuf[0] = AMI304_REG_CTRL4;  
    if( chipset == AMI304_CHIPSET ) //AMI304
    {
        ctrl4[0] = AMI304_CTRL4_COMPASS_MODE;    //0x00
        ctrl4[1] = AMI304_CTRL4_LOWSPEED_MODE;   //0x00
    }
    else if( chipset == AMI306_CHIPSET )   //AMI306
    {
        ctrl4[0] = AMI306_CTRL4_GYROSCOPE_MODE;  //0x7E
        ctrl4[1] = AMI306_CTRL4_HIGHSPEED_MODE;  //0xA0     
    }   
    databuf[1] = ctrl4[0];
    databuf[2] = ctrl4[1];
    res = i2c_master_send(ami304_i2c_client, databuf, 3);
    if (res <= 0) 
        goto exit_AMI304_Chipset_Init;
        
    return 0;        

exit_AMI304_Chipset_Init:
    if (res <= 0)
    {
        printk(KERN_ERR "Fail to init chipset(I2C error): ret value=%d\n", res);
    } 
    return res;
}

static int AMI304_SetMode(int newmode)
{
    int mode = 0;
    int chipset = 0;
    
    read_lock(&ami304_data.lock);
    mode = ami304_data.mode;
    chipset = ami304_data.chipset;
    read_unlock(&ami304_data.lock);     
    
    if (mode == newmode) 
        return 0;   
            
    return AMI304_Chipset_Init(newmode, chipset);
}

static int AMI304_ReadChipInfo(char *buf, int bufsize)
{
    if ((!buf)||(bufsize<=30))
        return -1;
        
    if (!ami304_i2c_client)
    {
        *buf = 0;
        return -2;
    }

    if (ami304_data.chipset == AMI306_CHIPSET)  
    {
        sprintf(buf, "AMI306 Chip");
    }
    else if (ami304_data.chipset == AMI304_CHIPSET) 
    {
        sprintf(buf, "AMI304 Chip");
    }
    else 
        return -1;

    return 0;
}

static int AMI304_WIA(char *wia, int bufsize)
{
    char cmd;
    unsigned char databuf[10];
    int res = 0;

    if ((!wia)||(bufsize<=30))
        return -1;  
        
    if (!ami304_i2c_client)
    {
        *wia = 0;
        return -2;
    }

    cmd = AMI304_REG_WIA;
    res = i2c_master_send(ami304_i2c_client, &cmd, 1);
    if (res <= 0) 
        goto exit_AMI304_WIA;         
    udelay(20);
    res = i2c_master_recv(ami304_i2c_client, &(databuf[0]), 1);
    if (res <= 0) 
        goto exit_AMI304_WIA;       
    sprintf(wia, "%02x", databuf[0]);
    printk(KERN_INFO "WIA=%x", databuf[0]);
    
    return 0;
    
exit_AMI304_WIA:
    if (res <= 0)
    {
        printk(KERN_ERR "Fail to wia(I2C error): ret value=%d\n", res);
    }   
    return res;
}

static int AMI304_ReadSecurity(char *buf, int bufsize)
{
    int info_length = 10;
    char cmd;
    unsigned char databuf[info_length];
    int res = 0;   

    if ((!buf)||(bufsize<=80))
        return -1;
		
    if (!ami304_i2c_client)
    {
        *buf = 0;
        return -2;
    }

    cmd = AMI304_REG_INFO;
    res = i2c_master_send(ami304_i2c_client, &cmd, 1);
    if (res <= 0) 
        goto exit_AMI304_ReadSecurity;          
    udelay(20);
    res = i2c_master_recv(ami304_i2c_client, &(databuf[0]), (info_length-1) ); //get 9 bytes
    if (res <= 0) 
        goto exit_AMI304_ReadSecurity;    

    sprintf(buf, "%02x %02x %02x %02x %02x %02x %02x %02x %02x", 
		databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5], databuf[6], databuf[7], databuf[8]);
	
		return 0;

exit_AMI304_ReadSecurity:
    if (res <= 0)
    {
        printk(KERN_ERR "Fail to read infomation(I2C error): ret value=%d\n", res);
    }    
    return res;
}

static int Identify_AMI_Chipset(void)
{
    char strbuf[AMI304_BUFSIZE];
    int WIARet = 0;
    int res;

    if( (res=AMI304_WIA(strbuf, AMI304_BUFSIZE))!=0 )
        return res;
      
    sscanf(strbuf, "%x", &WIARet);  
	//printk(KERN_INFO "WIARet=0x%x", WIARet); 
	
    if (WIARet == AMI306_WIA_VALUE) 
    {
        ami304_data.chipset = AMI306_CHIPSET;
    }
    else if (WIARet == AMI304_WIA_VALUE)
    {
        ami304_data.chipset = AMI304_CHIPSET;
    }
    else
        return -1;

    return 0;
}

static int AMI304_ReadSensorData(char *buf, int bufsize)
{
    char cmd;
    int mode = 0;   
    unsigned char databuf[10];
    int mx, my, mz;
    int res = 0;

    if ((!buf)||(bufsize<=80))
        return -1;
    
    memset(databuf, 0, sizeof(databuf));   
    if (!ami304_i2c_client)
    {
        *buf = 0;
        return -2;
    }
    
    read_lock(&ami304_data.lock);   
    mode = ami304_data.mode;
    read_unlock(&ami304_data.lock);
        
    mx = my = mz = 0;
    databuf[0] = AMI304_REG_CTRL3;
    databuf[1] = AMI304_CTRL3_FORCE_BIT;
    res = i2c_master_send(ami304_i2c_client, databuf, 2);
    if (res <= 0) 
        goto exit_AMI304_ReadSensorData;         
    msleep(1);	//udelay(700);
    // We can read all measured data in once
    cmd = AMI304_REG_DATAXL;
    res = i2c_master_send(ami304_i2c_client, &cmd, 1);
    if (res <= 0) 
        goto exit_AMI304_ReadSensorData;          
    udelay(20);
    res = i2c_master_recv(ami304_i2c_client, &(databuf[0]), 6);
    if (res <= 0) 
        goto exit_AMI304_ReadSensorData;      
    //mxl, mxh, myl, myh, mzl, mzh
    mx = (databuf[1] << 8) | databuf[0]; 
    my = (databuf[3] << 8) | databuf[2]; 
    mz = (databuf[5] << 8) | databuf[4]; 
    sprintf(buf, "%x %x %x", mx, my, mz);    

		return 0;

exit_AMI304_ReadSensorData:
    if (res <= 0)
    {
        printk(KERN_ERR "Fail to read sensor data(I2C error): ret value=%d\n", res);
    }
    return res;
}

static int AMI304_ReadAxisInterference(char *buf, int bufsize)
{
    char cmd;
    unsigned char databuf[10];
    int res = 0;    

    if ((!buf)||(bufsize<=80))
        return -1;
    if (!ami304_i2c_client)
    {
        *buf = 0;
        return -2;
    } 

    cmd = AMI304_REG_GAIN_PARAX;
    res = i2c_master_send(ami304_i2c_client, &cmd, 1);
    if (res <= 0) 
        goto exit_AMI304_ReadAxisInterference;          
    udelay(20);
    res = i2c_master_recv(ami304_i2c_client, &(databuf[0]), 6);
    if (res <= 0) 
        goto exit_AMI304_ReadAxisInterference;    
    //GAIN_PARA_XZ¡BGAIN_PARA_XY¡BGAIN_PARA_YZ¡BGAIN_PARA_YX¡BGAIN_PARA_ZY¡BGAIN_PARA_ZX
    sprintf(buf, "%02x %02x %02x %02x %02x %02x", databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);

		return 0;

exit_AMI304_ReadAxisInterference:
    if (res <= 0)
    {
        printk(KERN_ERR "Fail to read axis interference(I2C error): ret value=%d\n", res);
    }    
    return res;
}

static void Set_Report_Sensor_Flag(int controldata_active_sensor)
{
    if(controldata_active_sensor & AMIT_BIT_ORIENTATION) 
        atomic_set(&o_status, 1);
    else
        atomic_set(&o_status, 0);

    if(controldata_active_sensor & AMIT_BIT_ACCELEROMETER) 
        atomic_set(&a_status, 1);
    else
        atomic_set(&a_status, 0);

    if(controldata_active_sensor & AMIT_BIT_MAGNETIC_FIELD) 
        atomic_set(&m_status, 1);
    else
        atomic_set(&m_status, 0);
    
    if(controldata_active_sensor & AMIT_BIT_GYROSCOPE) 
        atomic_set(&g_status, 1);
    else
        atomic_set(&g_status, 0);

    if(controldata_active_sensor & AMIT_BIT_ROTATION_VECTOR) 
        atomic_set(&rv_status, 1);
    else
        atomic_set(&rv_status, 0);
		
    if(controldata_active_sensor & AMIT_BIT_LINEAR_ACCELERATION) 
        atomic_set(&la_status, 1);
    else
        atomic_set(&la_status, 0);

    if(controldata_active_sensor & AMIT_BIT_GRAVITY) 
        atomic_set(&gv_status, 1);
    else
        atomic_set(&gv_status, 0);				
}

static int AMI304_ReadPostureData(char *buf, int bufsize)
{
    if ((!buf)||(bufsize<=80))
        return -1;

    read_lock(&ami304mid_data.datalock);
    sprintf(buf, "%d %d %d %d", ami304mid_data.pos.p.yaw, ami304mid_data.pos.p.pitch, ami304mid_data.pos.p.roll, ami304mid_data.status);
    read_unlock(&ami304mid_data.datalock);
    return 0;
}

static int AMI304_ReadCaliData(char *buf, int bufsize)
{
    if ((!buf)||(bufsize<=80))
        return -1;

    read_lock(&ami304mid_data.datalock);
    sprintf(buf, "%d %d %d %d %d %d %d", ami304mid_data.nm.x, ami304mid_data.nm.y, ami304mid_data.nm.z,ami304mid_data.na.x,ami304mid_data.na.y,ami304mid_data.na.z,ami304mid_data.status);
    read_unlock(&ami304mid_data.datalock);
    return 0;
}

static int AMI304_ReadGyroData(char *buf, int bufsize)
{
    if ((!buf)||(bufsize<=80))
        return -1;

    read_lock(&ami304mid_data.datalock);
    sprintf(buf, "%d %d %d", ami304mid_data.gyro.x, ami304mid_data.gyro.y, ami304mid_data.gyro.z);
    read_unlock(&ami304mid_data.datalock);
    return 0;   
}

static int AMI304_ReadPedoData(char *buf, int bufsize)
{
    if ((!buf)||(bufsize<=80))
        return -1;

    read_lock(&ami304mid_data.datalock);
    sprintf(buf, "%ld %ld %d", ami304mid_data.pedo.pedo_step, ami304mid_data.pedo.pedo_time, ami304mid_data.pedo.pedo_stat);
    read_unlock(&ami304mid_data.datalock);
    return 0;       
}

static int AMI304_ReadMiddleControl(char *buf, int bufsize)
{
    if ((!buf)||(bufsize<=80))
        return -1;

    read_lock(&ami304mid_data.ctrllock);
    sprintf(buf, "%d %d %d %d %d %d %d %d %d %d", 
        ami304mid_data.controldata[AMI304_CB_LOOPDELAY], ami304mid_data.controldata[AMI304_CB_RUN], ami304mid_data.controldata[AMI304_CB_ACCCALI], ami304mid_data.controldata[AMI304_CB_MAGCALI],
        ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS], ami304mid_data.controldata[AMI304_CB_PD_RESET], ami304mid_data.controldata[AMI304_CB_PD_EN_PARAM], ami304mid_data.controldata[AMI304_CB_GYROCALI],
        ami304mid_data.controldata[AMI304_CB_ALGORITHMLOG], ami304mid_data.controldata[AMI304_CB_UNDEFINE_1] );
    read_unlock(&ami304mid_data.ctrllock);
    return 0;
}

static int AMI304_ReadRotationVector(char *buf, int bufsize)
{
    if ((!buf)||(bufsize<=80))
        return -1;

    read_lock(&ami304mid_data.ctrllock);
    sprintf(buf, "%d %d %d %d", ami304mid_data.rotationvector[0], ami304mid_data.rotationvector[1], ami304mid_data.rotationvector[2], ami304mid_data.rotationvector[3]);
    read_unlock(&ami304mid_data.ctrllock);
    return 0;
}

static int AMI304_ReadLinearAccel(char *buf, int bufsize)
{
    if ((!buf)||(bufsize<=80))
        return -1;

    read_lock(&ami304mid_data.ctrllock);
    sprintf(buf, "%d %d %d", ami304mid_data.linear_accel.x, ami304mid_data.linear_accel.y, ami304mid_data.linear_accel.z);
    read_unlock(&ami304mid_data.ctrllock);
    return 0;
}

static int AMI304_ReadGravity(char *buf, int bufsize)
{
    if ((!buf)||(bufsize<=80))
        return -1;

    read_lock(&ami304mid_data.ctrllock);
    sprintf(buf, "%d %d %d", ami304mid_data.gravity.x, ami304mid_data.gravity.y, ami304mid_data.gravity.z);
    read_unlock(&ami304mid_data.ctrllock);
    return 0;
}

static void AMI304_Report_Value(void)
{
    struct ami304_i2c_data *data = i2c_get_clientdata(ami304_i2c_client);
    int report_enable = 0;
    
    if(atomic_read(&o_status))      
    {
        input_report_abs(data->input_dev_compass, ABS_RX, ami304mid_data.pos.p.yaw);  /* yaw */
        input_report_abs(data->input_dev_compass, ABS_RY, ami304mid_data.pos.p.pitch);/* pitch */
        input_report_abs(data->input_dev_compass, ABS_RZ, ami304mid_data.pos.p.roll);/* roll */
        input_report_abs(data->input_dev_compass, ABS_RUDDER, ami304mid_data.status);/* status of orientation sensor */
        report_enable = AMI304_REPORT_EN_COMPASS;
    }

    if(atomic_read(&a_status))
    {
        input_report_abs(data->input_dev_compass, ABS_X, ami304mid_data.na.x);/* x-axis raw acceleration */
        input_report_abs(data->input_dev_compass, ABS_Y, ami304mid_data.na.y);/* y-axis raw acceleration */
        input_report_abs(data->input_dev_compass, ABS_Z, ami304mid_data.na.z);/* z-axis raw acceleration */
        report_enable = AMI304_REPORT_EN_COMPASS;
    }

    if(atomic_read(&m_status))
    {
        input_report_abs(data->input_dev_compass, ABS_HAT0X, ami304mid_data.nm.x);/* x-axis of raw magnetic vector */
        input_report_abs(data->input_dev_compass, ABS_HAT0Y, ami304mid_data.nm.y);/* y-axis of raw magnetic vector */
        input_report_abs(data->input_dev_compass, ABS_BRAKE, ami304mid_data.nm.z);/* z-axis of raw magnetic vector */
        input_report_abs(data->input_dev_compass, ABS_WHEEL, ami304mid_data.status);/* status of magnetic sensor */
        report_enable = AMI304_REPORT_EN_COMPASS;    
    }

    if(atomic_read(&rv_status))
    {
        input_report_abs(data->input_dev_compass, ABS_HAT3X, ami304mid_data.rotationvector[0]);/* x-axis of rotation vector */
        input_report_abs(data->input_dev_compass, ABS_HAT3Y, ami304mid_data.rotationvector[1]);/* y-axis of rotation vectorn */
		input_report_abs(data->input_dev_compass, ABS_TILT_X, ami304mid_data.rotationvector[2]);/* z-axis of rotation vector */
		input_report_abs(data->input_dev_compass, ABS_TILT_Y, ami304mid_data.rotationvector[3]);/* theta of rotation vector */
        report_enable = AMI304_REPORT_EN_COMPASS;    
    }
	
    if(atomic_read(&la_status))
    {
        input_report_abs(data->input_dev_compass, ABS_HAT1X, ami304mid_data.linear_accel.x);/* x-axis of linear acceleration */
        input_report_abs(data->input_dev_compass, ABS_HAT1Y, ami304mid_data.linear_accel.y);/* y-axis of linear acceleration */
		input_report_abs(data->input_dev_compass, ABS_TOOL_WIDTH, ami304mid_data.linear_accel.z);/* z-axis of linear acceleration */
        report_enable = AMI304_REPORT_EN_COMPASS;    
    }

    if(atomic_read(&gv_status))
    {
        input_report_abs(data->input_dev_compass, ABS_HAT2X, ami304mid_data.gravity.x);/* x-axis of gravityr */
        input_report_abs(data->input_dev_compass, ABS_HAT2Y, ami304mid_data.gravity.y);/* y-axis of gravity */
        input_report_abs(data->input_dev_compass, ABS_VOLUME, ami304mid_data.gravity.z);/* z-axis of gravity */
        report_enable = AMI304_REPORT_EN_COMPASS;    
    }			
	
    if(AMI304_REPORT_EN_COMPASS == report_enable)
    {
        input_event(data->input_dev_compass, EV_SYN, SYN_REPORT, 1);
        input_sync(data->input_dev_compass);
    }
	
    if(atomic_read(&g_status))
    {
        input_report_rel(data->input_dev_gyroscope, REL_RX, ami304mid_data.gyro.x);/* x-axis of gyro sensor */
        input_report_rel(data->input_dev_gyroscope, REL_RY, ami304mid_data.gyro.y);/* y-axis of gyro sensor */
        input_report_rel(data->input_dev_gyroscope, REL_RZ, ami304mid_data.gyro.z);/* z-axis of gyro sensor */
        report_enable = AMI304_REPORT_EN_GYROSCOPE;       
    }

    if(AMI304_REPORT_EN_GYROSCOPE == report_enable)
    {
        input_event(data->input_dev_gyroscope, EV_SYN, SYN_REPORT, 1);
        input_sync(data->input_dev_gyroscope);
    }	
}

static ssize_t show_chipinfo_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    char strbuf[AMI304_BUFSIZE];
    int res = -EFAULT;
	memset(&strbuf[0], 0, sizeof(strbuf)); 
    if( !AMI304_ReadChipInfo(strbuf, AMI304_BUFSIZE) ) //successful
		return sprintf(buf, "%s\n", strbuf);
    return res;
}

static ssize_t show_sensordata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    int res = 0, data_res = 0;
    char strbuf[AMI304_BUFSIZE];
	
	if(atomic_read(&off_status_hal) == 1)
    {
        res = AMI304_Chipset_Active();
        if(res) {
          printk(KERN_ERR "Can't AMI304_Chipset_Active error at show_sensordata_value\n");
          return res;
				}
    }
	memset(&strbuf[0], 0, sizeof(strbuf)); 
	data_res = AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
	
    if(atomic_read(&off_status_hal) == 1)	
    {
 				res = AMI304_Chipset_StandBy();
        if(res) {
          printk(KERN_ERR "Can't AMI304_Chipset_StandBy error at show_sensordata_value\n");
          return res;
				} 	
    }
    if(!data_res)  //successful
		return	sprintf(buf, "%s\n", strbuf); 	
	
	return 0;
}

static ssize_t show_posturedata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    char strbuf[AMI304_BUFSIZE];
    int res = -EFAULT;
	memset(&strbuf[0], 0, sizeof(strbuf)); 
    if( !AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE) ) //successful
		return sprintf(buf, "%s\n", strbuf);            
    return res;
}

static ssize_t show_calidata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    char strbuf[AMI304_BUFSIZE];
    int res = -EFAULT;
	memset(&strbuf[0], 0, sizeof(strbuf)); 
    if( !AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE) ) //successful
		return sprintf(buf, "%s\n", strbuf);            
    return res;
}

static ssize_t show_gyrodata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    char strbuf[AMI304_BUFSIZE];
    int res = -EFAULT;
    memset(&strbuf[0], 0, sizeof(strbuf)); 
    if( !AMI304_ReadGyroData(strbuf, AMI304_BUFSIZE) ) //successful
			return sprintf(buf, "%s\n", strbuf);            
    return res;
}

static ssize_t show_rv_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    char strbuf[AMI304_BUFSIZE];
    int res = -EFAULT;
    memset(&strbuf[0], 0, sizeof(strbuf)); 
    if( !AMI304_ReadRotationVector(strbuf, AMI304_BUFSIZE) )
			return sprintf(buf, "%s\n", strbuf);            
    return res;
}

static ssize_t show_ladata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    char strbuf[AMI304_BUFSIZE];
    int res = -EFAULT;
    memset(&strbuf[0], 0, sizeof(strbuf)); 
    if( !AMI304_ReadLinearAccel(strbuf, AMI304_BUFSIZE) )
			return sprintf(buf, "%s\n", strbuf);            
    return res;
}

static ssize_t show_gravitydata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    char strbuf[AMI304_BUFSIZE];
    int res = -EFAULT;
    memset(&strbuf[0], 0, sizeof(strbuf)); 
    if( !AMI304_ReadGravity(strbuf, AMI304_BUFSIZE) )
			return sprintf(buf, "%s\n", strbuf);            
    return res;
}

static ssize_t show_midcontrol_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    char strbuf[AMI304_BUFSIZE];
    int res = -EFAULT;
    memset(&strbuf[0], 0, sizeof(strbuf));
    if( !AMI304_ReadMiddleControl(strbuf, AMI304_BUFSIZE) )  //successful
			return sprintf(buf, "%s\n", strbuf);            
    return res;
}

static ssize_t show_mode_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    int mode=0;
    read_lock(&ami304_data.lock);
    mode = ami304_data.mode;
    read_unlock(&ami304_data.lock);     
    return sprintf(buf, "%d\n", mode);          
}

static ssize_t show_wia_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    char strbuf[AMI304_BUFSIZE];
	int res = -EFAULT;
    memset(&strbuf[0], 0, sizeof(strbuf));
    if( !AMI304_WIA(strbuf, AMI304_BUFSIZE) ) //successful
			return sprintf(buf, "%s\n", strbuf);            
    return res;
}

static DEVICE_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DEVICE_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DEVICE_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DEVICE_ATTR(calidata, S_IRUGO, show_calidata_value, NULL);
static DEVICE_ATTR(gyrodata, S_IRUGO, show_gyrodata_value, NULL);
static DEVICE_ATTR(rvdata, S_IRUGO, show_rv_value, NULL);
static DEVICE_ATTR(ladata, S_IRUGO, show_ladata_value, NULL);
static DEVICE_ATTR(gravitydata, S_IRUGO, show_gravitydata_value, NULL);
static DEVICE_ATTR(midcontrol, S_IRUGO | S_IWUSR, show_midcontrol_value, NULL );
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_mode_value, NULL );
static DEVICE_ATTR(wia, S_IRUGO, show_wia_value, NULL);

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
static ssize_t read_compass_status(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	return sprintf(buf, "%d\n", ecompass_status);
}

static ssize_t read_compass_raw(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int res = 0;
    unsigned char databuf[6];
    short dat[3];
	
	if(!ami304_i2c_client)
	{
		printk(KERN_INFO "[AMI306] I2C error\n");
		return sprintf(buf, "0 0 0\n"); 
	}
	if(atomic_read(&off_status_hal) == 1)
    {
        res = AMI304_Chipset_Active();
        if(res) {
          printk(KERN_ERR "Can't AMI304_Chipset_Active error at show_sensordata_value\n");
          return sprintf(buf, "0 0 0\n"); 
		}
    }

    databuf[0] = AMI304_REG_CTRL3;
    databuf[1] = AMI304_CTRL3_FORCE_BIT;
    res = i2c_master_send(ami304_i2c_client, databuf, 2);
    if (res <= 0) 
        return sprintf(buf, "0 0 0\n");        
    msleep(1);	//udelay(700);

    res = i2c_smbus_read_i2c_block_data(ami304_i2c_client, AMI304_REG_DATAXL, sizeof(databuf), databuf);
    
	dat[0] = le16_to_cpup((__le16 *)(&databuf[0]));
	dat[1] = le16_to_cpup((__le16 *)(&databuf[2]));
	dat[2] = le16_to_cpup((__le16 *)(&databuf[4]));
     
	return sprintf(buf, "%d %d %d\n", dat[0], dat[1], dat[2]);
}

static ssize_t check_compass_i2c(struct device *dev, struct device_attribute *devattr, char *buf)
{
	unsigned char data[8];
	int result = 0, ret = 0;
	char cmd;
	
	if(!ami304_i2c_client)
	{
		printk(KERN_INFO "[AMI306] I2C error\n");
		return sprintf(buf, "0\n"); 
	}
	
	cmd = AMI304_REG_WIA;
	result = i2c_master_send(ami304_i2c_client, &cmd, 1);
    udelay(20);
    result = i2c_master_recv(ami304_i2c_client, &(data[0]), 8);	
	
	printk("[AMI306] chip ID = (%d) \n", data[0]);
	
	ret = (data[0] == 0x46) ? (1) : (0);
	
	return sprintf(buf, "%d\n", ret);
}

static ssize_t check_compass_run_mode(struct device *dev, struct device_attribute *devattr, char *buf)
{
	unsigned char data[8];
	int result = 0;
	char cmd;
	
	if(!ami304_i2c_client)
	{
		printk(KERN_INFO "[AMI306] I2C error\n");
		return sprintf(buf, "i2c error\n"); 
	}
	
	cmd = AMI304_REG_CTRL1;
	result = i2c_master_send(ami304_i2c_client, &cmd, 1);
    udelay(20);
    result = i2c_master_recv(ami304_i2c_client, &(data[0]), 8);	
	
	return sprintf(buf, "%d\n", (((data[0] & AMI304_CTRL1_PC1_ACTIVE) >> 7) & 0x1));   
}

static DEVICE_ATTR(compass_6500_status, S_IRUGO, read_compass_status, NULL);
static DEVICE_ATTR(compass_6500_raw, S_IRUGO, read_compass_raw, NULL);
static DEVICE_ATTR(compass_6500_i2c, S_IRUGO, check_compass_i2c, NULL);
static DEVICE_ATTR(compass_6500_run_mode, S_IRUGO, check_compass_run_mode, NULL);
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"

static struct attribute *ami304_attributes[] = {
    &dev_attr_chipinfo.attr,
    &dev_attr_sensordata.attr,
    &dev_attr_posturedata.attr,
    &dev_attr_calidata.attr,
    &dev_attr_gyrodata.attr,	
    &dev_attr_rvdata.attr,
    &dev_attr_ladata.attr,
    &dev_attr_gravitydata.attr,
    &dev_attr_midcontrol.attr,
    &dev_attr_mode.attr,
    &dev_attr_wia.attr,
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
    &dev_attr_compass_6500_status.attr,
    &dev_attr_compass_6500_raw.attr,
    &dev_attr_compass_6500_i2c.attr,
    &dev_attr_compass_6500_run_mode.attr,
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
    NULL
};

static struct attribute_group ami304_attribute_group = {
    .attrs = ami304_attributes
};

static int ami304_open(struct inode *inode, struct file *file)
{   
    int ret = -EFAULT;
    if( atomic_cmpxchg(&dev_open_count, 0, 1)==0 ) {
        printk(KERN_INFO "Open device node:ami304\n");
        ret = nonseekable_open(inode, file);
    }   
    return ret;
}

static int ami304_release(struct inode *inode, struct file *file)
{
    atomic_set(&dev_open_count, 0);
    printk(KERN_INFO "Release device node:ami304\n");       
    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
static long ami304_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
#else
static int ami304_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
#endif
{
    char strbuf[AMI304_BUFSIZE];
    int dirpolarity[AMI304_DP_LENGTH];
    int controlbuf[AMI304_CB_LENGTH];
    int valuebuf[4];
    int calidata[7];
    int gyrodata[3];
    long pedodata[3], ladata[3], gravitydata[3];   
    int pedoparam[AMI304_PD_LENGTH];
    void __user *data;
    int retval=0;
    int mode=0,chipset=0;
    int rotation_vector[4];
    int i2creaddata[3];
    int i2cwrdata[AMI304_I2C_BUFSIZE];
        
    switch (cmd) {
    
        case AMI304_IOCTL_SET_INIT:
            read_lock(&ami304_data.lock);
            mode = ami304_data.mode;
            chipset = ami304_data.chipset;
            read_unlock(&ami304_data.lock);
            retval = AMI304_Chipset_Init(mode, chipset);         
            break;
        
        case AMI304_IOCTL_SET_STANDBY:
            retval = AMI304_Chipset_StandBy();
            break;
        
        case AMI304_IOCTL_READ_CHIPINFO:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            AMI304_ReadChipInfo(strbuf, AMI304_BUFSIZE);
            if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
                retval = -EFAULT;
                goto err_out;
            }               
            break;

        case AMI304_IOCTL_READ_SENSORDATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);			
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					   retval = -EFAULT;
	    					   goto err_out;
	    				}
	    			}
            break;              
                        
        case AMI304_IOCTL_READ_POSTUREDATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}
	    			}				
            break;          
        
        case AMI304_IOCTL_WRITE_POSTUREDATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(&valuebuf, data, sizeof(valuebuf))) {
                retval = -EFAULT;
                goto err_out;
            }               
            write_lock(&ami304mid_data.datalock);
           
            memcpy(ami304mid_data.pos.data, &valuebuf[0], sizeof(int)*3);
            ami304mid_data.status = valuebuf[3];
            write_unlock(&ami304mid_data.datalock);         
            break;
         
        case AMI304_IOCTL_READ_CALIDATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}
	    			}
            break;
        
        case AMI304_IOCTL_WRITE_CALIDATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(&calidata, data, sizeof(calidata))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.datalock);           
            ami304mid_data.nm.x = calidata[0];
            ami304mid_data.nm.y = calidata[1];
            ami304mid_data.nm.z = calidata[2];
            ami304mid_data.na.x = calidata[3];
            ami304mid_data.na.y = calidata[4];
            ami304mid_data.na.z = calidata[5];
            ami304mid_data.status = calidata[6];
            write_unlock(&ami304mid_data.datalock);
            break;    

        case AMI304_IOCTL_READ_GYRODATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadGyroData(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}
	    			}
            break;
            
        case AMI304_IOCTL_WRITE_GYRODATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(&gyrodata, data, sizeof(gyrodata))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.datalock);           
            ami304mid_data.gyro.x = gyrodata[0];
            ami304mid_data.gyro.y = gyrodata[1];
            ami304mid_data.gyro.z = gyrodata[2];
            write_unlock(&ami304mid_data.datalock);     
            break;
            
        case AMI304_IOCTL_READ_PEDODATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadPedoData(strbuf, AMI304_BUFSIZE);
            if( !retval )  {//successful
							if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
								retval = -EFAULT;
								goto err_out;
							}
	    			}
            break;

        case AMI304_IOCTL_WRITE_PEDODATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(&pedodata, data, sizeof(pedodata))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.datalock);           
            ami304mid_data.pedo.pedo_step = pedodata[0];
            ami304mid_data.pedo.pedo_time = pedodata[1];
            ami304mid_data.pedo.pedo_stat = (int)pedodata[2];
            write_unlock(&ami304mid_data.datalock);         
            break;

        case AMI304_IOCTL_READ_PEDOPARAM:
            read_lock(&ami304mid_data.ctrllock);
            memcpy(pedoparam, &ami304mid_data.pedometerparam[0], sizeof(pedoparam));
            read_unlock(&ami304mid_data.ctrllock);          
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_to_user(data, pedoparam, sizeof(pedoparam))) {
                retval = -EFAULT;
                goto err_out;
            }           
            break;
            
        case AMI304_IOCTL_WRITE_PEDOPARAM:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(pedoparam, data, sizeof(pedoparam))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            memcpy(&ami304mid_data.pedometerparam[0], pedoparam, sizeof(pedoparam));
            write_unlock(&ami304mid_data.ctrllock);
            break;  
            
        case AMI304_IOCTL_READ_CONTROL:
            read_lock(&ami304mid_data.ctrllock);
            memcpy(controlbuf, &ami304mid_data.controldata[0], sizeof(controlbuf));
            read_unlock(&ami304mid_data.ctrllock);          
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
                retval = -EFAULT;
                goto err_out;
            }                               
            break;

        case AMI304_IOCTL_WRITE_CONTROL:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            memcpy(&ami304mid_data.controldata[0], controlbuf, sizeof(controlbuf));
            write_unlock(&ami304mid_data.ctrllock);     
            Set_Report_Sensor_Flag(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS]);
	    			if(!atomic_read(&m_status) && !atomic_read(&g_status) && !atomic_read(&rv_status) && !atomic_read(&la_status) && !atomic_read(&gv_status) &&!atomic_read(&o_status) && !atomic_read(&off_status_hal))//power off
	    			{//power off
	    				atomic_set(&off_status_hal, 1);				
	    				return AMI304_Chipset_StandBy();
	    			}
	    			else if( (atomic_read(&m_status) || (atomic_read(&g_status) || atomic_read(&rv_status) || atomic_read(&la_status) || atomic_read(&gv_status) || atomic_read(&o_status)))  && atomic_read(&off_status_hal))//power on			
	    			{//power on
	    				atomic_set(&off_status_hal, 0);		
	    				return AMI304_Chipset_Active();
	    			}
            break;
            
        case AMI304_IOCTL_WRITE_MODE:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(&mode, data, sizeof(mode))) {
                retval = -EFAULT;
                goto err_out;
            }       
            AMI304_SetMode(mode);               
            break;
        
        case AMI304_IOCTL_WRITE_REPORT:
            AMI304_Report_Value();     
            break;
        
        case AMI304_IOCTL_READ_WIA:
            data = (void __user *) arg;
            if (data == NULL)
                break;      
            retval = AMI304_WIA(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}
	    			}				
            break;
                     
        case AMI304_IOCTL_READ_AXISINTERFERENCE:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadAxisInterference(strbuf, AMI304_BUFSIZE);    
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}  
	    			}
            break;

				case AMI304_IOCTL_GET_DIRPOLARITY:	
            read_lock(&ami304mid_data.ctrllock);
            memcpy(dirpolarity, &ami304mid_data.dirpolarity[0], sizeof(dirpolarity));
            read_unlock(&ami304mid_data.ctrllock);          
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_to_user(data, dirpolarity, sizeof(dirpolarity))) {
                retval = -EFAULT;
                goto err_out;
            }                   
            break;

        case AMI304_IOCTL_READ_ROTATION_VECTOR:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadRotationVector(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}               
	    			}
            break;        

        case AMI304_IOCTL_WRITE_ROTATION_VECTOR:
            data = (void __user *) arg;         
            if (data == NULL)
                break;  
            if (copy_from_user(rotation_vector, data, sizeof(rotation_vector))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            memcpy(&ami304mid_data.rotationvector[0], rotation_vector, sizeof(rotation_vector));
            write_unlock(&ami304mid_data.ctrllock);                 
						break;			
			
        case AMI304_IOCTL_READ_LINEAR_ACCEL:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadLinearAccel(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}               
	    			}
            break;        

 				case AMI304_IOCTL_WRITE_LINEAR_ACCEL:
            data = (void __user *) arg;         
            if (data == NULL)
                break;  
            if (copy_from_user(ladata, data, sizeof(ladata))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            ami304mid_data.linear_accel.x = ladata[0];
            ami304mid_data.linear_accel.y = ladata[1];
            ami304mid_data.linear_accel.z = ladata[2];
            write_unlock(&ami304mid_data.ctrllock);                 
            break;  			
			
        case AMI304_IOCTL_READ_GRAVITY:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadGravity(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}               
	    			}
            break;        			

				case AMI304_IOCTL_WRITE_GRAVITY:
            data = (void __user *) arg;         
            if (data == NULL)
                break;  
            if (copy_from_user(gravitydata, data, sizeof(gravitydata))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            ami304mid_data.gravity.x = gravitydata[0];
            ami304mid_data.gravity.y = gravitydata[1];
            ami304mid_data.gravity.z = gravitydata[2];
            write_unlock(&ami304mid_data.ctrllock);                 
            break;  		
			
				case AMI304_IOCTL_READ_SECURITY:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
	    			retval = AMI304_ReadSecurity(strbuf, AMI304_BUFSIZE);	
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}  
	    			}
	    			break;

	 			case AMI304_IOCTL_WRITE_I2CDATA:
	 					data = (void __user *)arg;
	 					if (data == NULL)
	 						break;
	 					if (copy_from_user(i2cwrdata, data, sizeof(i2cwrdata))) {
	 						retval = -EFAULT;
	 						goto err_out;
	 					}
	 					//write buf order is reg_addr, buf_len(unit:byte), data 
	 					retval = AMI304_I2C_Write(i2cwrdata[0], i2cwrdata[1], &i2cwrdata[2]);					
	 					break;
			
				case AMI304_IOCTL_WRITE_I2CADDR:
						data = (void __user *)arg;
						if (data == NULL)
							break;
						if (copy_from_user(i2creaddata, data, sizeof(i2creaddata))) {
							retval = -EFAULT;
							goto err_out;
						}
						read_lock(&ami304_data.lock);
  			 		ami304_data.i2c_read_addr = i2creaddata[0];
  			 		ami304_data.i2c_read_len = i2creaddata[1];
  			 		read_unlock(&ami304_data.lock);		
						break;

				case AMI304_IOCTL_READ_I2CDATA:
						data = (void __user *)arg;
						if (data == NULL)
							break;
						retval = AMI304_I2C_Read(ami304_data.i2c_read_addr, ami304_data.i2c_read_len, &i2cwrdata[0]);
						if( !retval )  {//successful
							if (copy_to_user(data, i2cwrdata, ami304_data.i2c_read_len)) {
								retval = -EFAULT;
								goto err_out;
							}		
						}
						break;
			
        default:
            printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
            retval = -ENOIOCTLCMD;
            break;
    }
    
err_out:
    return retval;  
}

unsigned int ami304_poll(struct file* filp , poll_table * pwait)
{
    unsigned int mask=0; 
    
    if( (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ORIENTATION) || 
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GYROSCOPE) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_MAGNETIC_FIELD) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ACCELEROMETER) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ROTATION_VECTOR) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_LINEAR_ACCELERATION) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GRAVITY) ) 		
    		{
       		mask |= POLLIN | POLLRDNORM;
    		}
    		return mask;
}

static ssize_t ami304_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
  char strbuf[AMI304_BUFSIZE];
	struct input_event event[3];
	int retval = 0;
	int mx, my, mz;

    if( (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ORIENTATION) || 
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GYROSCOPE) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_MAGNETIC_FIELD) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ACCELEROMETER) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ROTATION_VECTOR) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_LINEAR_ACCELERATION) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GRAVITY) )
				{	
						memset(strbuf, 0x00, sizeof(strbuf)); 
						memset(event, 0x00, sizeof(event));
						retval = AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);			
						if( !retval )  {//successful
							//if (copy_to_user(buff, strbuf, strlen(strbuf)+1)) {
							//	retval = -EFAULT;
							//	goto ami304_read_err_out;
							//}
							 /* Most read functions return the number of bytes put into the buffer */
							//retval = strlen(strbuf)+1;
							sscanf(strbuf, "%x %x %x", &mx, &my, &mz); 
							do_gettimeofday(&event[0].time);
							event[0].value = mx;
							event[0].code = ABS_HAT0X;
							event[0].type = EV_ABS;
							
							event[1].time = event[0].time;
							event[1].value = my;
							event[1].code = ABS_HAT0Y;
							event[1].type = EV_ABS;
    				
							event[2].time = event[0].time;
							event[2].value = mz;
							event[2].code = ABS_BRAKE;
							event[2].type = EV_ABS;
    				
							if (copy_to_user(buff, event, sizeof(event))) {
								retval = -EIO;
								goto ami304_read_err_out;
							} 
							retval = sizeof(event);			
						}
    		} //if( [AMI304_CB_ACTIVESENSORS] )
		
ami304_read_err_out:
	return retval;
}

static int ami304daemon_open(struct inode *inode, struct file *file)
{
    int ret = -EFAULT;
    if( atomic_cmpxchg(&daemon_open_count, 0, 1)==0 ) {
        printk(KERN_INFO "Open device node:ami304daemon\n");
        ret = 0;
    }
    return ret; 
}

static int ami304daemon_release(struct inode *inode, struct file *file)
{
    atomic_set(&daemon_open_count, 0);
    printk(KERN_INFO "Release device node:ami304daemon\n"); 
    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
static long ami304daemon_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int ami304daemon_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
    int valuebuf[4];
    int calidata[7];
    int gyrodata[3];
    long pedodata[3], ladata[3], gravitydata[3];
    int dirpolarity[AMI304_DP_LENGTH];
    int controlbuf[AMI304_CB_LENGTH];
    char strbuf[AMI304_BUFSIZE];
    int pedoparam[AMI304_PD_LENGTH];    
    void __user *data;
    int retval=0;
    int mode=0, chipset=0;
    int rotation_vector[4];
    int i2creaddata[3];
    int i2cwrdata[AMI304_I2C_BUFSIZE];	 
 
    switch (cmd) {

        case AMI304DAE_IOCTL_SET_INIT:
            read_lock(&ami304_data.lock);
            mode = ami304_data.mode;
            chipset = ami304_data.chipset;
            read_unlock(&ami304_data.lock);
            retval = AMI304_Chipset_Init(mode, chipset);
            break;

        case AMI304DAE_IOCTL_SET_STANDBY:
            retval = AMI304_Chipset_StandBy();
            break;
    
        case AMI304DAE_IOCTL_GET_SENSORDATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
			if( !retval )  {//successful
				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
					retval = -EFAULT;
					goto err_out;
				}
			}
            break;
                
        case AMI304DAE_IOCTL_SET_POSTURE:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(&valuebuf, data, sizeof(valuebuf))) {
                retval = -EFAULT;
                goto err_out;
            }               
            write_lock(&ami304mid_data.datalock);
           
            memcpy(ami304mid_data.pos.data, &valuebuf[0], sizeof(int)*3);
            ami304mid_data.status = valuebuf[3];
            write_unlock(&ami304mid_data.datalock); 
            break;      
            
        case AMI304DAE_IOCTL_SET_CALIDATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(&calidata, data, sizeof(calidata))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.datalock);           
            ami304mid_data.nm.x = calidata[0];
            ami304mid_data.nm.y = calidata[1];
            ami304mid_data.nm.z = calidata[2];
            ami304mid_data.na.x = calidata[3];
            ami304mid_data.na.y = calidata[4];
            ami304mid_data.na.z = calidata[5];
            ami304mid_data.status = calidata[6];
            write_unlock(&ami304mid_data.datalock);             
            break;                              

        case AMI304DAE_IOCTL_SET_GYRODATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(&gyrodata, data, sizeof(gyrodata))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.datalock);           
            ami304mid_data.gyro.x = gyrodata[0];
            ami304mid_data.gyro.y = gyrodata[1];
            ami304mid_data.gyro.z = gyrodata[2];
            write_unlock(&ami304mid_data.datalock);
            break;
        
        case AMI304DAE_IOCTL_SET_PEDODATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(&pedodata, data, sizeof(pedodata))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.datalock);           
            ami304mid_data.pedo.pedo_step = pedodata[0];
            ami304mid_data.pedo.pedo_time = pedodata[1];
            ami304mid_data.pedo.pedo_stat = (int)pedodata[2];
            write_unlock(&ami304mid_data.datalock);        
            break;

        case AMI304DAE_IOCTL_GET_PEDOPARAM:
            read_lock(&ami304mid_data.ctrllock);
            memcpy(pedoparam, &ami304mid_data.pedometerparam[0], sizeof(pedoparam));
            read_unlock(&ami304mid_data.ctrllock);          
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_to_user(data, pedoparam, sizeof(pedoparam))) {
                retval = -EFAULT;
                goto err_out;
            }                   
            break;
            
        case AMI304DAE_IOCTL_SET_PEDOPARAM:
            data = (void __user *) arg;         
            if (data == NULL)
                break;  
            if (copy_from_user(pedoparam, data, sizeof(pedoparam))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            memcpy(&ami304mid_data.pedometerparam[0], pedoparam, sizeof(pedoparam));
            write_unlock(&ami304mid_data.ctrllock);                 
            break;  

        case AMI304DAE_IOCTL_GET_CONTROL:
            read_lock(&ami304mid_data.ctrllock);
            memcpy(controlbuf, &ami304mid_data.controldata[0], sizeof(controlbuf));
            read_unlock(&ami304mid_data.ctrllock);          
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
                retval = -EFAULT;
                goto err_out;
            }                   
            break;      
            
        case AMI304DAE_IOCTL_SET_CONTROL:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
                retval = -EFAULT;
                goto err_out;
            }
            write_lock(&ami304mid_data.ctrllock);
            memcpy(&ami304mid_data.controldata[0], controlbuf, sizeof(controlbuf));
            write_unlock(&ami304mid_data.ctrllock);
            Set_Report_Sensor_Flag(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS]);
	    			if(!atomic_read(&m_status) && !atomic_read(&g_status) && !atomic_read(&rv_status) && !atomic_read(&la_status) && !atomic_read(&gv_status) &&!atomic_read(&o_status) && !atomic_read(&off_status_hal))//power off
	    			{//power off
	    				atomic_set(&off_status_hal, 1);				
	    				return AMI304_Chipset_StandBy();
	    			}
	    			else if((atomic_read(&m_status) || (atomic_read(&g_status) || atomic_read(&rv_status) || atomic_read(&la_status) || atomic_read(&gv_status) || atomic_read(&o_status)))  && atomic_read(&off_status_hal))//power on			
	    			{//power on
	    				atomic_set(&off_status_hal, 0);		
	    				return AMI304_Chipset_Active();
	    			}			
            break;  
    
        case AMI304DAE_IOCTL_SET_MODE:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(&mode, data, sizeof(mode))) {
                retval = -EFAULT;
                goto err_out;
            }       
            AMI304_SetMode(mode);               
            break;
        
        //Add for input_device sync         
        case AMI304DAE_IOCTL_SET_REPORT:
            AMI304_Report_Value();
            break;
        
        case AMI304DAE_IOCTL_GET_WIA:
            data = (void __user *) arg;
            if (data == NULL)
                break;      
            retval = AMI304_WIA(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}
	    			}
            break;

        case AMI304DAE_IOCTL_GET_AXISINTERFERENCE:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadAxisInterference(strbuf, AMI304_BUFSIZE);    
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}                   
	    			}
            break;

				case AMI304DAE_IOCTL_GET_DIRPOLARITY:	
            read_lock(&ami304mid_data.ctrllock);
            memcpy(dirpolarity, &ami304mid_data.dirpolarity[0], sizeof(dirpolarity));
            read_unlock(&ami304mid_data.ctrllock);          
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_to_user(data, dirpolarity, sizeof(dirpolarity))) {
                retval = -EFAULT;
                goto err_out;
            }                   
            break;

        case AMI304DAE_IOCTL_SET_ROTATION_VECTOR:
            data = (void __user *) arg;         
            if (data == NULL)
                break;  
            if (copy_from_user(rotation_vector, data, sizeof(rotation_vector))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            memcpy(&ami304mid_data.rotationvector[0], rotation_vector, sizeof(rotation_vector));
            write_unlock(&ami304mid_data.ctrllock);                 
						break;			  

	    	case AMI304DAE_IOCTL_SET_LINEAR_ACCEL:
            data = (void __user *) arg;         
            if (data == NULL)
                break;  
            if (copy_from_user(ladata, data, sizeof(ladata))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            ami304mid_data.linear_accel.x = ladata[0];
            ami304mid_data.linear_accel.y = ladata[1];
            ami304mid_data.linear_accel.z = ladata[2];
            write_unlock(&ami304mid_data.ctrllock);                 
            break;  			
			
        case AMI304DAE_IOCTL_SET_GRAVITY:
            data = (void __user *) arg;         
            if (data == NULL)
                break;  
            if (copy_from_user(gravitydata, data, sizeof(gravitydata))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            ami304mid_data.gravity.x = gravitydata[0];
            ami304mid_data.gravity.y = gravitydata[1];
            ami304mid_data.gravity.z = gravitydata[2];
            write_unlock(&ami304mid_data.ctrllock);                 
            break;  	
			
				case AMI304DAE_IOCTL_GET_SECURITY:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
	    			retval = AMI304_ReadSecurity(strbuf, AMI304_BUFSIZE);	
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}  
	    			}
	    			break;

				case AMI304DAE_IOCTL_SET_I2CDATA:
						data = (void __user *)arg;
						if (data == NULL)
							break;
						if (copy_from_user(i2cwrdata, data, sizeof(i2cwrdata))) {
							retval = -EFAULT;
							goto err_out;
						}
						//write buf order is reg_addr, buf_len(unit:byte), data 
						retval = AMI304_I2C_Write(i2cwrdata[0], i2cwrdata[1], &i2cwrdata[2]);
						break;
		
				case AMI304DAE_IOCTL_SET_I2CADDR:
						data = (void __user *)arg;
						if (data == NULL)
							break;
						if (copy_from_user(i2creaddata, data, sizeof(i2creaddata))) {
							retval = -EFAULT;
							goto err_out;
						}
						read_lock(&ami304_data.lock);
    				ami304_data.i2c_read_addr = i2creaddata[0];
    				ami304_data.i2c_read_len = i2creaddata[1];
            read_unlock(&ami304_data.lock);			
						break;

				case AMI304DAE_IOCTL_GET_I2CDATA:
						data = (void __user *)arg;
						if (data == NULL)
							break;
						retval = AMI304_I2C_Read(ami304_data.i2c_read_addr, ami304_data.i2c_read_len, &i2cwrdata[0]);
						if( !retval )  {//successful
							if (copy_to_user(data, i2cwrdata, ami304_data.i2c_read_len)) {
								retval = -EFAULT;
								goto err_out;
							}		
						}
						break;
            
        default:
            printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
            retval = -ENOIOCTLCMD;
            break;
    }
    
err_out:
    return retval;  
}

unsigned int ami304daemon_poll(struct file* filp , poll_table * pwait)
{
    unsigned int mask=0; 

    if( (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ORIENTATION) || 
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GYROSCOPE) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_MAGNETIC_FIELD) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ACCELEROMETER) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ROTATION_VECTOR) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_LINEAR_ACCELERATION) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GRAVITY) ) 		 
    		{
       				mask |= POLLIN | POLLRDNORM;
    		}
    		return mask;
}

static ssize_t ami304daemon_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
    char strbuf[AMI304_BUFSIZE];
    struct input_event event[3];
    int retval = 0;
    int mx, my, mz;

    if( (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ORIENTATION) || 
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GYROSCOPE) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_MAGNETIC_FIELD) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ACCELEROMETER) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ROTATION_VECTOR) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_LINEAR_ACCELERATION) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GRAVITY) )
				{	
						memset(strbuf, 0x00, sizeof(strbuf)); 
						memset(event, 0x00, sizeof(event));
						retval = AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);			
						if( !retval )  {//successful
							//if (copy_to_user(buff, strbuf, strlen(strbuf)+1)) {
							//	retval = -EFAULT;
							//	goto ami304daemon_read_err_out;
							//}
							 /* Most read functions return the number of bytes put into the buffer */
							//retval = strlen(strbuf)+1;
							sscanf(strbuf, "%x %x %x", &mx, &my, &mz); 
							
							do_gettimeofday(&event[0].time);
							event[0].value = mx;
							event[0].code = ABS_HAT0X;
							event[0].type = EV_ABS;
							
							event[1].time = event[0].time;
							event[1].value = my;
							event[1].code = ABS_HAT0Y;
							event[1].type = EV_ABS;
    				
							event[2].time = event[0].time;
							event[2].value = mz;
							event[2].code = ABS_BRAKE;
							event[2].type = EV_ABS;
    				
							if (copy_to_user(buff, event, sizeof(event))) {
								retval = -EIO;
								goto ami304daemon_read_err_out;
							} 
							retval = sizeof(event);						
						}
    		} //if( [AMI304_CB_ACTIVESENSORS] )
		
ami304daemon_read_err_out:
	return retval;
}

static int ami304hal_open(struct inode *inode, struct file *file)
{
    //atomic_inc_and_test(&hal_open_count);
    //printk(KERN_INFO "Open device node:ami304hal %d times.\n", atomic_read(&hal_open_count));   
    return 0;
}

static int ami304hal_release(struct inode *inode, struct file *file)
{
    //atomic_dec_and_test(&hal_open_count);
    //printk(KERN_INFO "Release ami304hal, remainder is %d times.\n", atomic_read(&hal_open_count));  
    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
static long ami304hal_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
#else
static int ami304hal_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
#endif
{
    int controlbuf[AMI304_CB_LENGTH];
    char strbuf[AMI304_BUFSIZE];
    int pedoparam[AMI304_PD_LENGTH];        
    void __user *data;
    int retval=0;
        
    switch (cmd) {
        
        case AMI304HAL_IOCTL_GET_SENSORDATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}       
	    			}
            break;
                                    
        case AMI304HAL_IOCTL_GET_POSTURE:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}               
	    			}
            break;          
     
        case AMI304HAL_IOCTL_GET_CALIDATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}               
	    			}
            break;

        case AMI304HAL_IOCTL_GET_GYRODATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadGyroData(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}               
	    			}
            break;
            
        case AMI304HAL_IOCTL_GET_PEDODATA:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadPedoData(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}                       
	    			}
            break;

        case AMI304HAL_IOCTL_GET_PEDOPARAM:
            read_lock(&ami304mid_data.ctrllock);
            memcpy(pedoparam, &ami304mid_data.pedometerparam[0], sizeof(pedoparam));
            read_unlock(&ami304mid_data.ctrllock);          
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_to_user(data, pedoparam, sizeof(pedoparam))) {
                retval = -EFAULT;
                goto err_out;
            }           
            break;
            
       case AMI304HAL_IOCTL_SET_PEDOPARAM:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(pedoparam, data, sizeof(pedoparam))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            memcpy(&ami304mid_data.pedometerparam[0], pedoparam, sizeof(pedoparam));
            write_unlock(&ami304mid_data.ctrllock);
            break;  
            
        case AMI304HAL_IOCTL_GET_CONTROL:
            read_lock(&ami304mid_data.ctrllock);
            memcpy(controlbuf, &ami304mid_data.controldata[0], sizeof(controlbuf));
            read_unlock(&ami304mid_data.ctrllock);          
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
                retval = -EFAULT;
                goto err_out;
            }           
            break;

        case AMI304HAL_IOCTL_SET_CONTROL:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
                retval = -EFAULT;
                goto err_out;
            }   
            write_lock(&ami304mid_data.ctrllock);
            memcpy(&ami304mid_data.controldata[0], controlbuf, sizeof(controlbuf));
            write_unlock(&ami304mid_data.ctrllock);
            Set_Report_Sensor_Flag(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS]);
	    			if(!atomic_read(&m_status) && !atomic_read(&g_status) && !atomic_read(&rv_status) && !atomic_read(&la_status) && !atomic_read(&gv_status) &&!atomic_read(&o_status) && !atomic_read(&off_status_hal))//power off
	    			{//power off
	    				atomic_set(&off_status_hal, 1);				
	    				return AMI304_Chipset_StandBy();
	    			}
	    			else if((atomic_read(&m_status) || (atomic_read(&g_status) || atomic_read(&rv_status) || atomic_read(&la_status) || atomic_read(&gv_status) || atomic_read(&o_status)))  && atomic_read(&off_status_hal))//power on			
	    			{//power on
	    				atomic_set(&off_status_hal, 0);		
	    				return AMI304_Chipset_Active();
	    			}			
            break;

        case AMI304HAL_IOCTL_GET_WIA:
            data = (void __user *) arg;
            if (data == NULL)
                break;      
            retval = AMI304_WIA(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}
	    			}
            break;

        case AMI304HAL_IOCTL_GET_ROTATION_VECTOR:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadRotationVector(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}               
	    			}
            break;        
			
        case AMI304HAL_IOCTL_GET_LINEAR_ACCEL:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadLinearAccel(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}               
	    			}
            break;        
			
        case AMI304HAL_IOCTL_GET_GRAVITY:
            data = (void __user *) arg;
            if (data == NULL)
                break;  
            retval = AMI304_ReadGravity(strbuf, AMI304_BUFSIZE);
	    			if( !retval )  {//successful
	    				if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
	    					retval = -EFAULT;
	    					goto err_out;
	    				}               
	    			}
            break;        						
			
        default:
            printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
            retval = -ENOIOCTLCMD;
            break;
    }
    
err_out:
    return retval;  
}

unsigned int ami304hal_poll(struct file* filp , poll_table * pwait)
{
    unsigned int mask=0; 

    if( (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ORIENTATION) || 
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GYROSCOPE) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_MAGNETIC_FIELD) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ACCELEROMETER) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ROTATION_VECTOR) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_LINEAR_ACCELERATION) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GRAVITY) ) 		
    		{
       			mask |= POLLIN | POLLRDNORM;
    		}
    		return mask;
}

static ssize_t ami304hal_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
    char strbuf[AMI304_BUFSIZE];
    struct input_event event[3];
    int retval = 0;
    int mx, my, mz;

    if( (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ORIENTATION) || 
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GYROSCOPE) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_MAGNETIC_FIELD) || 
        (ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ACCELEROMETER) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_ROTATION_VECTOR) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_LINEAR_ACCELERATION) ||
				(ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] & AMIT_BIT_GRAVITY) )
				{	
							memset(strbuf, 0x00, sizeof(strbuf)); 
							memset(event, 0x00, sizeof(event));
							retval = AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);			
							if( !retval )  {//successful
								//if (copy_to_user(buff, strbuf, strlen(strbuf)+1)) {
								//	retval = -EFAULT;
								//	goto ami304hal_read_err_out;
								//}
								 /* Most read functions return the number of bytes put into the buffer */
								//retval = strlen(strbuf)+1;
								sscanf(strbuf, "%x %x %x", &mx, &my, &mz); 
								do_gettimeofday(&event[0].time);
								event[0].value = mx;
								event[0].code = ABS_HAT0X;
								event[0].type = EV_ABS;
								
								event[1].time = event[0].time;
								event[1].value = my;
								event[1].code = ABS_HAT0Y;
								event[1].type = EV_ABS;
    					
								event[2].time = event[0].time;
								event[2].value = mz;
								event[2].code = ABS_BRAKE;
								event[2].type = EV_ABS;
    					
								if (copy_to_user(buff, event, sizeof(event))) {
									retval = -EIO;
									goto ami304hal_read_err_out;
								} 
								retval = sizeof(event);						
							}
    		} //if( [AMI304_CB_ACTIVESENSORS] )
		
ami304hal_read_err_out:
	return retval;
}

static struct file_operations ami304_fops = {
    .owner = THIS_MODULE,
    .open = ami304_open,
    .release = ami304_release,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)    
		.unlocked_ioctl =	ami304_ioctl,
#else
    .ioctl = ami304_ioctl,
#endif    
    .poll = ami304_poll,
		.read = ami304_read,
};

static struct miscdevice ami304_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ami304",
    .fops = &ami304_fops,
};

static struct file_operations ami304daemon_fops = {
    .owner = THIS_MODULE,
    .open = ami304daemon_open,
    .release = ami304daemon_release,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)    
		.unlocked_ioctl =	ami304daemon_ioctl,
#else
    .ioctl = ami304daemon_ioctl,
#endif    
    .poll = ami304daemon_poll,
		.read = ami304daemon_read,
};

static struct miscdevice ami304daemon_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ami304daemon",
    .fops = &ami304daemon_fops,
};

static struct file_operations ami304hal_fops = {
    .owner = THIS_MODULE,
    .open = ami304hal_open,
    .release = ami304hal_release,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)    
		.unlocked_ioctl =	ami304hal_ioctl,
#else    
    .ioctl = ami304hal_ioctl,
#endif    
    .poll = ami304hal_poll,
		.read = ami304hal_read,
};

static struct miscdevice ami304hal_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ami304hal",
    .fops = &ami304hal_fops,
};

static int ami304_input_init(struct ami304_i2c_data *data)
{
    int err=0;
    
    data->input_dev_compass = input_allocate_device();
    if (!data->input_dev_compass) {
        err = -ENOMEM;
        printk(KERN_ERR
               "ami304_i2c_detect: Failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }
    set_bit(EV_ABS, data->input_dev_compass->evbit);
    /* yaw */
    input_set_abs_params(data->input_dev_compass, ABS_RX, 0, (360*AMI304_POSTURE_ACCURACY_RATE), 0, 0);
    /* pitch */
    input_set_abs_params(data->input_dev_compass, ABS_RY, -(180*AMI304_POSTURE_ACCURACY_RATE), (180*AMI304_POSTURE_ACCURACY_RATE), 0, 0);
    /* roll */
    input_set_abs_params(data->input_dev_compass, ABS_RZ, -(90*AMI304_POSTURE_ACCURACY_RATE), (90*AMI304_POSTURE_ACCURACY_RATE), 0, 0);
    /* status of orientation sensor */  
    input_set_abs_params(data->input_dev_compass, ABS_RUDDER, 0, 5, 0, 0);
    
    /* x-axis of raw acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_X, -(1000*AMI304_ACCELEROMETER_SENSITIVITY), (1000*AMI304_ACCELEROMETER_SENSITIVITY), 0, 0);
    //input_set_abs_params(data->input_dev_compass, ABS_X, -(1000*n), (1000*n), 0, 0);//the range is -ng to +ng
    /* y-axis of raw acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_Y, -(1000*AMI304_ACCELEROMETER_SENSITIVITY), (1000*AMI304_ACCELEROMETER_SENSITIVITY), 0, 0);
    //input_set_abs_params(data->input_dev_compass, ABS_Y, -(1000*n), (1000*n), 0, 0);//the range is -ng to +ng
    /* z-axis of raw acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_Z, -(1000*AMI304_ACCELEROMETER_SENSITIVITY), (1000*AMI304_ACCELEROMETER_SENSITIVITY), 0, 0);
    //input_set_abs_params(data->input_dev_compass, ABS_Z, -(1000*n), (1000*n), 0, 0);////the range is -ng to +ng

    /* x-axis of raw magnetic vector and the range is -3g to +3g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT0X, -(1000*AMI304_MEGNETIC_SENSITIVITY), (1000*AMI304_MEGNETIC_SENSITIVITY), 0, 0);
    /* y-axis of raw magnetic vector and the range is -3g to +3g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT0Y, -(1000*AMI304_MEGNETIC_SENSITIVITY), (1000*AMI304_MEGNETIC_SENSITIVITY), 0, 0);
    /* z-axis of raw magnetic vector and the range is -3g to +3g */
    input_set_abs_params(data->input_dev_compass, ABS_BRAKE, -(1000*AMI304_MEGNETIC_SENSITIVITY), (1000*AMI304_MEGNETIC_SENSITIVITY), 0, 0);
    /* status of magnetic sensor */
    input_set_abs_params(data->input_dev_compass, ABS_WHEEL, 0, 5, 0, 0);  

    /* x-axis of rotation vector */
    input_set_abs_params(data->input_dev_compass, ABS_HAT3X, -1000000, 1000000, 0, 0);
    /* y-axis of rotation vector */
    input_set_abs_params(data->input_dev_compass, ABS_HAT3Y, -1000000, 1000000, 0, 0);
    /* z-axis of rotation vector */
    input_set_abs_params(data->input_dev_compass, ABS_TILT_X, -1000000, 1000000, 0, 0);
	/* theta of rotation vector */
	input_set_abs_params(data->input_dev_compass, ABS_TILT_Y, -1000000, 1000000, 0, 0);	
	
    /* x-axis linear acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT1X, -(1000*AMI304_ACCELEROMETER_SENSITIVITY), (1000*AMI304_ACCELEROMETER_SENSITIVITY), 0, 0);
    /* y-axis linear acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT1Y, -(1000*AMI304_ACCELEROMETER_SENSITIVITY), (1000*AMI304_ACCELEROMETER_SENSITIVITY), 0, 0);
    /* z-axis linear acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_TOOL_WIDTH, -(1000*AMI304_ACCELEROMETER_SENSITIVITY), (1000*AMI304_ACCELEROMETER_SENSITIVITY), 0, 0);

	/* x-axis gravity and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT2X, -(1000*AMI304_ACCELEROMETER_SENSITIVITY), (1000*AMI304_ACCELEROMETER_SENSITIVITY), 0, 0);
    /* y-axis gravity and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT2Y, -(1000*AMI304_ACCELEROMETER_SENSITIVITY), (1000*AMI304_ACCELEROMETER_SENSITIVITY), 0, 0);
    /* z-axis gravity and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_VOLUME, -(1000*AMI304_ACCELEROMETER_SENSITIVITY), (1000*AMI304_ACCELEROMETER_SENSITIVITY), 0, 0);	

    data->input_dev_compass->name = "ami30x_compass";
    //register input device
    err = input_register_device(data->input_dev_compass);
    if (err) {
        printk(KERN_ERR
               "ami304_i2c_detect: Unable to register input device: %s\n",
               data->input_dev_compass->name);
        goto exit_input_register_compass_device_failed;
    }
	
    data->input_dev_gyroscope = input_allocate_device();
    if (!data->input_dev_gyroscope) {
        err = -ENOMEM;
        printk(KERN_ERR
               "ewtzmu2_input_init: Failed to allocate input device: %s\n",
               data->input_dev_gyroscope->name);
        goto exit_input_register_compass_device_failed;
    }
    set_bit(EV_REL, data->input_dev_gyroscope->evbit);
    /* x-axis of gyro sensor */
    //input_set_abs_params(data->input_dev_gyroscope, REL_RX, -50000, 50000, 0, 0);
    /* y-axis of gyro sensor */
    //input_set_abs_params(data->input_dev_gyroscope, REL_RY, -50000, 50000, 0, 0);
    /* z-axis of gyro sensor */
    //input_set_abs_params(data->input_dev_gyroscope, REL_RZ, -50000, 50000, 0, 0);
    data->input_dev_gyroscope->relbit[0] = BIT(REL_RX) | BIT(REL_RY) | BIT(REL_RZ);   

    data->input_dev_gyroscope->name = "ami30x_gyroscope";
    //register input device
    err = input_register_device(data->input_dev_gyroscope);
    if (err) {
        printk(KERN_ERR
               "ewtzmu2_input_init: Unable to register input device: %s\n",
               data->input_dev_gyroscope->name);
        goto exit_input_register_gyro_device_failed;
    }
	
    return 0;
exit_input_register_gyro_device_failed:
    input_free_device(data->input_dev_gyroscope); 
    
exit_input_register_compass_device_failed:
    input_free_device(data->input_dev_compass);
 
exit_input_dev_alloc_failed:
    return err; 	
}

static void ami304_dir_polarity(void)
{
   ami304mid_data.dirpolarity[0] = ACC_DIR;
   ami304mid_data.dirpolarity[1] = ACC_POLARITY;		
   ami304mid_data.dirpolarity[2] = GYRO_DIR;
   ami304mid_data.dirpolarity[3] = GYRO_POLARITY;
   ami304mid_data.dirpolarity[4] = MAG_DIR;
   ami304mid_data.dirpolarity[5] = MAG_POLARITY;		
   //for(i=0;i<6;i++){
   //	printk(KERN_INFO "$$$ewtzmu2_dir_polarity [%d]=%d \n",i,ami304mid_data.dirpolarity[i]);
   //}
}

// developers need to add ami304 i2c address at i2c_board_info structure 
// under arch/arm/chipset-name (ex:samsung/arch/arm/mach-s5pv210 for N-S)
// Ex:
// static struct i2c_board_info i2c_devs1[] __initdata = {
//        {
//                I2C_BOARD_INFO("ami30x", i2c_address),
//        },
//};

static int ami304_suspend(struct i2c_client *client, pm_message_t mesg)
{
    if( !atomic_read(&off_status) )
    {
    	atomic_set(&off_status, 1);
    	return AMI304_Chipset_StandBy();
    }
    return 0;
}

static int ami304_resume(struct i2c_client *client)
{
   if( atomic_read(&off_status) ){
   	atomic_set(&off_status, 0);	
   	return AMI304_Chipset_Active();
   }
   return 0;
}

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void ami304_early_suspend(struct early_suspend *handler)
{
	struct ami304_i2c_data *data;
	data = container_of(handler, struct ami304_i2c_data, early_suspend);
	ami304_suspend(data->client, PMSG_SUSPEND);
}

static void ami304_early_resume(struct early_suspend *handler)
{
	struct ami304_i2c_data *data;
	data = container_of(handler, struct ami304_i2c_data, early_suspend);
	ami304_resume(data->client);
}
#elif defined(CONFIG_FB)
static void ami304_early_suspend(void)
{
	printk(KERN_INFO "[AMI306] early-suspend ++\n");
	ami304_suspend(ami304_i2c_client, PMSG_SUSPEND);
	printk(KERN_INFO "[AMI306] early-suspend --\n");
}

static void ami304_early_resume(void)
{
	printk(KERN_INFO "[AMI306] late-resume ++\n");
	ami304_resume(ami304_i2c_client);
	printk(KERN_INFO "[AMI306] late-resume --\n");
}

static int ami304_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	static int blank_old = 0;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			if (blank_old == FB_BLANK_POWERDOWN) {
				blank_old = FB_BLANK_UNBLANK;
				ami304_early_resume();
			}
		} else if (*blank == FB_BLANK_POWERDOWN) {
			if (blank_old == 0 || blank_old == FB_BLANK_UNBLANK) {
				blank_old = FB_BLANK_POWERDOWN;
				ami304_early_suspend();
			}
		}
	}	
	return 0;
}
#endif
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
static void init_ami306_setting(void)
{
	printk(KERN_INFO "AMI304 sensor driver: init\n");
    printk(KERN_INFO "ami304: driver version:%s\n",DRIVER_VERSION);

    rwlock_init(&ami304mid_data.ctrllock);
    rwlock_init(&ami304mid_data.datalock);
    rwlock_init(&ami304_data.lock);
    memset(&ami304mid_data.controldata[0], 0, sizeof(int)*AMI304_CB_LENGTH);
    
    ami304mid_data.controldata[AMI304_CB_LOOPDELAY] = AMI30x_DEFAULT_POLLING_TIME;  // Loop Delay
    ami304mid_data.controldata[AMI304_CB_RUN] =           1;   // Run   
    ami304mid_data.controldata[AMI304_CB_ACCCALI] =       0;   // Start-AccCali
    ami304mid_data.controldata[AMI304_CB_MAGCALI] =       1;   // Start-MagCali
    ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] = 0;   // Active Sensors
    ami304mid_data.controldata[AMI304_CB_PD_RESET] =      0;   // Pedometer not reset    
    ami304mid_data.controldata[AMI304_CB_PD_EN_PARAM] =   0;   // Disable parameters of Pedometer
	memset(&ami304mid_data.dirpolarity[0], 0, sizeof(int)*AMI304_DP_LENGTH);
    memset(&ami304mid_data.pedometerparam[0], 0, sizeof(int)*AMI304_PD_LENGTH);
	
    ami304_data.chipset = ami304_data.mode = ami304_data.i2c_read_addr = ami304_data.i2c_read_len = ami304_data.updated = 0;

    
    atomic_set(&dev_open_count, 0); 
    atomic_set(&daemon_open_count, 0);
    //atomic_set(&hal_open_count, 0);
	
    atomic_set(&o_status, 0);
    atomic_set(&a_status, 0);
    atomic_set(&m_status, 0);
    atomic_set(&g_status, 0);
    atomic_set(&rv_status, 0);
	atomic_set(&la_status, 0);
	atomic_set(&gv_status, 0);	
	
	atomic_set(&off_status, 0);
	atomic_set(&off_status_hal, 0);
    
	ami304_data.i2c_read_addr = ami304_data.i2c_read_len = 0;
}
static void clear_ami306_setting(void)
{
	atomic_set(&dev_open_count, 0);
    atomic_set(&daemon_open_count, 0);
    //atomic_set(&hal_open_count, 0);
	
    atomic_set(&o_status, 0);
    atomic_set(&a_status, 0);
    atomic_set(&m_status, 0);
    atomic_set(&g_status, 0);
    atomic_set(&rv_status, 0);
	atomic_set(&la_status, 0);
	atomic_set(&gv_status, 0);

	atomic_set(&off_status, 0);
	atomic_set(&off_status_hal, 0);	
	
    ami304_data.i2c_read_addr = ami304_data.i2c_read_len = 0;
}

//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
static int __devinit ami304_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ami304_i2c_data *data;
    int err = 0;

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	init_ami306_setting();
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
    printk(KERN_INFO "\n\nEnter ami304_i2c_probe!!\n");
        
    if (!(data = kmalloc(sizeof(struct ami304_i2c_data), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }
    memset(data, 0, sizeof(struct ami304_i2c_data));

    data->client = client;
    i2c_set_clientdata(client, data);
    ami304_i2c_client = data->client;   

    //Now, AMIT support AMI304 and AMI306 in this driver
    if( (err=Identify_AMI_Chipset())!=0 )  
    {
        printk(KERN_INFO "Failed to identify AMI_Chipset!\n");  
        return err;
    }
        
    err = AMI304_Chipset_Init(AMI304_FORCE_MODE, ami304_data.chipset); // default is Force State  
    if(err)
        goto exit_kfree;	
    dev_info(&client->dev, "%s operating mode\n", ami304_data.mode? "force" : "normal");
    
    printk(KERN_INFO "Register input device!\n");   
    err = ami304_input_init(data);
    if(err)
        goto exit_kfree;

	//set sensor dir and polarity	
	ami304_dir_polarity();
		
    //register misc device:ami304          
    err = misc_register(&ami304_device);
    if (err) {
        printk(KERN_ERR
               "ami304_device register failed\n");
        goto exit_misc_device_register_failed;
    }   
    //register misc device:ami304daemon 
    err = misc_register(&ami304daemon_device);
    if (err) {
        printk(KERN_ERR
               "ami304daemon_device register failed\n");
        goto exit_misc_device_register_failed;
    }   
    //register misc device:ami304hal
    err = misc_register(&ami304hal_device);
    if (err) {
        printk(KERN_ERR
               "ami304hal_device register failed\n");
        goto exit_misc_device_register_failed;
    }   

    /* Register sysfs hooks */
    err = sysfs_create_group(&client->dev.kobj, &ami304_attribute_group);
    if (err)
        goto exit_sysfs_create_group_failed;

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"
#if defined(CONFIG_HAS_EARLYSUSPEND)
    data->early_suspend.suspend = ami304_early_suspend;
    data->early_suspend.resume = ami304_early_resume;
    register_early_suspend(&data->early_suspend);
#elif defined(CONFIG_FB)
	e_compass_fb_notif.notifier_call = ami304_fb_notifier_callback;
	err = fb_register_client(&e_compass_fb_notif);
	if (err)
		printk(KERN_INFO "[AMI306] Unable to register fb_notifier: %d\n", err); 
#endif
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	ecompass_status = 1;
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
    return 0;
exit_sysfs_create_group_failed: 
exit_misc_device_register_failed:
    input_free_device(data->input_dev_compass);
	input_free_device(data->input_dev_gyroscope);
exit_kfree: 
    kfree(data);
exit:
    return err;
}

static int __devexit ami304_i2c_remove(struct i2c_client *client)
{
    struct ami304_i2c_data *data = i2c_get_clientdata(client);
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"    
    clear_ami306_setting();
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
    sysfs_remove_group(&client->dev.kobj, &ami304_attribute_group);
    input_unregister_device(data->input_dev_compass);
    kfree(i2c_get_clientdata(client));
    ami304_i2c_client = NULL;   
    misc_deregister(&ami304hal_device);
    misc_deregister(&ami304daemon_device);
    misc_deregister(&ami304_device);
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"
#if defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&data->early_suspend);
#elif defined(CONFIG_FB)
	fb_unregister_client(&e_compass_fb_notif);
#endif
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"
    return 0;
}

struct i2c_device_id ami304_idtable[] = {
    { "ami30x", 0 },
    {}
};

MODULE_DEVICE_TABLE(i2c, ami304_idtable);

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
#ifdef CONFIG_OF
static struct of_device_id ami306_match_table[] = {
	{ .compatible = "aichi,ami306",},
	{},
};
#else
#define ami306_match_table NULL
#endif
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"

static struct i2c_driver ami304_i2c_driver = {
    .probe          = ami304_i2c_probe,
    .remove         = __devexit_p(ami304_i2c_remove),
    .id_table       = ami304_idtable,
    .driver = {
        .name   = AMI304_DRV_NAME,
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
		.of_match_table = ami306_match_table,
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
    },
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"
#if (!defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_FB))
	.suspend		= ami304_suspend,
	.resume			= ami304_resume,	
#endif
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Support early-suspend"	
};
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
module_i2c_driver(ami304_i2c_driver);

#if 0
static int __init ami304_init(void)
{
    int ret;
    
    printk(KERN_INFO "AMI304 MI sensor driver: init\n");
    printk(KERN_INFO "ami304: driver version:%s\n",DRIVER_VERSION);

    rwlock_init(&ami304mid_data.ctrllock);
    rwlock_init(&ami304mid_data.datalock);
    rwlock_init(&ami304_data.lock);
    memset(&ami304mid_data.controldata[0], 0, sizeof(int)*AMI304_CB_LENGTH);
    
    ami304mid_data.controldata[AMI304_CB_LOOPDELAY] = AMI30x_DEFAULT_POLLING_TIME;  // Loop Delay
    ami304mid_data.controldata[AMI304_CB_RUN] =           1;   // Run   
    ami304mid_data.controldata[AMI304_CB_ACCCALI] =       0;   // Start-AccCali
    ami304mid_data.controldata[AMI304_CB_MAGCALI] =       1;   // Start-MagCali
    ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] = 0;   // Active Sensors
    ami304mid_data.controldata[AMI304_CB_PD_RESET] =      0;   // Pedometer not reset    
    ami304mid_data.controldata[AMI304_CB_PD_EN_PARAM] =   0;   // Disable parameters of Pedometer
	memset(&ami304mid_data.dirpolarity[0], 0, sizeof(int)*AMI304_DP_LENGTH);
    memset(&ami304mid_data.pedometerparam[0], 0, sizeof(int)*AMI304_PD_LENGTH);
	
    ami304_data.chipset = ami304_data.mode = ami304_data.i2c_read_addr = ami304_data.i2c_read_len = ami304_data.updated = 0;

    
    atomic_set(&dev_open_count, 0); 
    atomic_set(&daemon_open_count, 0);
    //atomic_set(&hal_open_count, 0);
	
    atomic_set(&o_status, 0);
    atomic_set(&a_status, 0);
    atomic_set(&m_status, 0);
    atomic_set(&g_status, 0);
    atomic_set(&rv_status, 0);
	atomic_set(&la_status, 0);
	atomic_set(&gv_status, 0);	
	
	atomic_set(&off_status, 0);
	atomic_set(&off_status_hal, 0);
    
	ami304_data.i2c_read_addr = ami304_data.i2c_read_len = 0;
	
    ret = i2c_add_driver(&ami304_i2c_driver);
    if ( ret != 0 ) {
        printk(KERN_INFO "can not add i2c driver\n");
        return ret;
    }
    
    return ret;
}

static void __exit ami304_exit(void)
{
    atomic_set(&dev_open_count, 0);
    atomic_set(&daemon_open_count, 0);
    //atomic_set(&hal_open_count, 0);
	
    atomic_set(&o_status, 0);
    atomic_set(&a_status, 0);
    atomic_set(&m_status, 0);
    atomic_set(&g_status, 0);
    atomic_set(&rv_status, 0);
	atomic_set(&la_status, 0);
	atomic_set(&gv_status, 0);

	atomic_set(&off_status, 0);
	atomic_set(&off_status_hal, 0);	
	
    ami304_data.i2c_read_addr = ami304_data.i2c_read_len = 0;
	
    i2c_del_driver(&ami304_i2c_driver);
}
#endif
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"

MODULE_AUTHOR("Kyle K.Y. Chen");
MODULE_DESCRIPTION("AMI304 MI-Sensor driver without DRDY");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
#if 0
module_init(ami304_init);
module_exit(ami304_exit);
#endif
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
