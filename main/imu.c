/*  
    TITLE:          imu
    VERSION:        See Git logs.
    FILENAME:       imu.c
    AURTHOR(S):     Isaac Shields
    PURPOSE:        Implements the functions defined in imu.h
    HOW TO LOAD:    DEV BOARD-
                    Build in the project directory using "idf.py build"  Load to
                    the board using "idf.py -p PORT flash".  PORT should be replaced
                    with the serial port you are on (COM1, COM2, etc.).
                    PRODUCT-
                    In progress.
    DATE STARTED:   3/6/2020
    UPDATE HISTORY: See Git logs.
    NOTES:          This program uses example code provided by Espessif Systems.
                    The GitHub containing this code is found here: 
                    https://github.com/espressif/esp-idf
*/

/*****************************************************************************/
/* INCLUDES */
/*****************************************************************************/

#include "imu.h"
#include "ui.h"
#include "driver/i2c.h"
#include "freertos/task.h"

/*****************************************************************************/
/* CONSTANT VARIABLES */
/*****************************************************************************/

/* lables for calibration data stored in nonolitile memory */
/*
static const char * const 
nvs_labels[NVS_LABEL_COUNT] =   {   
                                    "ACC_OFFSET_X_L", "ACC_OFFSET_X_M",
                                    "ACC_OFFSET_Y_L", "ACC_OFFSET_Y_M",
                                    "ACC_OFFSET_Z_L", "ACC_OFFSET_Z_M",
                                    "MAG_OFFSET_X_L", "MAG_OFFSET_X_M",
                                    "MAG_OFFSET_Y_L", "MAG_OFFSET_Y_M",
                                    "MAG_OFFSET_Z_L", "MAG_OFFSET_Z_M",
                                    "GYR_OFFSET_X_L", "GYR_OFFSET_X_M",
                                    "GYR_OFFSET_Y_L", "GYR_OFFSET_Y_M",
                                    "GYR_OFFSET_Z_L", "GYR_OFFSET_Z_M",
                                    "ACC_RADIUS_L", "ACC_RADIUS_M"
                                    "MAG_RADIUS_L", "MAG_RADIUS_M"
                                };
                                */
                                
/*****************************************************************************/
/* FUNCTION IMPLEMENTATIONS */
/*****************************************************************************/
void i2c_master_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void imu_write(uint8_t reg, uint8_t val)
{
    uint8_t data[2];
    data[0] = reg;
    data[1] = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, sizeof(data), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

void imu_read(char reg, uint8_t * data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

void imu_init()
{   
    
    /**************************************/
    /* PUT INTO CONFIG MODE */
    /**************************************/
    imu_write(OPR_MODE, CONFIG_MODE);
    
    /**************************************/
    /* DISABLE AUTOMATIC LOW POWER MODE*/
    /**************************************/
    // I'm pretty sure this is auto controlled in fusion mode.  This needs
    // to be tested.
    
    /**************************************/
    /* CONFIGURE ACCELEROMETER G RANGE*/
    /**************************************/
    // This defaults to +/-4G.  We'll try this setting for now but it may needed
    // to be changed later.  There is conflicting info on the datasheet about
    // whether or not I can change this in the fusion mode.  Intuitively,
    // I think it is probably available.
    // Accelerometer offsets are based on the G-Range.  The default is four.  If we change the default
    // we need to make sure to change this.
    
    /**************************************/
    /* LOAD CALIBRATION PROFILE */
    /**************************************/
    // Documentation says use of accelerometer and gyroscope only should provide accurate
    // results without calibration.  This will be added later if our results are not accurate
    // enough.
    /* create nvs handle for each piece of calibration data */
    // nvs_handle_t handles[NVS_LABEL_COUNT]; // uncomment when integrating into main program
    //for (int i = 0; i < NVS_LABEL_COUNT; i++)
    //{
        /* open nvs */ // add error handling
        //err = nvs_open(nvs_labels[i], NVS_READONLY, &handles[i]); // handle this error
        /* read nvs value */
        //err = nvs_get_u8(handles[i], nvs_labels[i], &temp);       // handle this error
        /* close nvs handle */
        //err = nvs_close(handles[i]);       // handle this error
        /* load calibration data to IMU */
        //imu_write(ACC_OFFSET_X_LSB + i, 0x55);
    //}

    /**************************************/
    /* PUT INTO SUSPEND MODE */
    /**************************************/
    imu_write(PWR_MODE, SUSPEND_MODE);
}

void read_data(uint8_t * buf, uint16_t len)
{
    int32_t space_left = len;   /* space left in buffer */
    uint16_t i = 0;             /* data pointer */
    uint8_t temp[LIA_REG_CNT];  /* temp buf for i2c read */
    int j;                      /* counter */
    uint8_t lite = 0;           /* light on or off */
    
    /**************************************/
    /* WAKE FROM SUSPEND AND PUT INTO IMU */
    /**************************************/
    imu_write(PWR_MODE, NORMAL_MODE);
    imu_write(OPR_MODE, IMU_MODE);
    vTaskDelay(10 / portTICK_PERIOD_MS); /* delay to let IMU get ready */
    
    /**************************************/
    /* READ UNTIL PRESS OR FULL */
    /**************************************/
    do
    {
        if (i % 10 == 0)                                /* handle led */
            lite ^= 1;
        gpio_set_level(RED_LED_PIN, lite);
        imu_read(LIA_DATA_X_LSB, temp, LIA_REG_CNT);    /* take sample */
        for (j = 0; j < LIA_REG_CNT; j++)               /* copy data to buffer */
        {
            buf[i] = temp[j];
            i++;
        }
        space_left -= LIA_REG_CNT;                      /* decrement buffer size */
    } while (   (gpio_get_level(START_STOP_PIN) == 1    ) /* start/stop is not pressed */
             && (space_left > (TAIL_SZ * 2)             ) /* there is enough space in the buffer */
            );
    
    /**************************************/
    /* ADD PACKET TAIL */
    /**************************************/
    for(j = 0; j < TAIL_SZ; j++)
    {
        buf[i] = PKT_TAIL;
        i++;
    }
    gpio_set_level(RED_LED_PIN, 1); /* turn light back on */
}
