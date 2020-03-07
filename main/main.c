/*  
    TITLE:          Main
    VERSION:        See Git logs.
    FILENAME:       main.c
    AURTHOR(S):     Isaac Shields
    PURPOSE:        Handles data reading, processing, and sending on the device.
                    The program waits for the host (smartphone) to connect.  Once
                    connected, it waits for the user to press a button.  The button
                    press starts IMU data reading.  When the button is pressed again,
                    the program stop recording, prcesses the data, and send it to
                    the host.
    HOW TO LOAD:    DEV BOARD-
                    Build in the project directory using "idf.py build"  Load to
                    the board using "idf.py -p PORT flash".  PORT should be replaced
                    with the serial port you are on (COM1, COM2, etc.).
                    PRODUCT-
                    In progress.
    DATE STARTED:   2/24/2020
    UPDATE HISTORY: See Git logs.
    NOTES:          This program uses example code provided by Espessif Systems.
                    The GitHub containing this code is found here: 
                    https://github.com/espressif/esp-idf
*/

/*****************************************************************************/
/* INCLUDES */
/*****************************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "nvs.h"

/*****************************************************************************/
/* DEFINITIONS */
/*****************************************************************************/

/* APP_MAIN */
#define KB                  (1024)                      /* size of 1KB in bytes */
#define LOOP_SIZE           ((40) * KB)                 /* 40KB for loop task */
#define LOOP_PRIORITY       (10)                        /* priority of the loop task */
#define IMU_CALIB_SIZE      (2048)                      /* size of memory allocated to the loop_task() task */
#define IMU_CALIB_PRIORITY  (10)                        /* priority of the loop_task() task */


/* START/START PIN */
#define START_STOP_PIN   (33)                       /* use pin 33 */
#define START_STOP_MASK  (1ULL<<START_STOP_PIN)     /* create bit mask for setup call */

/* LOOP TASK */


/* IMU CALIB TASK */
#define NVS_LABEL_COUNT (14)    /* number of NVS labels */

/* IMU */

#define PWR_MODE            (0x3E)                  /* power mode register */
#define NORMAL_MODE         (0x00)
#define SUSPEND_MODE        (0x02)

#define OPR_MODE            (0x3D)                  /* operation mode register */
#define CONFIG_MODE         (0x00)
#define IMU_MODE            (0x08)

#define CALIB_STAT          (0x35)                  /* calibration status register */
#define ST_SYS1             (1 << 6)
#define ST_SYS2             (1 << 7)

#define ACC_OFFSET_X_LSB    (0x55)                  /* calibration offset registers */

#define LIA_DATA_X_LSB      (0X28)                  /* linear acceleration data registers */
#define LIA_REG_CNT         (0x06)

/* LEDs */
#define BLUE_LED_PIN    (16)
#define BLUE_LED_MASK   (1ULL<<BLUE_LED_PIN)
#define RED_LED_PIN     (5)
#define RED_LED_MASK    (1ULL<<RED_LED_PIN)

/* I2C */
#define I2C_FREQ                    (5000)                      /* clock speed */
#define I2C_MASTER_TX_BUF_DISABLE   (0)                          /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   (0)                          /* I2C master doesn't need buffer */
#define IMU_I2C_ADDR                (0x28)
#define ACK_CHECK_EN                (0x1)                        /* I2C master will check ack from slave*/
#define ACK_CHECK_DIS               (0x0)                        /* I2C master will not check ack from slave */
#define ACK_VAL                     (0x0)
#define NACK_VAL                    (0x1)
#define BUF_SIZE                    (128)
/*****************************************************************************/
/* CONSTANTS */
/*****************************************************************************/

/* IMU CONFIG */
/* labels used for non-volatile memory storage for accelerometer calibration */
static const char * const 
nvs_labels[NVS_LABEL_COUNT] =   {                   
                                    "ACC_OFFSET_X_L", "ACC_OFFSET_X_M",
                                    "ACC_OFFSET_Y_L", "ACC_OFFSET_Y_M",
                                    "ACC_OFFSET_Z_L", "ACC_OFFSET_Z_M",
                                    "GYR_OFFSET_X_L", "GYR_OFFSET_X_M",
                                    "GYR_OFFSET_Y_L", "GYR_OFFSET_Y_M",
                                    "GYR_OFFSET_Z_L", "GYR_OFFSET_Z_M",
                                    "ACC_RADIUS_L", "ACC_RADIUS_M"
                                };

/*****************************************************************************/
/* INTERRUPT SERVICE ROUTINES */
/*****************************************************************************/

/*****************************************************************************/
/* HELPER FUNCTIONS */
/*****************************************************************************/

/*  
    NAME:               i2c_master_init
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Initializes the i2c interface.
    CALLING CONVENTION: i2c_master_init();
    CONDITIONS AT EXIT: This function modifies the state of GPIO pins 22 and 21.
    DATE STARTED:       3/3/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void i2c_master_init()
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

/*  
    NAME:               imu_write
    AURTHOR(S):         Isaac Shields
    CALLED BY:          // a lot of stuff
    PURPOSE:            Writes data to an IMU register.
    CALLING CONVENTION: reg is the IMU register being written to.  val is the value you are
                        writing to.
                        Example: imu_write(PWR_MODE, NORMAL_MODE);
    CONDITIONS AT EXIT: 
    DATE STARTED:       3/2/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void imu_write(uint8_t reg, uint8_t val)
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

/*  
    NAME:               imu_read
    AURTHOR(S):         Isaac Shields
    CALLED BY:          // a lot of stuff
    PURPOSE:            Reads data from an IMU register.
    CALLING CONVENTION: 
    CONDITIONS AT EXIT: 
    DATE STARTED:       3/2/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void imu_read(char reg, uint8_t * data, size_t len)
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

/*  
    NAME:               start_stop_init
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Initializes the START/STOP button.
    CALLING CONVENTION: start_stop_init();
    CONDITIONS AT EXIT: This function modifies the state of GPIO pin 33.
    DATE STARTED:       2/24/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void start_stop_init()
{   
    /* Configure button for pullup and input. */
    gpio_config_t start_stop;
    start_stop.intr_type = GPIO_PIN_INTR_DISABLE;
    start_stop.mode = GPIO_MODE_INPUT;
    start_stop.pin_bit_mask = START_STOP_MASK;
    start_stop.pull_down_en = GPIO_PULLDOWN_DISABLE;
    start_stop.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&start_stop);
}

/*  
    NAME:               nvs_init
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Initializes non-volotile storage used for bluetooth and calibration data.
    CALLING CONVENTION: nvs_init();
    CONDITIONS AT EXIT: 
    DATE STARTED:       3/1/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void nvs_init()
{   
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        /* NVS partition was truncated and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}

/*  
    NAME:               led_init
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Initializes the LEDs.
    CALLING CONVENTION: led_init();
    CONDITIONS AT EXIT: This function modifies the state of GPIO pins 5 and 16.
    DATE STARTED:       3/1/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void led_init()
{   
    /* Configure red LED */
    gpio_config_t red_led;
    red_led.intr_type = GPIO_PIN_INTR_DISABLE;
    red_led.mode = GPIO_MODE_OUTPUT;
    red_led.pin_bit_mask = RED_LED_MASK;
    red_led.pull_down_en = GPIO_PULLDOWN_DISABLE;
    red_led.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&red_led);
    gpio_set_level(RED_LED_PIN, 1); /* keep led on when booted */
    
    /* Configure blue LED */
    gpio_config_t blue_led;
    blue_led.intr_type = GPIO_PIN_INTR_DISABLE;
    blue_led.mode = GPIO_MODE_OUTPUT;
    blue_led.pin_bit_mask = BLUE_LED_MASK;
    blue_led.pull_down_en = GPIO_PULLDOWN_DISABLE;
    blue_led.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&blue_led);
    gpio_set_level(BLUE_LED_PIN, 0);
}

/*  
    NAME:               imu_init
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Initializes the IMU for normal operation.  The IMU is loaded with
                        previous calibration values from the flash memory.  It is set for
                        IMU sensor fusion mode and put into suspend mode to save power.
    CALLING CONVENTION: imu_init();
    CONDITIONS AT EXIT: This function modifies the state of GPIO pin 33.
    DATE STARTED:       3/2/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void imu_init()
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


/*****************************************************************************/
/* TASKS */
/*****************************************************************************/

/*  
    NAME:               imu_calib_task
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Calibrates the IMU.  IMU callibration is only needed when data seems off.  We save
                        the calibration in the flash and load it into the IMU after every reset.  This function
                        generates new values for the calibration.  It is only entered if the user holds the
                        START/STOP button during power on.
    CALLING CONVENTION: xTaskCreate(imu_calib_task, "imu_calib", IMU_CONFIG_SIZE, NULL, IMU_CONFIG_PRIORITY, NULL);
    CONDITIONS AT EXIT: This function does not exit.
    DATE STARTED:       2/28/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              NVS code used from nvs.value.example.main.c from Espessif.
*/
static void imu_calib_task()
{
    bool calib_ready = false;                   /* is system calibrated */
    uint8_t read_buf[BUF_SIZE] = { 0 };         /* buffer for i2c read */
    uint8_t lite = 0x00;                        /* is gyro light on or off */
    esp_err_t err;                              /* esp error type */
    nvs_handle_t handles[NVS_LABEL_COUNT];  /* create nvs handle for each piece of calibration data */
    
    // Accelerometer offsets are based on the G-Range.  The default is four.  If we change the default
    // we need to make sure to change this.
    
    /**************************************/
    /* PUT INTO IMU MODE */
    /**************************************/
    imu_write(OPR_MODE, IMU_MODE);
    
    /**************************************/
    /* WAIT FOR IMU TO BE CALIBRATED */
    /**************************************/
    while(calib_ready == false)
    {
        /* handle LED display */
        lite ^= 1;
        gpio_set_level(BLUE_LED_PIN, lite);
        vTaskDelay(500 / portTICK_PERIOD_MS); /* delay to see led changes */
        /* read calib_stat values */
        imu_read(CALIB_STAT, read_buf, 1);
        /* check values */
        if ( (read_buf[0] & ST_SYS1) && (read_buf[0] & ST_SYS1) )
            calib_ready = true;
    }
    
    /**************************************/
    /* SWITCH TO CONFIG MODE */
    /**************************************/
    /* clear leds until load is complete */
    gpio_set_level(BLUE_LED_PIN, 0);
    /* put into config mode in order to read offset */
    imu_write(OPR_MODE, CONFIG_MODE);
    
    /**************************************/
    /* LOAD OFFSET VALUES INTO NVS */
    /**************************************/
    /* read offset values */
    imu_read(ACC_OFFSET_X_LSB, read_buf, NVS_LABEL_COUNT);
    /* load offsets into nvs */
    for (int i = 0; i < NVS_LABEL_COUNT; i++)
    {
        err = nvs_open(nvs_labels[i], NVS_READWRITE, &handles[i]);  /* open nvs */ // add error handling
        err = nvs_set_u8(handles[i], nvs_labels[i], read_buf[i]);   /* write nvs */   // do error checking
        err = nvs_commit(handles[i]);                               /* commit changes */ // do error checking
        nvs_close(handles[i]);                                      /* close the handle */ // do error checking
    }
    /* show that load is complete */
    gpio_set_level(BLUE_LED_PIN, 1);
    
    while(1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    
}

/*  
    NAME:               loop_task
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Handles the main loop of the application.  It handles the three
                        states of READY (waiting for button press), RECORDING (recording
                        IMU data and waiting for button press), PROCESSING (processing IMU
                        data), and SENDING (sending processed data to the phone).
    CALLING CONVENTION: This must be called as a task, so use the convention below.
                        xTaskCreate(loop_task, "loop", LOOP_SIZE, NULL, LOOP_PRIORITY, NULL);
    CONDITIONS AT EXIT: This task will not exit.
    DATE STARTED:       2/26/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void loop_task()
{
    imu_init();
    while(1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/*  
    NAME:               app_main
    AURTHOR(S):         Isaac Shields
    CALLED BY:          main_task() in cpu_start.c
    PURPOSE:            Initializes the necessary drivers and spins up the tasks.
    CALLING CONVENTION: This function is only called by espressif libraries.
    CONDITIONS AT EXIT: Creates tasks which run indefinitely.  Once this retunrs the
                        main_task() is destroyed.
    DATE STARTED:       2/26/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
void app_main()
{
    led_init();
    start_stop_init();
    nvs_init();
    i2c_master_init();
    if (gpio_get_level(GPIO_NUM_33) == 0) /* if button is pressed */
    {
        xTaskCreate(imu_calib_task, "imu_calib", IMU_CALIB_SIZE, NULL, IMU_CALIB_PRIORITY, NULL);
    }
    else    /* button is not pressed */
    {
        xTaskCreate(loop_task, "loop", LOOP_SIZE, NULL, LOOP_PRIORITY, NULL);
    }
    
}
