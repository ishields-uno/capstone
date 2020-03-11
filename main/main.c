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

#include "ui.h"
#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_log.h"

/*****************************************************************************/
/* DEFINITIONS */
/*****************************************************************************/

/* APP_MAIN */
#define KB                  (1024)                      /* size of 1KB in bytes */
#define LOOP_SIZE           ((40) * KB)                 /* 40KB for loop task */
#define LOOP_PRIORITY       (10)                        /* priority of the loop task */
#define IMU_CALIB_SIZE      (2048)                      /* size of memory allocated to the loop_task() task */
#define IMU_CALIB_PRIORITY  (10)                        /* priority of the loop_task() task */
#define PRO_CPU_NUM         (0)                         /* process cpu number */
#define APP_CPU_NUM         (1)                         /* application cpu number */

/* LOOP TASK */


/* IMU CALIB TASK */

/*****************************************************************************/
/* CONSTANTS */
/*****************************************************************************/

/*****************************************************************************/
/* INTERRUPT SERVICE ROUTINES */
/*****************************************************************************/

/*****************************************************************************/
/* HELPER FUNCTIONS */
/*****************************************************************************/

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
    //for (int i = 0; i < NVS_LABEL_COUNT; i++)
    //{
    //    err = nvs_open(nvs_labels[i], NVS_READWRITE, &handles[i]);  /* open nvs */ // add error handling
    //    err = nvs_set_u8(handles[i], nvs_labels[i], read_buf[i]);   /* write nvs */   // do error checking
    //    err = nvs_commit(handles[i]);                               /* commit changes */ // do error checking
    //    nvs_close(handles[i]);                                      /* close the handle */ // do error checking
    //}
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
    uint8_t data[READ_BUF];
    
    while(1)
    {
        if((gpio_get_level(START_STOP_PIN) == 0)) /* start/stop is pressed */
        {
            vTaskDelay(200 / portTICK_PERIOD_MS); /* debounce */
            read_data(data, sizeof(data));
            vTaskDelay(200 / portTICK_PERIOD_MS); /* debounce */
            /* //logging
            for (uint16_t i = 0; i < READ_BUF; i++)
            {
                ESP_LOGW("READ DUMP", "Byte %d: 0x%02x ", i, data[i]);
            }
            */
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);    /* wait for button press */
        }
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
    if (gpio_get_level(START_STOP_PIN) == 0) /* if button is pressed */
    {
        xTaskCreatePinnedToCore(imu_calib_task, "imu_calib", IMU_CALIB_SIZE, NULL, IMU_CALIB_PRIORITY, NULL, APP_CPU_NUM);
    }
    else    /* button is not pressed */
    {
        imu_init();
        xTaskCreatePinnedToCore(loop_task, "loop", LOOP_SIZE, NULL, LOOP_PRIORITY, NULL, APP_CPU_NUM);
    } 
}
