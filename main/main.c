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
#include "ble.h"
#include "esp_gatts_api.h"

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
#define SEND_BUF        (509)       /* size of BLE MTU */


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
    //esp_err_t err;                              /* esp error type */
    //nvs_handle_t handles[NVS_LABEL_COUNT];  /* create nvs handle for each piece of calibration data */
    
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
    uint8_t NEW_DATA[2] = { 'N', 'C'};                  /* new data from device */
    uint8_t READ_MORE_DATA[2] = { 'N', 'N'};            /* more data to be read */
    uint8_t STOP_READING_DATA[2] = { 'N', 'P'};         /* no more data to read */
    
    uint8_t data[READ_BUF];                             /* imu data buffer */
    uint8_t send[SEND_BUF] = { 'D' };                   /* buffer for sending data over bluetooth */
    bool done = false;                                  /* have we sent all the data? */
    int i = 0;                                          /* iterator to track single packet size*/
    int j = 0;                                          /* iterator to track total data size*/
    const uint8_t * ret;                                /* buffer for iPhone response */
    uint16_t ret_length;                                /* length of iPhone response */
    
    while(1)
    {
        if((gpio_get_level(START_STOP_PIN) == 0)) /* start/stop is pressed */
        {
            ESP_LOGW("TRANSFER TEST:", "Started reading from IMU.");
            vTaskDelay(200 / portTICK_PERIOD_MS); /* debounce */
            read_data(data, sizeof(data));
            vTaskDelay(200 / portTICK_PERIOD_MS); /* debounce */
            ESP_LOGW("TRANSFER TEST:", "Stopped reading from IMU.");
            
            /* notify the phone of new data */
            esp_ble_gatts_send_indicate (   heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if, 
                                            heart_rate_profile_tab[PROFILE_APP_IDX].conn_id, 
                                            heart_rate_handle_table[IDX_CHAR_VAL_A],
                                            sizeof(NEW_DATA), NEW_DATA, false
                                        );
                                                
            do
            {
                /* transfer data into the send buffer */
                for (i = 1; i < SEND_BUF; i += 4, j += 6)
                {
                    /* if we hit the packet tail, say we are done and break */
                    if  (   
                            (data[j] == PKT_TAIL) && (data[j + 1] == PKT_TAIL) &&
                            (data[j + 2] == PKT_TAIL) && (data[j + 3] == PKT_TAIL)
                        )
                    {
                        ESP_LOGW("TRANSFER TEST:", "Hit end of data at j = %d.", j);
                        done = true;
                        break;
                    }
                    /* load the next samples */
                    else
                    {
                        send[i] = data[j];
                        send[i + 1] = data[j + 1];
                        send[i + 2] = data[j + 2];
                        send[i + 3] = data[j + 3];
                    }
                }
                
                ESP_LOGW("TRANSFER TEST:", "Loaded packet with %d bytes.  %d bytes have been processed.", i, j);
                ESP_LOGW("TRANSFER TEST:", "Packet value: %.*s", SEND_BUF, send);
                
                /* wait to see if the phone has processed the previous packet */
                do
                {
                    esp_ble_gatts_get_attr_value(heart_rate_handle_table[IDX_CHAR_VAL_A], &ret_length, &ret);
                    vTaskDelay(10 / portTICK_PERIOD_MS);    /* don't spin on this */
                } while(*ret != 0x55);
                
                ESP_LOGW("TRANSFER TEST:", "Received read ready signal from the iPhone.");
                
                /* update the characteristic and notify the phone that it can read more data */
                esp_ble_gatts_set_attr_value(heart_rate_handle_table[IDX_CHAR_VAL_A], i + 1, send);
                esp_ble_gatts_send_indicate (   heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if, 
                                                heart_rate_profile_tab[PROFILE_APP_IDX].conn_id, 
                                                heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(READ_MORE_DATA), READ_MORE_DATA, false
                                            );
                ESP_LOGW("TRANSFER TEST:", "Characteristic value updated, ready to be read.");
            } while(done != true);
            
            /* notify phone to stop reading data */
            esp_ble_gatts_send_indicate (   heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if, 
                                            heart_rate_profile_tab[PROFILE_APP_IDX].conn_id, 
                                            heart_rate_handle_table[IDX_CHAR_VAL_A],
                                            sizeof(STOP_READING_DATA), STOP_READING_DATA, false
                                        );
            
            /* reset data pointer */
            j = 0;
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
    ble_init();
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
