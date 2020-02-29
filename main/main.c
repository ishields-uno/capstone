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
#include "driver/uart.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"

/*****************************************************************************/
/* DEFINITIONS */
/*****************************************************************************/

/* APP_MAIN */
#define KB                  (1024)                      /* size of 1KB in bytes */
#define LOOP_SIZE           ((40) * KB)                 /* 40KB for loop task */
#define LOOP_PRIORITY       (10)                        /* priority of the loop task */
#define IMU_CONFIG_SIZE     (2048)                      /* size of memory allocated to the loop_task() task */
#define IMU_CONFIG_PRIORITY (10)                        /* priority of the loop_task() task */


/* UART */
#define IMU_TX      (GPIO_NUM_1)                    /* UART TX Connected to BNO055 */
#define IMU_RX      (GPIO_NUM_3)                    /* UART RX Connected to BNO055 */
#define RTS         (UART_PIN_NO_CHANGE)            /* Not using RTS */
#define CTS         (UART_PIN_NO_CHANGE)            /* Not using CTS */
#define BUF_SIZE    (256)                           /* UART Data Buffer */
#define BAUD_RATE   (115200)                        /* UART Baud Rate */

/* START/START PIN */
#define START_STOP_PIN   (33)                       /* use pin 33 */
#define START_STOP_MASK  (1ULL<<START_STOP_PIN)     /* create bit mask for setup call */

/* LOOP TASK */


/* IMU CONFIG TASK */
#define NVS_LABEL_COUNT (14)    /* number of NVS labels */

/* IMU */
#define START           (0xAA)                      /* start byte for transmission to IMU */
#define WR_RESP_HEAD    (0xEE)                      /* start byte for write response from IMU */
#define RD_SUCC_HEAD    (0xBB)                      /* start byte for read respone from IMU */
#define RD_FAIL_HEAD    (0xEE)                      /* start byte for failed read response from IMU */
#define READ            (0x01)                      /* r/w byte value to read */
#define WRITE           (0x00)                      /* r/w byte value to write */
#define WRITE_SUCCESS   (0x01)  

#define PWR_MODE            (0x3E)                  /* power mode register */
#define NORMAL_MODE         (0x00)
#define SUSPEND_MODE        (0x02)

#define OPR_MODE            (0x3D)                  /* operation mode register */
#define CONFIG_MODE         (0x00)
#define IMU_MODE            (0x08)

#define CALIB_STAT          (0x35)                  /* calibration status register */
#define ST_GYR_MASK         ((1 << 5) | (1 << 4))
#define ST_ACC_MASK         ((1 << 3) | (1 << 2))

#define ACC_OFFSET_X_LSB    (0x55)                  /* calibration offset registers */
#define ACC_OFFSET_X_MSB    (0x56)
#define ACC_OFFSET_Y_LSB    (0x57)
#define ACC_OFFSET_Y_MSB    (0x58)
#define ACC_OFFSET_Z_LSB    (0x59)
#define ACC_OFFSET_Z_MSB    (0x5A)
#define GYR_OFFSET_X_LSB    (0x61)
#define GYR_OFFSET_X_MSB    (0x62)
#define GYR_OFFSET_Y_LSB    (0x63)
#define GYR_OFFSET_Y_MSB    (0x64)
#define GYR_OFFSET_Z_LSB    (0x65)
#define GYR_OFFSET_Z_MSB    (0x66)
#define ACC_RADIUS_LSB      (0x67)
#define ACC_RADIUS_MSB      (0x68)

#define LIA_DATA_X_LSB      (0X28)                  /* linear acceleration data registers */
#define LIA_DATA_X_MSB      (0X29)
#define LIA_DATA_Y_LSB      (0X2A)
#define LIA_DATA_Y_MSB      (0X2B)
#define LIA_DATA_Z_LSB      (0X2C)
#define LIA_DATA_Z_MSB      (0X2D)

/*****************************************************************************/
/* CONSTANTS */
/*****************************************************************************/

/* IMU CONFIG TASK */
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
    NAME:               delay
    AURTHOR(S):         Isaac Shields
    CALLED BY:          
    PURPOSE:            Delays the program for x microseconds.
    CALLING CONVENTION: Pass the number of microseconds you want to delay.  Do not use this for
                        times greater than 10ms.  Instead, us vTaskDelay().  Don't call with 
                        negative numbers.
                        Example for 10 uS: delay(10);
    CONDITIONS AT EXIT: This function has not return and does not modify any external variables.
    DATE STARTED:       2/28/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void delay(int16_t uS)
{
    volatile uint8_t val; /* must be volatile to avoid compiler optimization */
    while (uS > 0)
    {
        for(val = 0; val < 9; val++)
        {
        }
        uS--;
    }
}

/*  
    NAME:               uart_init
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Initializes UART communication with BNO055.
    CALLING CONVENTION: uart_init();
    CONDITIONS AT EXIT: This function modifies the state of UART pins 1 and 3.  It also configures
                        the UART0 peripheral.
    DATE STARTED:       2/24/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void uart_init()
{   
    /* Configure UART for 115200 bps, 8N1, no parity. */
    uart_config_t uart_config = 
    {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, IMU_TX, IMU_RX, RTS, CTS);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
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
    NAME:               read_data
    AURTHOR(S):         Isaac Shields
    CALLED BY:          loop_task() in main.c
    PURPOSE:            Reads raw linear acceleration data from the IMU.
    CALLING CONVENTION: This function takes a pointer to a uint8_t type.  This pointer is filled with 
                        IMU data.  The maximum size of data collected is 36KB.  If the functiion runs
                        longer (uninterrupted by a user button press), it will abort.  TRUE means data was
                        collected successfully and FALSE means data collection failed.
                        is 36KB.
    CONDITIONS AT EXIT: This function fills the buffer passed to it.
    DATE STARTED:       2/27/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
//static bool read_data(uint8_t * data)
//{
    
//}

/*****************************************************************************/
/* TASKS */
/*****************************************************************************/

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


/*  
    NAME:               imu_config_task
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Calibrates the IMU.  IMU callibration is only needed when data seems off.  We save
                        the calibration in the flash and load it into the IMU after every reset.  This function
                        generates new values for the calibration.  It is only entered if the user holds the
                        START/STOP button during power on.
    CALLING CONVENTION: xTaskCreate(imu_config_task, "imu_config", IMU_CONFIG_SIZE, NULL, IMU_CONFIG_PRIORITY, NULL);
    CONDITIONS AT EXIT: This function does not exit.
    DATE STARTED:       2/28/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              NVS code used from nvs.value.example.main.c from Espessif.
*/
static void imu_config_task()
{
    uint8_t temp;   // remove when done testing
    
    while (1)
    {
    if (gpio_get_level(GPIO_NUM_33) == 0) /* if button is pressed */
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS); // debounce
        
        /**************************************/
        /* INITIALIZE NVS */
        /**************************************/
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
        {
            /* NVS partition was truncated and needs to be erased */
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK( err );
        
        /**************************************/
        /* OPEN NVS HANDLES */
        /**************************************/
        nvs_handle_t handles[NVS_LABEL_COUNT];   /* create nvs handle for each piece of calibration data */
        for (int i = 0; i < NVS_LABEL_COUNT; i++)
        {
            err = nvs_open(nvs_labels[i], NVS_READWRITE, &handles[i]);/* open nvs */ // add error handling
            if (err != ESP_OK)
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            err = nvs_set_u8(handles[i], nvs_labels[i], (uint8_t) i); /* write nvs */   // do error checking
            if (err != ESP_OK)
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            err = nvs_commit(handles[i]);   // do error checking
            if (err != ESP_OK)
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            err = nvs_get_u8(handles[i], nvs_labels[i], &temp);
            if (err != ESP_OK)
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            uart_write_bytes(UART_NUM_0, (char *) &temp, 1);
                
        }
    }
    else    /* if button is not pressed */
    {
        /* delay to feed the watchdog */
        vTaskDelay(50 / portTICK_PERIOD_MS);
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
    uart_init();
    start_stop_init();
    //xTaskCreate(loop_task, "loop", LOOP_SIZE, NULL, LOOP_PRIORITY, NULL);
    xTaskCreate(imu_config_task, "imu_config", IMU_CONFIG_SIZE, NULL, IMU_CONFIG_PRIORITY, NULL);
}
