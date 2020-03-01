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

/* TEST TASK */
#define TEST_CONFIG_SIZE        (2048)              /* size of memory allocated to the loop_task() task */
#define TEST_CONFIG_PRIORITY    (10)                /* priority of the loop_task() task */
#define TEST_PIN                (25)                /* use pin 33 */
#define TEST_MASK               (1ULL<<TEST_PIN)    /* create bit mask for setup call */

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

/*****************************************************************************/
/* TASKS */
/*****************************************************************************/
/*  
    NAME:               test_task
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Runs the unit test for this branch.  See teh git branch
                        name for the test code.
    CALLING CONVENTION: xTaskCreate(test_task, "test_task", IMU_CONFIG_SIZE, NULL, IMU_CONFIG_PRIORITY, NULL);
    CONDITIONS AT EXIT: This function modifies the state of GPIO pin 33.
    DATE STARTED:       2/24/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void test_task()
{
    /* Configure output port for testing */
    gpio_config_t test;
    test.intr_type = GPIO_PIN_INTR_DISABLE;
    test.mode = GPIO_MODE_OUTPUT;
    test.pin_bit_mask = TEST_MASK;
    test.pull_down_en = GPIO_PULLDOWN_DISABLE;
    test.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&test);
    
    if (gpio_get_level(START_STOP_PIN) == 1) /* if button is not pressed */
    {
        while(1)
        {
            if (gpio_get_level(START_STOP_PIN) == 0)
            {
                vTaskDelay( 2000 / portTICK_PERIOD_MS ); /* two second debounce */
                gpio_set_level(TEST_PIN, 0);
                vTaskDelay( 100 / portTICK_PERIOD_MS );
                gpio_set_level(TEST_PIN, 1);
                vTaskDelay( 100 / portTICK_PERIOD_MS );
            }
            vTaskDelay( 100 / portTICK_PERIOD_MS );
        } 
    }
    else /* button is pressed */
    {
        while(1)
        {
            gpio_set_level(TEST_PIN, 0);
            vTaskDelay( 100 / portTICK_PERIOD_MS );
            gpio_set_level(TEST_PIN, 1);
            vTaskDelay( 100 / portTICK_PERIOD_MS );
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
    start_stop_init();
    xTaskCreate(test_task, "test_task", TEST_CONFIG_SIZE, NULL, TEST_CONFIG_PRIORITY, NULL);
}
