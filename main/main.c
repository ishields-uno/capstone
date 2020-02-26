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

/*****************************************************************************/
/* DEFINITIONS */
/*****************************************************************************/

/* UART */
#define IMU_TX      (GPIO_NUM_1)            /* UART TX Connected to BNO055 */
#define IMU_RX      (GPIO_NUM_3)            /* UART RX Connected to BNO055 */
#define RTS         (UART_PIN_NO_CHANGE)    /* Not using RTS */
#define CTS         (UART_PIN_NO_CHANGE)    /* Not using CTS */
#define BUF_SIZE    (1024)                  /* UART Data Buffer */
#define BAUD_RATE   (115200)                /* UART Baud Rate */

/* START/START PIN */
#define START_STOP_PIN   33                         /* use pin 33 */
#define START_STOP_MASK  (1ULL<<START_STOP_PIN)     /* create bit mask for setup call */

/* LOOP TASK */
#define LOOP_ACLLOC     (2048)  /* size of memory allocated to the loop() task */
#define LOOP_PRIORITY   (10)    /* priority of the loop() task */

/* IMU */

/*****************************************************************************/
/* INTERRUPT SERVICE ROUTINES */
/*****************************************************************************/

/*****************************************************************************/
/* HELPER FUNCTIONS */
/*****************************************************************************/

/*  
    NAME:               init
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Initializes the following firmware modules needed for the application:
                        1. UART for communication with BNO055.
                        2. START/START button pullup resistor and iterrupt setting.
    CALLING CONVENTION: init();
    CONDITIONS AT EXIT: This function modifies the state of UART pins 1 and 3.  It also configures
                        the UART0 peripheral.  Additonally, it modifies the function of pin 33 to be
                        a general purpose GPIO pin.
    DATE STARTED:       2/24/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void init()
{
    /**************************************/
    /* CONFIGURE UART */
    /**************************************/
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
    
    /**************************************/
    /* CONFIGURE START/STOP BUTTON */
    /**************************************/
    /* Configure button for pullup and input. */
    gpio_config_t start_stop;
    start_stop.intr_type = GPIO_PIN_INTR_DISABLE;
    start_stop.mode = GPIO_MODE_INPUT;
    start_stop.pin_bit_mask = START_STOP_MASK;
    start_stop.pull_down_en = GPIO_PULLDOWN_DISABLE;
    start_stop.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&start_stop);
    
    /**************************************/
    /* CONFIGURE BNO055 (IMU) */
    /**************************************/
    
}

/*****************************************************************************/
/* TASKS */
/*****************************************************************************/

/*  
    NAME:               loop
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Handles the main loop of the application.  It handles the three
                        states of READY (waiting for button press), RECORDING (recording
                        IMU data and waiting for button press), PROCESSING (processing IMU
                        data), and SENDING (sending processed data to the phone).
    CALLING CONVENTION: This must be called as a task, so use the convention below.
                        xTaskCreate(echo_task, "uart_echo_task", LOOP_ACLLOC, NULL, 10, NULL);
    CONDITIONS AT EXIT: This task will not exit.
    DATE STARTED:       2/26/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
static void loop()
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);   /* buffer for IMU data */
    
    while(1)
    {
        /**************************************/
        /* WAIT FOR BUTTON PRESS */
        /**************************************/
        if (gpio_get_level(GPIO_NUM_33) == 0) /* if button is pressed */
        {
            /* debounce the button press */
            vTaskDelay(50 / portTICK_PERIOD_MS);  

            /**************************************/
            /* READ IMU DATA */
            /**************************************/
            
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
    init();
    xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL);
}
