/* 	
	TITLE:			Main
	VERSION:		See Git logs.
	FILENAME:		main.c
	AURTHOR(S):		Isaac Shields
	PURPOSE:		Handles data reading, processing, and sending on the device.
					The program waits for the host (smartphone) to connect.  Once
					connected, it waits for the user to press a button.  The button
					press starts IMU data reading.  When the button is pressed again,
					the program stop recording, prcesses the data, and send it to
					the host.
	HOW TO LOAD:	DEV BOARD-
					Build in the project directory using "idf.py build"  Load to
					the board using "idf.py -p PORT flash".  PORT should be replaced
					with the serial port you are on (COM1, COM2, etc.).
					PRODUCT-
					In progress.
	DATE STARTED:	2/24/2020
	UPDATE HISTORY:	See Git logs.
	NOTES:			This program uses example code provided by Espessif Systems.
					The GitHub containing this code is found here: 
					https://github.com/espressif/esp-idf
*/

/*****************************************************************************/
/* INCLUDES */
/*****************************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/*****************************************************************************/
/* UART DEFINITIONS */
/*****************************************************************************/

#define IMU_TX  	(GPIO_NUM_1)			/* UART TX Connected to BNO055 */
#define IMU_RX  	(GPIO_NUM_3)			/* UART RX Connected to BNO055 */
#define RTS  		(UART_PIN_NO_CHANGE)	/* Not using RTS */
#define CTS  		(UART_PIN_NO_CHANGE)	/* Not using CTS */
#define BUF_SIZE 	(1024)					/* UART Data Buffer */
#define BAUD_RATE	(115200)				/* UART Baud Rate */

/*****************************************************************************/
/* HELPER FUNCTIONS */
/*****************************************************************************/

/* 	
	NAME:				init
	AURTHOR(S):			Isaac Shields
	CALLED BY:			app_main() in main.c
	PURPOSE:			Initializes the following firmware modules needed for the application:
						1. UART for communication with BNO055.
						2. START/START button pullup resistor and iterrupt setting.
	CALLING CONVENTION:	if (!init())
							return <error message>;
	CONDITIONS AT EXIT:	This function does not modify any global variables.  Returns false
						if the initizalization failed and true if it succeded.  This function
						does not take any arguments.
	DATE STARTED:		2/24/2020
	UPDATE HISTORY:		See Git logs.
	NOTES:				
*/
static bool init()
{

	
	/**************************************/
	/* CONFIGURE START/STOP BUTTON */
	/**************************************/
	/* Configure button for pullup input with negedge detection. */
	gpio_config_t start_stop = {
		.pin_bit_mask	= GPIO_SEL_33,
		.mode			= GPIO_MODE_INPUT,
		.pull_up_en		= GPIO_PULLUP_ENABLE,
		.pull_down_en	= GPIO_PULLDOWN_DISABLE,
		.intr_type		= GPIO_INTR_DISABLE
	};
	gpio_config(&start_stop);
	
	return true;
}

/*****************************************************************************/
/* TASKS */
/*****************************************************************************/


static void echo_task()
{
	//init();
	/**************************************/
	/* CONFIGURE UART */
	/**************************************/
	/* Configure UART for 115200 bps, 8N1. */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, IMU_TX, IMU_RX, RTS, CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
	uint8_t data = (uint8_t *) malloc(BUF_SIZE);
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        uart_write_bytes(UART_NUM_1, (const char *) data, len);
    }
}


void app_main()
{
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
}
