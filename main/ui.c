/*  
    TITLE:          ui
    VERSION:        See Git logs.
    FILENAME:       ui.c
    AURTHOR(S):     Isaac Shields
    PURPOSE:        Implements the functions defined in ui.h
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

#include "ui.h"
#include "driver/gpio.h"

/*****************************************************************************/
/* CONSTANT VARIABLES */
/*****************************************************************************/
                                
/*****************************************************************************/
/* FUNCTION IMPLEMENTATIONS */
/*****************************************************************************/
void led_init()
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

void start_stop_init()
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
