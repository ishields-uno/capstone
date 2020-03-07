/*  
    TITLE:          ui
    VERSION:        See Git logs.
    FILENAME:       ui.h
    AURTHOR(S):     Isaac Shields
    PURPOSE:        Contains variable and function definitions for user interface
                    components (LEDs and buttons).
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

/*****************************************************************************/
/* DEFINITIONS */
/*****************************************************************************/

/**************************************/
/* START/STOP BUTTON */
/**************************************/
#define START_STOP_PIN   (33)                       /* use pin 33 */
#define START_STOP_MASK  (1ULL<<START_STOP_PIN)     /* create bit mask for setup call */

/**************************************/
/* LEDs */
/**************************************/
#define BLUE_LED_PIN    (16)                        /* use pin 33 */
#define BLUE_LED_MASK   (1ULL<<BLUE_LED_PIN)        /* mask for pin 16 */
#define RED_LED_PIN     (5)                         /* use pin 5 */
#define RED_LED_MASK    (1ULL<<RED_LED_PIN)         /* mask for pin 5 */

/*****************************************************************************/
/* FUNCTION DEFINITIONS */
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
void start_stop_init();

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
void led_init();
