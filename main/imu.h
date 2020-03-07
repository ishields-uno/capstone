/*  
    TITLE:          imu
    VERSION:        See Git logs.
    FILENAME:       imu.h
    AURTHOR(S):     Isaac Shields
    PURPOSE:        Contains definitions for the i2c and IMU.  Defines the functions
                    used in the imu.c file.
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

#include <stdint.h>
#include <stddef.h>
/*****************************************************************************/
/* DEFINITIONS */
/*****************************************************************************/

/**************************************/
/* IMU */
/**************************************/
/* power mode register */
#define PWR_MODE            (0x3E)      /* register address */        
#define NORMAL_MODE         (0x00)      /* normal mode value */
#define SUSPEND_MODE        (0x02)      /* suspend mode value */
/* operation mode register */
#define OPR_MODE            (0x3D)      /* register address */
#define CONFIG_MODE         (0x00)      /* config mode value */
#define IMU_MODE            (0x08)      /* imu_mode value */
/* calibration status register */
#define CALIB_STAT          (0x35)      /* register address */  
#define ST_SYS1             (1 << 6)    /* mask for system stat bit */
#define ST_SYS2             (1 << 7)    /* mask for system stat bit */
/* calibration registers */
#define ACC_OFFSET_X_LSB    (0x55)      /* register address for first calibration register */
#define NVS_LABEL_COUNT     (22)        /* number of NVS labels (number of calibration registers) */
/* linear acceleration registers */
#define LIA_DATA_X_LSB      (0X28)      /* linear acceleration data registers */
#define LIA_REG_CNT         (6)         /* amount of linear acceleration registers */

/**************************************/
/* I2C */
/**************************************/
#define I2C_FREQ                    (10000)     /* clock speed */
#define I2C_MASTER_TX_BUF_DISABLE   (0)         /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   (0)         /* I2C master doesn't need buffer */
#define IMU_I2C_ADDR                (0x28)      /* BNO055 address */
#define ACK_CHECK_EN                (0x1)       /* I2C master will check ack from slave*/
#define ACK_CHECK_DIS               (0x0)       /* I2C master will not check ack from slave */
#define ACK_VAL                     (0x0)       /* I2C ack val */
#define NACK_VAL                    (0x1)       /* I2C nack val */
#define BUF_SIZE                    (128)       /* I2C read buf size */

/*****************************************************************************/
/* FUNCTION DEFINITIONS */
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
void i2c_master_init();

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
void imu_write(uint8_t reg, uint8_t val);

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
void imu_read(char reg, uint8_t * data, size_t len);

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
void imu_init();