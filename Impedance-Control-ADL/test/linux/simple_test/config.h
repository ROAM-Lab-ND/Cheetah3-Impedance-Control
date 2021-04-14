/**
 * @file config.h
 * @brief File containing configuration settings used for the hardware interface.
 *
 * This file has settings for loop timing, ethercat, serial, and other I/O.
 */


#ifndef _config
#define _config
/** @brief Loop period of controller in nanoseconds */
#define K_LOOP_PERIOD_NSEC 1000000
/** @brief Loop period for LCM send when running over wireless in nanoseconds */
#define K_LCM_LOOP_PERIOD_WIRELESS_NSEC 1000000*50

/*****************
 *    SERIAL     *
 *****************/
//The breakout board provides 4 serial ports, numbered 0,1,2,3.  
//These numbers are not related to the port names (/dev/ttyUSBx)

/**@brief Name of the serial port used to connect to IMU */
#define K_SERIAL_PORT_NAME   "/dev/ttyUSB0" 

/**@brief Constant used to identify IMU serial port in C */
#define K_IMU_SERIAL_PORT 0

/**@brief String to put IMU in configuration mode */
#define K_IMU_INIT_1      "=config,1\r\n"
/**@brief String to set rotataion format for IMU */
#define K_IMU_INIT_2      "=rotfmt,RATE\r\n"
/**@brief String to exit IMU configuration mode */
#define K_IMU_INIT_3      "=config,0\r\n"

/**@brief IMU command to enable msync (currently unused) */
#define K_IMU_INIT_MSYNC  "=msync,ext\r\n"

/**@brief When defined, LCM writes occur in a separate thread */
#define LCM_THREAD

/**@brief Constant used to identify the other three serial ports not used by the IMU in C */
#define K_OTHER_SERIAL_1 1
/**@brief Constant used to identify the other three serial ports not used by the IMU in C */
#define K_OTHER_SERIAL_2 2
/**@brief Constant used to identify the other three serial ports not used by the IMU in C */
#define K_OTHER_SERIAL_3 3

/**@brief Name of serial port */
#define K_OTHER_SERIAL_1_NAME "/dev/ttyUSB1" 
/**@brief Name of serial port */
#define K_OTHER_SERIAL_2_NAME "/dev/ttyACM1" 
/**@brief Name of serial port */
#define K_OTHER_SERIAL_3_NAME "/dev/ttyACM0" 

/**@brief Baud rate for serial port 1 (xbee) ->taking over for footsensors*/
#define K_SERIAL_1_BAUD B921600 //xbee -> amw006
/**@brief Baud rate for serial port 2 (Alexa interface) */
#define K_SERIAL_2_BAUD B115200 //likely alexa
/**@brief Baud rate for serial port 3 (MBED) */
#define K_SERIAL_3_BAUD B9600

/**@brief Port number for xbee serial */
#define K_XBEE_PORT 1
/**@brief Port number for Alexa interface serial */
#define K_ALEXA_PORT 2
/**@brief Port number for RC Mbed interface serial */
#define K_RC_MBED_PORT 3


//if K_ETHERCAT_ERR_MAX packets out of K_ETHERCAT_ERR_PERIOD are
//bad, then restart ethercat.
/**@brief EtherCAT errors are measured over this period of loop iterations */
#define K_ETHERCAT_ERR_PERIOD 100

/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define K_ETHERCAT_ERR_MAX 100

/**@brief Leg to use when testing a single leg */
#define K_3DOF_SLAVE 0

/**@brief Set to 0 to disable printing Simulink "log_data" messages */
#define K_PRINT_DEBUG  0


//#define K_ETHERCAT_DISABLE 0

/**@brief Number of floats to send over Xbee */
#define K_XBEE_SEND_FLOATS    10
/**@brief Number of floats to receive over Xbee */
#define K_XBEE_RECEIVE_FLOATS 9
/**@brief Number of integers to send over Xbee */
#define K_XBEE_SEND_INTS      10
/**@brief Number of ints to receive over Xbee */
#define K_XBEE_RECEIVE_INTS   3

/**@brief Size of incoming Xbee message buffer */
#define K_XBEE_INCOMING_MESSAGE_SIZE 300

/**@brief Size of the Xbee field buffer */
#define K_XBEE_INCOMING_FIELD_SIZE (8*20)

/**@brief Size of the outgoing Xbee message buffer */
#define K_XBEE_OUTGOING_MESSAGE_SIZE 1000

/**@brief Set to 1 to enable printing of NMEA decode errors */
#define K_PRINT_NMEA_DEBUG 1

/**@brief Sleep period between Xbee writes */
#define K_XBEE_WRITE_SLEEP 50000

/**@brief Number of legs connected over EtherCAT */
#define K_NUMBER_OF_LEGS 4

//#define K_ETHERCAT_DISABLE 1

#endif
