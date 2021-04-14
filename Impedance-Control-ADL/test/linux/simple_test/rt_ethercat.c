/**
 * @file rt_ethercat.c
 * @brief Interface to EtherCAT hardware
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <time.h>
#include "config.h"
#include "rt_gpio.h"
#include "rt_logger.h"
// #include "rt_lcm.h"

#include "ethercat.h"
#include "rt_tiboard_data.h"
#include "rt_ethercat.h"



#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

/**
 * @brief Called when EtherCAT connection has failed.
 *
 * Does a GPIO E-Stop, writes to log file with reason/time, closes log file, and exits.
 * Does nothing when in simulation
 */
void degraded_handler()
{
#ifndef SIMULATOR
    //shut of gpio enables
    estop();
    printf("[EtherCAT Error] Logging error...\n");
    FILE* fd = get_logger_fd();
    time_t current_time = time(NULL);
    char* time_str = ctime(&current_time);
    fprintf(fd, "ESTOP. EtherCAT became degraded at %s.\n", time_str);
    fclose(fd);
    printf("[EtherCAT Error] Stopping RT process.\n");
    exit(0);
#endif
}

void rt_ethercat_stop(){
    inOP = FALSE;
     ec_slave[0].state = EC_STATE_INIT;
    /* request INIT state for all slaves */
    ec_writestate(0);
}

//most of this function is from the EtherCAT example code
/**
 * @brief Starts an EtherCAT connection on a given interface
 *
 * Most of this function is from the SOEM example code
 * This function does nothing in simulation
 * @param ifname String for network interface
 */
int run_ethercat(char *ifname)
{
#ifndef SIMULATOR
    int i, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;


    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("[EtherCAT Init] Initialization on device %s succeeded.\n",ifname);
        /* find and auto-config slaves */

        if ( ec_config_init(FALSE) > 0 )
        {
            printf("[EtherCAT Init] %d slaves found and configured.\n",ec_slavecount);
            if(ec_slavecount < K_NUMBER_OF_LEGS)
            {
                char error_msg[200];
                printf("[RT EtherCAT] Warning: Expected %d legs, found %d.\n", K_NUMBER_OF_LEGS, ec_slavecount); 
                // log_error(error_msg);
               // printf(error_msg);
            }

            ec_config_map(&IOmap);
            ec_configdc();

            printf("[EtherCAT Init] Mapped slaves.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 8) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 8) iloop = 8;

            printf("[EtherCAT Init] segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

            printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("[EtherCAT Init] Operational state reached for all slaves.\n");
                inOP = TRUE;
                return 1;

            }
            else
            {
                log_error("[EtherCAT Error] Not all slaves reached operational state.\n");
                ec_readstate();

                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        char error_msg[100];

                        sprintf(error_msg,"[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                                i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                        log_error(error_msg);
                    }
                }
            }
        }
        else
        {
            printf("[EtherCAT Error] No slaves found!\n");
        }
    }
    else
    {
        printf("[EtherCAT Error] No socket connection on %s - are you running run.sh?\n",ifname);
    }
#endif
    return 0;
}

int err_count = 0;
int err_iteration_count = 0;

//thread to monitor the state of the etherCAT connection.
/**
 * @brief Function that monitors the state of the connection and attempts to keep all devices connected.
 *
 * Most of this code is from the EtherCAT example code.  This code should run in a separate thread as this function does not return.
 * This function does nothing in Simulation.
 */
OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
#ifndef SIMULATOR
    int slave = 0;
    while(1)
    {
        //count errors
        if(err_iteration_count > K_ETHERCAT_ERR_PERIOD)
        {
            err_iteration_count = 0;
            err_count = 0;
        }

        if(err_count > K_ETHERCAT_ERR_MAX)
        {
            //possibly shut down
            log_error("[EtherCAT Error] EtherCAT connection degraded.\n");
            log_error("[Simulink-Linux] Shutting down....\n");
//	 printf("[EtherCAT Error] EtherCAT connection degraded. \n");	   
         degraded_handler();
        }
        err_iteration_count++;

        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    char error_msg[100];
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                       // log_error(error_msg);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                        err_count++;
                        // increment_ecat_error();
                    }
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                       // log_error(error_msg);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                        err_count++;
                        // increment_ecat_error();
                    }
                    else if(ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("[EtherCAT Status] Slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("[EtherCAT Error] Slave %d lost\n",slave);
                            //log_error(error_msg);
                            err_count++;
                            // increment_ecat_error();
                            // increment_ecat_error();
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if(!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("[EtherCAT Status] Slave %d recovered\n",slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("[EtherCAT Status] Slave %d found\n",slave);
                    }
                }
            }
            if(!ec_group[currentgroup].docheckstate)
                printf("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(50000);
    }
#endif
}


/**
 * @brief Initialize EtherCAT connection
 *
 * On the robot, this function starts the etherCAT monitoring thread and tries connecting to the legs every second for 100 seconds until a connection is made. 
 *
 * In simulation, the EtherCAT LCM is initialized.
 */
static pthread_mutex_t rt_command_mutex;
static pthread_mutex_t rt_data_mutex;

void rt_ethercat_init(char *ifname)
{

    printf("[EtherCAT] Initializing EtherCAT\n");
    //initialize monitoring thread
    osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);

    // Initialize mutexes
    if (pthread_mutex_init(&rt_command_mutex, NULL) < 0) // success 0 fail -1
    {
        printf("Error: rt_command_mutex not initialized \n");
	return;       
    }

    if (pthread_mutex_init(&rt_data_mutex, NULL) < 0)
    {
        printf("Error: rt_data_mutex not initialized \n");    
        return;
    }
    
    //try initialization until it succeeds
    int i;
    int rc;
    for(i = 1; i < 100; i++)
    {
        printf("[EtherCAT] Attempting to start EtherCAT, try %d of 100.\n", i);
        rc = run_ethercat(ifname);
        if(rc) break;
        osal_usleep(1000000);
    }
    if(rc) printf("[EtherCAT] EtherCAT successfully initialized on attempt %d \n", i);
    else
    {
        char error_msg[100];
        sprintf(error_msg,"[EtherCAT Error] Failed to initialize EtherCAT after 100 tries. \n");
        log_error(error_msg);
    }

}


int wkc_err_count = 0;
int wkc_err_iteration_count = 0;

//initiate etherCAT communication
/** @brief Send and receive data over EtherCAT
 *
 * In Simulation, send data over LCM
 * On the robt, verify the EtherCAT connection is still healthy, send data, receive data, and check for lost packets
 */
void rt_ethercat_run()
{
#ifdef SIMULATOR
    // cheetahlcm_ecat_command_t_publish(lcm_ecat, "CHEETAH_ecat_command", (cheetahlcm_ecat_command_t *) &ecat_command);
#else
    //check connection
    if(wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if(wkc_err_count > K_ETHERCAT_ERR_MAX)
    {
        log_error("[EtherCAT Error] Error count too high!\n");
        //program terminates in degraded handler.
        degraded_handler();
    }

    //send
    pthread_mutex_lock(&rt_command_mutex);
    ec_send_processdata();
    pthread_mutex_unlock(&rt_command_mutex);

    //receive
    pthread_mutex_lock(&rt_data_mutex);
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    pthread_mutex_unlock(&rt_data_mutex);

    //check for dropped packet
    if(wkc < expectedWKC)
    {
        log_error("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
        wkc_err_count++;
        // increment_ecat_error();
    }
    wkc_err_iteration_count++;
#endif
}

void rt_ethercat_get_data_3DOF(TiBoardData* data) {
    pthread_mutex_lock(&rt_data_mutex);
    for (int slave = 0; slave < 4; ++slave)
    {
        TiBoardData* slave_src = (TiBoardData*)(ec_slave[slave + 1].inputs);
        if(slave_src)
            data[slave] = *(TiBoardData*)(ec_slave[slave + 1].inputs);
    }
    pthread_mutex_unlock(&rt_data_mutex);
}
void rt_ethercat_set_command_3DOF(TiBoardCommand* command) {
    pthread_mutex_lock(&rt_command_mutex);
    for (int slave = 0; slave < 4; ++slave)
    {
        TiBoardCommand* slave_dest = (TiBoardCommand*)(ec_slave[slave + 1].outputs);
        if(slave_dest)
            *(TiBoardCommand*)(ec_slave[slave + 1].outputs) = command[slave];
    }
    pthread_mutex_unlock(&rt_command_mutex);
}

