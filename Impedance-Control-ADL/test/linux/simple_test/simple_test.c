#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>
#include <math.h> // need to link to library m in cmakelist
#include <signal.h> // handle interrupted keyboard command ctrl-c
// #include <eigen3/Eigen/Dense>
// #include <vector>
#include "rt_tiboard_data.h"
#include "rt_ethercat.h"
#include "ethercat.h"
#include "config.h"
#include "lcm-types/ecat_data_t.h"
#include "lcm-types/ecat_command_t.h"
#include "lcm-types/ecat_adl_status_t.h"
#include "lcm-types/ecat_hard_enable_t.h"
#include "rt_gpio.h"
#include "rt_logger.h"

// typedef Matrix<float, 3, 1> Vec3;
// typedef Matrix<float, 3, 3> Mat3;

#define controller_dt 0.002
#define ESTOP_PORT 1

// using std::vector;
/*
  global variables
*/
TiBoardCommand tiboardcommand[4];
TiBoardData tiboarddata[4];
ecat_command_t lcmEcatCmd;
ecat_data_t lcmEcatData;   
ecat_adl_status_t lcmADLStatus;
ecat_hard_enable_t lcmHardEnable;
bool estop_status = 1;
bool hard_enable[4] = {1,1,1,1};
int control_mode = 0;

size_t iter_stand = 0;
bool standup_flag = false;
float init_pos[4][3];
float des_pos[4][3];
float current_pos[4][3];
float des_control_pos[4][3];
float h_max_default = 0.55;


pthread_t lcm_thread; 
pthread_t ecat_thread; 
pthread_t gpio_thread;
pthread_mutex_t cmd_mutex;
pthread_mutex_t data_mutex;

void standup_enter();
void standup_run();
void standup_control();
void run_controller();
void getEcatToLcm(ecat_data_t *_lcmEcatData, const TiBoardData* _tiBoardData);
void run_GUI_mannual_cmd(const ecat_command_t *_lcmEcatCmd, TiBoardCommand *_tiBoardCommand);
void enable_leg(int legidx, int value);
unsigned char read_ESTOP();
void *handleLCMthread(void *lcm);
void *handleECATthread(void *lcm);
void *handleGPIOthread(void *lcm);
void int_handler(int sig_num);
void zero_tiboardcmd();
void soft_enable(int value);
static void handlelcmEcatCmd(const lcm_recv_buf_t *rbuf, const char *channel, const ecat_command_t *msg,
                       void *user);
static void handleEcatHardEnableLcm(const lcm_recv_buf_t *rbuf, const char *channel, const ecat_hard_enable_t *msg,
                       void *user);

int main(int argc, char *argv[])
{
  // Initialize gpio and estop every leg
  init_gpio();
  estop();
  clock_t start, end;
  double time_elapse = 0;
  signal(SIGINT, int_handler);
  if (argc > 1)
  {
    lcm_t * lcm = lcm_create(NULL); // use the default LCM betwork provider
    // lcm_t * lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1"); // Sets the multicast TTL to 1 so that packets published will enter the local network
    if (!lcm)
      {
        printf("Failted to initialize LCM \n");
        return 1;
      }

    printf("LCM initialized!\n");

    /* Initialize ethercat */
    rt_ethercat_init(argv[1]);

    pthread_create(&ecat_thread, NULL, handleECATthread, lcm);
    pthread_create(&gpio_thread, NULL, handleGPIOthread, lcm);

    /* Subscribe to channel ecat_cmd_GUI channel*/
    ecat_command_t_subscribe(lcm, "ecat_cmd_GUI", &handlelcmEcatCmd, &lcmEcatCmd);

    /* Subscribe to channel ecat_hard_enable*/
    ecat_hard_enable_t_subscribe(lcm, "ecat_hard_enable", &handleEcatHardEnableLcm, &lcmHardEnable);

    pthread_mutex_init(&cmd_mutex, NULL);

    pthread_create(&lcm_thread, NULL, handleLCMthread, lcm);  

    /* Call controller every controller_dt seconds*/
    start = clock();
    while (1)
      {  
          end = clock();
          time_elapse = ((double) (end - start)) / CLOCKS_PER_SEC;
          if (time_elapse >= controller_dt)
          {
            start = clock();
            switch (control_mode)
            {
              case 0:
                run_GUI_mannual_cmd(&lcmEcatCmd, tiboardcommand);
                break;
              case 1:
                run_controller();
                break;
            }
          }          
                    
      }
      printf("While loop breaks \n");
      /*Once finished, stop EtherCAT and destroy allocated memory for LCM */
      rt_ethercat_stop();
      lcm_destroy(lcm);
  }
  else
  {
    printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
  }   
  printf("End program\n");   
  exit(0);  // terminate all threads 
  return (0);
}

void int_handler(int sig_num)
{
    estop();
    printf("[Impedance-Control-ADL] SIGINT received!\n");
    time_t current_time = time(NULL);
    char* time_str = ctime(&current_time);
    printf("ESTOP. Shutting down due to SIGINT (likely Ctrl-C) at %s\n", time_str);
    printf("[Impedance-Control-ADL] Stopping...\n");
    exit(0);
}

void *handleLCMthread(void *lcm)
{
  while (1)
  {
    // handle received LCM message
    lcm_handle(lcm);
 }
  pthread_exit(NULL); // exit current thread
}

void *handleECATthread(void *lcm)
{
  while (1)
  {
    pthread_mutex_lock(&cmd_mutex);      
    /*send command to tiboard via EtherCAT */
    rt_ethercat_set_command_3DOF(tiboardcommand);
    rt_ethercat_run();

    /* Get ecat data from tiboard */
    rt_ethercat_get_data_3DOF(tiboarddata);

    /*publish tiboard data to development computer via LCM */
    getEcatToLcm(&lcmEcatData, tiboarddata);  
    pthread_mutex_unlock(&cmd_mutex);   
   
    // publish ecat_data data
    if (ecat_data_t_publish(lcm, "ecat_data", &lcmEcatData) < 0)
    {
      printf("Failed to publish ecat_data \n");
    }      
 
    usleep(1000); // suspend execution for 1 ms
  }
  pthread_exit(NULL); // exit current thread
}

void *handleGPIOthread(void *lcm)
{
  while(1)
    {
      lcmADLStatus.estop_status = read_ESTOP();

      // If ESTOP, disable all legs
      if (lcmADLStatus.estop_status)
      {
        estop();   
        hard_enable[0] = 0;
        hard_enable[1] = 0;
        hard_enable[2] = 0;
        hard_enable[3] = 0;
      }
      else
      {
        // update harde_nable status if it is changed
        bool update_flag = 0;
        for (int leg = 0; leg < 4; ++leg)
        {
            if (hard_enable[leg] != lcmHardEnable.hard_enable[leg]) // if hard_enable status changed, reset leg hard_enable
            {
                hard_enable[leg] = lcmHardEnable.hard_enable[leg]; 
                update_flag = 1;
                enable_leg(leg, hard_enable[leg]);
            }    
        }
        if(update_flag) // if hard_enable status is changed, update gpio
        {
          update_gpio();
        }  
      }

      // check ethercat status
      for (int leg = 0; leg < 4; ++leg)
      {
        lcmADLStatus.ecat_status[leg] = ec_slave[leg+1].state;
      }
      // publish ecat_adl_status data
      if (ecat_adl_status_t_publish(lcm, "ecat_adl_status", &lcmADLStatus) < 0)
      {
        printf("Failed to publish ecat_adl_status \n");
      }
    }
  pthread_exit(NULL); // exit current thread
}

static void handlelcmEcatCmd(const lcm_recv_buf_t *rbuf, const char *channel, const ecat_command_t *msg,
                       void *user)
{  
  memcpy(user, msg, sizeof(ecat_command_t));
}

static void handleEcatHardEnableLcm(const lcm_recv_buf_t *rbuf, const char *channel, const ecat_hard_enable_t *msg,
                       void *user)
{
  // update lcmHardEnable with new data
  memcpy(user, msg, sizeof(ecat_hard_enable_t));  

  // update control mode
  control_mode = msg->control_mode;
}               


void getEcatToLcm(ecat_data_t *_lcmEcatData, const TiBoardData *_tiBoardData)
{  

  for (int leg = 0; leg < 4; ++leg)
  {
    _lcmEcatData->x[leg] = _tiBoardData[leg].position[0];
    _lcmEcatData->y[leg] = _tiBoardData[leg].position[1];
    _lcmEcatData->z[leg] = _tiBoardData[leg].position[2];
    _lcmEcatData->dx[leg] = _tiBoardData[leg].velocity[0];
    _lcmEcatData->dy[leg] = _tiBoardData[leg].velocity[1];
    _lcmEcatData->dz[leg] = _tiBoardData[leg].velocity[2];
    _lcmEcatData->fx[leg] = _tiBoardData[leg].force[0];
    _lcmEcatData->fy[leg] = _tiBoardData[leg].force[1];
    _lcmEcatData->fz[leg] = _tiBoardData[leg].force[2];
    _lcmEcatData->q_abad[leg] = _tiBoardData[leg].q[0];
    _lcmEcatData->q_hip[leg] = _tiBoardData[leg].q[1];
    _lcmEcatData->q_knee[leg] = _tiBoardData[leg].q[2];
    _lcmEcatData->dq_abad[leg] = _tiBoardData[leg].dq[0];
    _lcmEcatData->dq_hip[leg] = _tiBoardData[leg].dq[1];
    _lcmEcatData->dq_knee[leg] = _tiBoardData[leg].dq[2];
    _lcmEcatData->tau_abad[leg] = _tiBoardData[leg].tau[0];
    _lcmEcatData->tau_hip[leg] = _tiBoardData[leg].tau[1];
    _lcmEcatData->tau_knee[leg] = _tiBoardData[leg].tau[2];
    _lcmEcatData->tau_des_abad[leg] = _tiBoardData[leg].tau_des[0];
    _lcmEcatData->tau_des_hip[leg] = _tiBoardData[leg].tau_des[1];
    _lcmEcatData->tau_des_knee[leg] = _tiBoardData[leg].tau_des[2];
    _lcmEcatData->loop_count_ti[leg] = _tiBoardData[leg].loop_count_ti;
    _lcmEcatData->ethercat_count_ti[leg] = _tiBoardData[leg].ethercat_count_ti;
    _lcmEcatData->microtime_ti[leg] = _tiBoardData[leg].microtime_ti; 
  }    

}

void run_GUI_mannual_cmd(const ecat_command_t *_lcmEcatCmd, TiBoardCommand *_tiBoardCommand)
{
      standup_flag = 0;
      pthread_mutex_lock(&cmd_mutex); 
      for (int leg = 0; leg < 4; ++leg)
      {
         _tiBoardCommand[leg].position_des[0] = _lcmEcatCmd->x_des[leg];
         _tiBoardCommand[leg].position_des[1] = _lcmEcatCmd->y_des[leg];
         _tiBoardCommand[leg].position_des[2] = _lcmEcatCmd->z_des[leg];
         _tiBoardCommand[leg].velocity_des[0] = _lcmEcatCmd->dx_des[leg];
         _tiBoardCommand[leg].velocity_des[1] = _lcmEcatCmd->dy_des[leg];
         _tiBoardCommand[leg].velocity_des[2] = _lcmEcatCmd->dz_des[leg];
         _tiBoardCommand[leg].kp[0] = _lcmEcatCmd->kpx[leg];
         _tiBoardCommand[leg].kp[1] = _lcmEcatCmd->kpy[leg];
         _tiBoardCommand[leg].kp[2] = _lcmEcatCmd->kpz[leg];
         _tiBoardCommand[leg].kd[0] = _lcmEcatCmd->kdx[leg];
         _tiBoardCommand[leg].kd[1] = _lcmEcatCmd->kdy[leg];
         _tiBoardCommand[leg].kd[2] = _lcmEcatCmd->kdz[leg];
         _tiBoardCommand[leg].enable = _lcmEcatCmd->enable[leg];
         _tiBoardCommand[leg].zero = _lcmEcatCmd->zero_joints[leg];
         _tiBoardCommand[leg].force_ff[0] = _lcmEcatCmd->fx_ff[leg];
         _tiBoardCommand[leg].force_ff[1] = _lcmEcatCmd->fy_ff[leg];
         _tiBoardCommand[leg].force_ff[2] = _lcmEcatCmd->fz_ff[leg];
         _tiBoardCommand[leg].tau_ff[0] = _lcmEcatCmd->tau_abad_ff[leg];
         _tiBoardCommand[leg].tau_ff[1] = _lcmEcatCmd->tau_hip_ff[leg];
         _tiBoardCommand[leg].tau_ff[2] = _lcmEcatCmd->tau_knee_ff[leg];
         _tiBoardCommand[leg].q_des[0] = _lcmEcatCmd->q_des_abad[leg];
         _tiBoardCommand[leg].q_des[1] = _lcmEcatCmd->q_des_hip[leg];
         _tiBoardCommand[leg].q_des[2] = _lcmEcatCmd->q_des_knee[leg];
         _tiBoardCommand[leg].qd_des[0] = _lcmEcatCmd->qd_des_abad[leg];
         _tiBoardCommand[leg].qd_des[1] = _lcmEcatCmd->qd_des_hip[leg];
         _tiBoardCommand[leg].qd_des[2] = _lcmEcatCmd->qd_des_knee[leg];
         _tiBoardCommand[leg].kp_joint[0] = _lcmEcatCmd->kp_joint_abad[leg];
         _tiBoardCommand[leg].kp_joint[1] = _lcmEcatCmd->kp_joint_hip[leg];
         _tiBoardCommand[leg].kp_joint[2] = _lcmEcatCmd->kp_joint_knee[leg];
         _tiBoardCommand[leg].kd_joint[0] = _lcmEcatCmd->kd_joint_abad[leg];
         _tiBoardCommand[leg].kd_joint[1] = _lcmEcatCmd->kd_joint_hip[leg];
         _tiBoardCommand[leg].kd_joint[2] = _lcmEcatCmd->kd_joint_knee[leg];
         _tiBoardCommand[leg].max_torque = _lcmEcatCmd->max_torque[leg];
      }
     
     pthread_mutex_unlock(&cmd_mutex); 
}

void zero_tiboardcmd()
{
  memset(tiboardcommand, 0, 4*sizeof(TiBoardCommand));
}

void run_controller()
{
  memset(tiboardcommand, 0, 4*sizeof(TiBoardCommand));
  soft_enable(1);
  standup_control();
}

void standup_control()
{
  if (!standup_flag) 
  {   
    standup_enter();
    standup_flag = true;
  }
  else standup_run();
}

void standup_enter()
{
  iter_stand = 0;
  
  for (int leg = 0; leg < 4; ++leg)
  {    
    for (int i = 0; i < 3; ++i)
    {
      init_pos[leg][i] = tiboarddata[leg].position[i];
    }

  }
 
}

void standup_run()
{
  float h_max = fmin(lcmHardEnable.h_max, h_max_default);
  iter_stand ++;
  float progress;  

  for (int leg = 0; leg<4; ++leg)
  {
      des_pos[leg][0]  = 0 ; 
      des_pos[leg][1]  = 0.045;
      des_pos[leg][2]  = -h_max; 
  } 
  progress = iter_stand * controller_dt/5;

  float kp_cartesian[3] = {1000, 1000, 1000};
  float kd_cartesian[3] = {10, 10, 10};   

  if (progress > 1.){ progress = 1.; }

  for(int leg = 0; leg<4; ++leg){     
    for(int i=0; i<3; ++i)
    {
      des_control_pos[leg][i] = progress* des_pos[leg][i] + 
                        (1. - progress) * init_pos[leg][i]; 
      tiboardcommand[leg].position_des[i] = des_control_pos[leg][i];     
    }     
                                     
    memcpy(tiboardcommand[leg].kp, kp_cartesian, 3*sizeof(float));
    memcpy(tiboardcommand[leg].kd, kd_cartesian, 3*sizeof(float));
  }       
}

void enable_leg(int legidx, int val)
{
  switch (legidx)
    {
      case 0:      
      set_gpio(0, !val);
      set_gpio(1, !val);
      set_gpio(8, !val);
      break;
    case 1:
      set_gpio(2, !val);
      set_gpio(9, !val);
      set_gpio(10, !val);
      break;
    case 2:
      set_gpio(3, !val);
      set_gpio(4, !val);
      set_gpio(11, !val);
      break;
    case 3:
      set_gpio(5, !val);
      set_gpio(12, !val);
      set_gpio(13, !val);
      break;    
    }
}

void soft_enable(int value)
{
  for (int leg = 0; leg < 4; ++leg)
  {
    tiboardcommand[leg].enable = value;
    if (value == 1)
      tiboardcommand[leg].max_torque = 50;
    else
      tiboardcommand[leg].max_torque = 0;      
  }
}


// if estop return 1, else return 0
unsigned char read_ESTOP()
{
  unsigned char estop_data=0x0;

  // read port 1 (GPIO10 ~ GPIO17 where GPIO10 is the lowest bit) to estop_data
  gpio_read(ESTOP_PORT, &estop_data);
  
  // GPIO17 is estop bit
  return !(estop_data >> 7); 
}       

