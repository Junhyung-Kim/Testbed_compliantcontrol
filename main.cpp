#include "red_ec_master.h"
//#include "walking_controller.h"
#include "sensoray826.h"
#include "controller.h"

#include <Eigen/Dense>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include "omd/opto.h"
#include <unistd.h>


#include <rbdl/rbdl.h>

#define DOF 1
//#define DOF 2
#define DEG2RAD  0.01745329251
#define RAD2DEG  57.2957795147

#define RAD2CNT  318309.887
#define CNT2RAD  0.00000314159265
#define CNT2NM  0.058598
#define NM2CNT  1/CNT2NM

//#define RAD2CNT  83443.0268
//#define CNT2RAD  0.000011984225
//#define NM2CNT  7.692
//#define CNT2NM  0.13
using namespace EtherCAT_Elmo;
using namespace Eigen;

EthercatElmoBridge elmo;
ArmController _controller_;
//WalkingCtrl _WalkingCtrl;
sensoray826_dev _sensoray826_dev;
enum SLOT_TIME {NONE = 0, DEFAULT = 3};
const SLOTATTR slotAttrs[16] = {
    {0, 3}, {0, DEFAULT}, {0, DEFAULT}, {0, DEFAULT},
    {0, DEFAULT}, {0, DEFAULT}, {0, DEFAULT}, {0, DEFAULT},
    {0, DEFAULT}, {0, DEFAULT}, {0, DEFAULT}, {0, DEFAULT},
    {0, DEFAULT}, {0, DEFAULT}, {0, DEFAULT}, {0, DEFAULT}
};

ElmoGoldDevice::elmo_gold_rx *rxPDO[DOF];
ElmoGoldDevice::elmo_gold_tx *txPDO[DOF];

VectorXd start_positionElmo(DOF);

VectorXd positionElmo(DOF);
VectorXd velocityElmo(DOF);
VectorXd torqueElmo(DOF);
VectorXd torqueDemandElmo(DOF);
double F_zzz;
double F_zzz_low;
bool first_torque_sensor = false;
bool first_torque_sensor_bias = false;

VectorXd positionDesiredElmo(DOF);
VectorXd velocityDesiredElmo(DOF);
VectorXd torqueDesiredElmo(DOF);

pthread_t thread1, thread2, thread3, thread4,thread5;

OptoDAQ daq;
OptoPorts ports;
OPort* portlist;

double torque_sensor_[3];
double torque_sensor_1[3];
double torque_sensor_prev[3];
double torque_sensor_bias;
double jhtoff =0;
double F_zzz_prev;
int jhcnt2=0;

char *ifname;
volatile int wkc;

static struct termios initial_settings, new_settings;
static int peek_character = -1;

//KOJS 20190308 -----
double _tChirp = 0.0;
double amplitudeChirp = 3.0;
double freqChirp = 0.0;
double torqueChirp = 0.0;
double PI = 3.1415926535;
//KOJS 20190308 -----


//int to_controller()
//{
//  _controller_.q_init_ = jhqinit;
//  _controller_.q_ = positionElmo(slave-1);


//  return 0;
//}



int kbhit(void)
{


  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec > NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

void ethercatTask(void *ptr)
{
    char IOmap[4096];

    int wkc_count;
    boolean needlf = FALSE;
    boolean inOP = FALSE;
    int jhcnt=0;
    double jhqinit=0.0;
    double jhkp=400;  //14400    // low torque control PD 400,40
    double jhkv=40;   //240
    double jhqd=0;
    double jhqgoal=2.858;
    int jhmode=1; //1 : position mode, else : torque mode
    int16_t jhtd=0;

    double traj_qd=0;


    struct timespec current, begin, time;
    double elapsed = 0.0, elapsed_sum = 0.0, elapsed_avg = 0.0, elapsed_var = 0.0, prev = 0.0, now = 0.0, current_time = 0.0, begin_time = 0.0;
    double elapsed_time[10000] = {0.0};
    static int elapsed_cnt = 0, max_cnt=0, min_cnt=0;
    double elapsed_min = 210000000.0, elapsed_max= 0.0;
    double time_mem[10000]={0.0};

    bool reachedInitial[DOF] = {false};
   // int Walking_State = 0;



    ofstream fout_positionElmo("/home/dyros/dyros_red_ethercat_master/positionElmo.txt");
    ofstream fout_positionDesiredElmo("/home/dyros/dyros_red_ethercat_master/positionDesiredElmo.txt");
    ofstream fout_velocityElmo("/home/dyros/dyros_red_ethercat_master/velocityElmo.txt");
    ofstream fout_velocityDesiredElmo("/home/dyros/dyros_red_ethercat_master/velocityDesiredElmo.txt");
    ofstream fout_torqueElmo("/home/dyros/dyros_red_ethercat_master/torqueElmo.txt");
    ofstream fout_torqueDesiredElmo("/home/dyros/dyros_red_ethercat_master/torqueDesiredElmo.txt");
    ofstream fout_torqueSensor("/home/dyros/dyros_red_ethercat_master/torqueSensor.txt", std::ofstream::out);

    struct sched_param schedp;
    memset( &schedp, 0, sizeof(schedp) );
    schedp.sched_priority = 49;
    if(sched_setscheduler(0, SCHED_FIFO, &schedp) == -1) {
        exit(EXIT_FAILURE);
    }

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        _controller_.initDimension();

        /* find and auto-config slaves */
        /* network discovery */
        if ( ec_config_init(FALSE) > 0 ) // TRUE when using configtable to init slaves, FALSE otherwise
        {
            printf("%d slaves found and configured.\n",ec_slavecount); // ec_slavecount -> slave num

            /** CompleteAccess disabled for Elmo driver */
            for(int slave=1; slave<=ec_slavecount; slave++)
            {
                printf("Has Slave[%d] CA? %s\n", slave, ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA ? "true":"false" );
                ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

            for(int slave=1; slave<=ec_slavecount; slave++)
            {
                uint16 map_1c12[2] = {0x0001, 0x1605};
                uint16 map_1c13[4] = {0x0003, 0x1a04, 0x1a11, 0x1a12};
                int os;
                os=sizeof(map_1c12);
                ec_SDOwrite(slave,0x1c12,0,TRUE,os,map_1c12,EC_TIMEOUTRXM);
                os=sizeof(map_1c13);
                ec_SDOwrite(slave,0x1c13,0, TRUE, os,map_1c13,EC_TIMEOUTRXM);
            }

            /** if CA disable => automapping works */
            ec_config_map(&IOmap);


            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            printf("Request operational state for all slaves\n");
            int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            /* request OP state for all slaves */
            ec_writestate(0);

            int wait_cnt = 40;

            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 5000);
            }
            while (wait_cnt-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("Operational state reached for all slaves.\n");
                wkc_count = 0;
                inOP = TRUE;

                /* cyclic loop */
                for(int slave=1; slave<=ec_slavecount; slave++)
                {
                    txPDO[slave-1] = (ElmoGoldDevice::elmo_gold_tx *)(ec_slave[slave].outputs);
                    rxPDO[slave-1] = (ElmoGoldDevice::elmo_gold_rx *)(ec_slave[slave].inputs);
                }

                struct timespec ts;
                int64 cycletime;

                cycletime = *(int*)ptr * 1000; /* cycletime in ns */

                clock_gettime(CLOCK_MONOTONIC, &ts);
                clock_gettime(CLOCK_MONOTONIC, &begin);
                prev = begin.tv_sec; prev += begin.tv_nsec/1000000000.0;

                while(!kbhit())
                {
                    /* wait to cycle start */
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

                    //KOJS 20190308 -----
                    _tChirp = now - prev;
                    freqChirp = _tChirp * 2.5;
                    torqueChirp = amplitudeChirp * sin(2.0*PI*_tChirp*freqChirp);
                    //KOJS 20190308 -----

                    /** PDO I/O refresh */
                    ec_send_processdata();
                    wkc = ec_receive_processdata(250);

                    if(wkc >= expectedWKC)
                    {

                        for(int slave=1; slave<=ec_slavecount; slave++)
                        {
                            if(elmo.controlWordGenerate(rxPDO[slave-1]->statusWord, txPDO[slave-1]->controlWord))
                            {
                                reachedInitial[slave-1] = true;
                            }
                        }


                            for(int slave=1; slave<=ec_slavecount; slave++)
                            {
                                if(reachedInitial[slave-1])
                                {
                                    positionElmo(slave-1) = rxPDO[slave-1]->positionActualValue*CNT2RAD;
                                    _controller_.q_(0) = positionElmo(slave-1);
                                    velocityElmo(slave-1) =
                                            (((int32_t)ec_slave[slave].inputs[14]) +
                                            ((int32_t)ec_slave[slave].inputs[15] << 8) +
                                            ((int32_t)ec_slave[slave].inputs[16] << 16) +
                                            ((int32_t)ec_slave[slave].inputs[17] << 24)) * CNT2RAD;
                                    _controller_.qdot_(0) = velocityElmo(slave-1);
                                    torqueDemandElmo(slave-1) =
                                            (int16_t)((ec_slave[slave].inputs[18]) +
                                            (ec_slave[slave].inputs[19] << 8))*CNT2NM;
                                    torqueElmo(slave-1) = rxPDO[slave-1]->torqueActualValue*CNT2NM;

                                    //txPDO[slave-1]->modeOfOperation  = ProfileTorquemode;
                                    //txPDO[slave-1]->modeOfOperation  = CyclicSynchronousTorquemode;

                                    //txPDO[slave-1]->modeOfOperation = ProfilePositionmode;
                                    //txPDO[slave-1]->targetTorque    = (int)torqueDesiredElmo(slave-1) * NM2CNT[slave-1];


                                    //-----to controller-----//

                                    //to_controller();



                                    //---position---//
                                    _controller_.play_time_ = jhcnt;

                                    if (jhcnt==0){ jhqinit= positionElmo(slave-1);
                                      jhqd = jhqinit;
                                      _controller_.q_init_desired(0) = jhqinit;
                                      traj_qd = jhqinit;
                                      jhqgoal = jhqinit - 0.2;
                                    }
                                    else
                                    {
                                      //jhqd = _controller_.q_desired_(0);
                                    }

                                    _controller_.torque_(0) = torque_sensor_[0];
                    
                                    //Controller

                                    //Computetorque
                                    _sensoray826_dev.analogOversample();
                                    double torqueover =0;
                                    for(int i = 0;i<16;i++){
                                      torqueover -= _sensoray826_dev.adcVoltages[i];
                                    }
                                    torqueover= torqueover/16.0;
                                    torque_sensor_[2] = 0;
                                    torque_sensor_[1] = 0;
                                    //torque_sensor_[0] = _sensoray826_dev.adcVoltages[0]*20;
                                    torque_sensor_[0] = torqueover;

                                    //jhcnt2++;
                                    for(int i=0; i<3; i++)
                                     {
                                         torque_sensor_[i] = torque_sensor_[i]*9.807;
                                     }
                                     if(first_torque_sensor==false)
                                     {
                                       torque_sensor_prev[0] = torque_sensor_[0];
                                       first_torque_sensor = true;
                                     }

                                     torque_sensor_[0] = ((1/20)*torque_sensor_prev[0] + (0.001)*torque_sensor_[0])/((1/20)+(0.001));
                                     if(first_torque_sensor_bias==false)
                                    {
                                      torque_sensor_bias = torque_sensor_[0];
                                      first_torque_sensor_bias = true;
                                    }
                                    torque_sensor_[0] =torque_sensor_[0] -torque_sensor_bias;
                                    torque_sensor_prev[0] = torque_sensor_[0];

                                    //----safe----//


                                    _controller_.compute();


                                    if(jhcnt!=0)
				    {
                                      jhqd = _controller_.q_desired_(0);
                                    }

                                    if(jhqd > jhqinit + 0.3)
                                    {
                                     jhqd = jhqinit + 0.3;
                                    }
                                    if(jhqd < jhqinit - 0.3)
                                    {
                                     jhqd = jhqinit - 0.3;
                                    }

                                    fout_torqueSensor<< " "<< jhcnt<< " " << jhtd << " " << F_zzz_low << " " << F_zzz << "\n";

                                    fout_torqueElmo << " "<< jhcnt<< " "  << torqueElmo(slave-1) << "\n";

                                    fout_torqueDesiredElmo << " " << torque_sensor_[0]<< " "<< _controller_.torque_ext(0)<<" " << _controller_.q_(0)<< " "<<  _controller_.qimp_ddot(0) << " " << _controller_.torque_(0) << " "<< _controller_.g_(0)<<"   " << _controller_.qdot_desired_(0)/4000.0<<"   " << _controller_.torque_ext<<"   " << _controller_.play_time_<<"   " << _controller_.q_desired_prev<<"   " << _controller_.qdot_desired_(0)<<"   " << jhqinit << "\n";

                                    fout_positionDesiredElmo << jhcnt << "  " << jhqd <<  "  " << positionElmo(slave-1) << "  " << _controller_.dist_hat << "\n";


                                    jhcnt ++;
                                    if (jhcnt%500==0){ //100
                                      //printf("cnt %d td %lf t %lf qd %lf q %lf \n",jhcnt/1000,(double)jhtd*CNT2NM,(double)torqueElmo(slave-1),jhqd,positionElmo(slave-1));
                                    //  printf("cnt %d td %lf t %lf qd %d q %d \n",jhcnt/1000,(double)jhtd*CNT2NM,(double)torqueElmo(slave-1),(int)(jhqd*RAD2CNT),(int)(positionElmo(slave-1)*RAD2CNT));
                                  //    printf("torque %.3lf \n", torque_sensor_[0]);
                                      //printf("jhtd %.6lf \n", (double)jhtd);
                                    //  printf("jhqd %.6lf \n", jhqd);
                                      //printf("dist_hat %.6lf \n", dist_hat);
                                //     printf("d_p %.10lf \n", (double)_controller_.d_p(0));
                                //      printf("cnt %d td %lf t %lf qd %lf q %lf \n",jhcnt/1000,(double)jhtd*CNT2NM,(double)torqueElmo(slave-1),jhqd,positionElmo(slave-1));

                                      std::cout << "torque sensor" << torque_sensor_[0] << "   " <<  torque_sensor_prev[0] << "  " <<torque_sensor_bias<< std::endl;

                                    }
                                    jhtd = 0;
                                    if(jhmode==1){
                                      txPDO[slave-1]->modeOfOperation  = CyclicSynchronousPositionmode;
                                      txPDO[slave-1]->targetPosition = (int32_t)(jhqd*RAD2CNT);
                                    }
                                    else{
                                      txPDO[slave-1]->modeOfOperation  = CyclicSynchronousTorquemode;
                                      txPDO[slave-1]->targetTorque    = (int16_t)jhtd;
                                    }

                                    txPDO[slave-1]->maxTorque       = (uint16)300;

                                }
                            }

                        needlf = TRUE;

                    }
                    clock_gettime(CLOCK_MONOTONIC, &time);
                    now = time.tv_sec; now += time.tv_nsec/1000000000.0;
                    elapsed_time[elapsed_cnt] = now - prev;
                    prev = now;

                    elapsed_sum += elapsed_time[elapsed_cnt];
                    if(elapsed_min>elapsed_time[elapsed_cnt]) elapsed_min = elapsed_time[elapsed_cnt];
                    if(elapsed_max<elapsed_time[elapsed_cnt]) elapsed_max = elapsed_time[elapsed_cnt];

                    time_mem[elapsed_cnt] = (elapsed_time[elapsed_cnt] - (cycletime/1000000000.0)) * 1000;

                    if(++elapsed_cnt >= 100)
                    {
                        elapsed_avg = elapsed_sum/elapsed_cnt;
                        for(int i=0; i<elapsed_cnt; i++)
                        {
                            elapsed_var += (elapsed_time[i]-elapsed_avg)*(elapsed_time[i]-elapsed_avg);
                            if(elapsed_time[i]>elapsed_avg+0.00010) max_cnt++;
                            if(elapsed_time[i]<elapsed_avg-0.00010) min_cnt++;
                        }

                        elapsed_var = elapsed_var/elapsed_cnt;
                        //printf("avg = %.3lf\tmin = %.3lf\tmax = %.3lf\tvar = %.6lf\tmax_cnt=%d\tmin_cnt=%d\tcnt = %d\n", elapsed_avg*1000, elapsed_min*1000, elapsed_max*1000, elapsed_var*1000000, max_cnt, min_cnt, elapsed_cnt);
                        //printf("torqued %d %d \n",D_torque,torqueDemandElmo(1));

                        max_cnt = 0;
                        min_cnt = 0;
                        elapsed_sum = 0;
                        elapsed_var = 0;
                        elapsed_cnt = 0;
                        elapsed_min=210000000.0;
                        elapsed_max=0.0;
                    }

                   add_timespec(&ts, cycletime);
                }
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(int slave = 1; slave<=ec_slavecount ; slave++)
                {
                    if(ec_slave[slave-1].state != EC_STATE_OPERATIONAL)
                    {
                        printf("EtherCAT State Operation Error : Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               slave-1, ec_slave[slave-1].state, ec_slave[slave-1].ALstatuscode, ec_ALstatuscode2string(ec_slave[slave-1].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            /** request INIT state for all slaves
             *  slave number = 0 -> write to all slaves
             */
            ec_slave[0].state = EC_STATE_INIT;
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }

 //   fout_torqueSensor<<"torqueSensor" << torque_sensor_[0] <<"\n" ;
}

void computeTorque(void *ptr)
{
  while(1)
  {

   // _controller_.torque_(0) = torque_sensor_[0];
  }
}

int computeFtsensor(void *ptr)
{
  ofstream fout_FTSensor("/home/dyros/dyros_red_ethercat_master/FTSensor.txt");

  if (ports.getLastSize()>0)
  {
      while(1){
      daq.open(portlist[0]);

      if (daq.getVersion()!=_95 && daq.getVersion() != _64) // It is a 3D sensor
      {
          OptoPackage pack3D;
          int size=daq.read(pack3D,false);	// Reading Sensor #0 (up to 16 Sensors)
      }
      else					  // It is a 6D sensor
      {
          OptoPackage6D pack6D;
          int size=daq.read6D(pack6D,false);
          std::cout<<" Fz: "<<pack6D.Fz<<std::endl;
          fout_FTSensor<< " " << pack6D.Fz << "\n";
      }
      daq.close();
      }
  }
  else
  {
      std::cout<<"No sensor available"<<std::endl;
  }
  return 0;
}

double lowPassFilter(double input, double prev, double ts, double tau)
{
    return (tau*prev + ts*input)/(tau+ts);
}

int main(int argc, char *argv[])
{

    //Torque sensor init

    _sensoray826_dev.open();
    _sensoray826_dev.analogSingleSamplePrepare(slotAttrs, 16);

    printf("Sensoray Board DAQ Setting Complete\n");

    usleep(2500 * 1000); // We wait some ms to be sure about OptoPorts enumerated PortList

    //FTsensor

    portlist=ports.listPorts(true);

    printf("FT sensor init");

    printf("SOEM (Simple Open EtherCAT Master)\nRed EtherCat Master\n");

    if (argc > 2)
    {
        ifname = argv[1];
        int ctime = atoi(argv[2]);
        int ctime1 = 1000;

        /* create thread to handle slave error handling in OP */
        osal_thread_create( &thread1, NULL, (void *) &EthercatElmoBridge::ethercatCheck, NULL);

        /* create RT thread*/
        osal_thread_create_rt( &thread3, NULL, (void *) &computeTorque, (void *) &ctime);

        osal_thread_create_rt( &thread2, NULL, (void *) &ethercatTask, (void *) &ctime);

        osal_thread_create_rt( &thread4, NULL, (void *) &computeFtsensor, (void *) &ctime);

        /* start cyclic part */
        pthread_join( thread3, NULL);
        pthread_join( thread2, NULL);
        pthread_join( thread4, NULL);
    }
    else
    {
        printf("Usage: dyros_red_ethercat_master ifname1 \nifname = eth0 for example\n");
    }

    printf("End program\n");
    return (0);
}

