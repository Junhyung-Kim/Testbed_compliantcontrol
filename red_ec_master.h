#include <sched.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <thread>
#include <sys/time.h>
#include <termio.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <fstream>
#include <sys/mman.h>
#include <Eigen/Dense>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#define NSEC_PER_SEC 1000000000

#define CNT_TO_RAD_46 (3.141592*2/819200)//819200
#define RAD_TO_CNT_46 (1/(CNT_TO_RAD_46))

#define Kp_Yaw1    150000   //Hip
#define Kp_Roll1   500000   //Hip
#define Kp_Pitch1  500000   //Hip
#define Kp_Pitch2  500000   //Knee
#define Kp_Pitch3  500000   //Ankle
#define Kp_Roll2   650000   //Ankle

#define Kv_Yaw1    3000     //Hip
#define Kv_Roll1   3000     //Hip
#define Kv_Pitch1  3000     //Hip
#define Kv_Pitch2  2000     //Knee
#define Kv_Pitch3  3000     //Ankle
#define Kv_Roll2   5000     //Ankle

#define EC_TIMEOUTMON 500
#define DOF 12

const double CNT2RAD[DOF] =
{
    CNT_TO_RAD_46,
    CNT_TO_RAD_46,
    CNT_TO_RAD_46,
    CNT_TO_RAD_46,
    CNT_TO_RAD_46,
    CNT_TO_RAD_46,
    CNT_TO_RAD_46,
    CNT_TO_RAD_46,
    CNT_TO_RAD_46,
    CNT_TO_RAD_46,
    CNT_TO_RAD_46,
    CNT_TO_RAD_46
};

const double RAD2CNT[DOF] =
{
    RAD_TO_CNT_46,
    RAD_TO_CNT_46,
    RAD_TO_CNT_46,
    RAD_TO_CNT_46,
    RAD_TO_CNT_46,
    RAD_TO_CNT_46,
    RAD_TO_CNT_46,
    RAD_TO_CNT_46,
    RAD_TO_CNT_46,
    RAD_TO_CNT_46,
    RAD_TO_CNT_46,
    RAD_TO_CNT_46
};
const double EncoderOffset[DOF] =
{
    498672,
    536174
};
const double NM2CNT[DOF] =
{
    0.1724,
    0.2307,
    0.2834,
    0.2834,
    0.2834,
    0.0811,
    0.1724,
    0.2307,
    0.2834,
    0.2834,
    0.2834,
    0.0811
};

const double Kp[DOF] =
{
    Kp_Yaw1,
    Kp_Yaw1, //Kp_Roll1,
    Kp_Pitch1,
    Kp_Pitch2,
    Kp_Pitch3,
    Kp_Roll2,
    Kp_Yaw1,
    Kp_Roll1,
    Kp_Pitch1,
    Kp_Pitch2,
    Kp_Pitch3,
    Kp_Roll2
};

const double Kv[DOF] =
{
    Kv_Yaw1,
    Kp_Yaw1, //Kp_Roll1,
    Kv_Pitch1,
    Kv_Pitch2,
    Kv_Pitch3,
    Kv_Roll2,
    Kv_Yaw1,
    Kv_Roll1,
    Kv_Pitch1,
    Kv_Pitch2,
    Kv_Pitch3,
    Kv_Roll2
};

using namespace std;

namespace EtherCAT_Elmo
{
enum MODE_OF_OPERATION
{
    ProfilePositionmode = 1,
    ProfileVelocitymode = 3,
    ProfileTorquemode = 4,
    Homingmode = 6,
    InterpolatedPositionmode = 7,
    CyclicSynchronousPositionmode = 8,
    CyclicSynchronousVelocitymode = 9,
    CyclicSynchronousTorquemode = 10,
    CyclicSynchronousTorquewithCommutationAngle = 11
};

struct ElmoGoldDevice
{
    struct elmo_gold_tx {
        int32_t		targetPosition;
        int32_t		targetVelocity;
        int16_t		targetTorque;
        uint16_t	maxTorque;
        uint16_t	controlWord;
        int8_t      modeOfOperation;
    };
    struct elmo_gold_rx {
        int32_t		positionActualValue;
        int32_t		positionFollowingErrrorValue;
        int16_t		torqueActualValue;
        uint16_t	statusWord;
        int8_t		modeOfOperationDisplay;
        int32_t     velocityActualValue;
        int16_t     torqueDemandValue;
    };
};

class EthercatElmoBridge
{
    const int FAULT_BIT = 3;
    const int OPERATION_ENABLE_BIT = 2;
    const int SWITCHED_ON_BIT = 1;
    const int READY_TO_SWITCH_ON_BIT = 0;

    enum {
        CW_SHUTDOWN = 6,
        CW_SWITCHON = 7,
        CW_ENABLEOP = 15,
        CW_DISABLEOP = 7,
    };

public: // Methods
    EthercatElmoBridge();
    EthercatElmoBridge(int deviceCount);

    virtual ~EthercatElmoBridge();

    bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord);
    void ethercatCheck( void *ptr );
    void add_timespec(struct timespec *ts, int64 addtime);
    void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime);
    void ecatthread(void *ptr);

};

}

