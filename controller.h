#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <fstream>
#include "math_type_define.h"


#define EYE(X) Matrix<double, X, X>::Identity()

using namespace std;
using namespace Eigen;

class ArmController
{




//FILE *fp1;
//FILE *fp2;

private:
  void printState();
//void moveJointPosition(const Vector7d &target_position, double duration);
//void moveJointPositionTorque(const Vector7d &target_position, double duration);

public:

  size_t dof_;

// Initial state
Vector1d q_init_;
Vector1d qdot_init_;

// Current state
Vector1d q_;
Vector1d qdot_;
Vector1d qddot_;
Vector1d torque_;
Vector1d q_error_sum_;
Vector1d torque_ext;
Vector1d r_init_prev;
Vector1d r_init;
Vector1d q_init_desired;
Vector1d g_;

Vector1d torque_ext_model;

Vector1d m_;
Vector1d torque_prev1;
Vector1d qddot_temp_;
Vector1d q_desired_prev;
Vector1d  torque_1k;
Vector2d x_state_prev;
Vector2d x_state;

Vector1d q_prev;
Vector1d qimp_ddot;
Vector1d q_ddot_desired_;
Vector1d q_ddot_temp_;
bool apply_force =false;
Vector1d q_command_;
Vector1d 		qd_desired_prev;
// Control value (position controlled)
Vector1d q_desired_; // Control value
Vector1d qdot_desired_;
Vector1d torque_desired_;
Vector1d torque_desired_1;
Vector1d torque_desired_2;
Vector1d q_desired_motor_;
Vector1d q_desired_tick1_;
Vector1d q_desired_tick2_;
Vector1d q_desired_tick3_;
Vector1d q_desired_tick4_;
Vector1d q_desired_tick5_;

//dob
Vector1d d_p;
Vector1d d_p_pre;
Vector1d dist_hat;
Vector1d u_hat;

  unsigned long tick_;
  double play_time_;
  double hz_;
  double control_start_time_;

  std::string control_mode_;
  bool is_mode_changed_;

void readData(const Vector1d &position, const Vector1d &velocity, const Vector1d &torque);
void readData(const Vector1d &position, const Vector1d &velocity);
const Vector1d & getDesiredPosition();
const Vector1d & getDesiredTorque();
double lowPassFilter1(double input, double prev, double ts, double tau);
public:

  void setMode(const std::string & mode);
  void initDimension();
  void initModel();
  void initPosition();
  void compute();

  ArmController();

  ~ArmController();
};
