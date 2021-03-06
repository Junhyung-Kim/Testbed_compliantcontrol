#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <math.h>

void ArmController::compute()
{

  /////////////////////////////MOMENTUM OBSERVER/////////////////////////////////////

    Vector1d target_;
    Vector3d Quanticcc;
    Quanticcc.setZero();
    target_(0) = M_PI / 2;
    for (int i = 0; i < 1; i++)
    {
      Quanticcc = DyrosMath::QuinticSpline(play_time_, control_start_time_ + 0.2,control_start_time_ + 2,q_init_(i),0,0,target_(i),0,0);
    }
    q_desired_(0) = Quanticcc(0);
    qdot_desired_(0) = Quanticcc(1);
    q_ddot_desired_(0) = Quanticcc(2);
    m_(0) = 8.3117 *(0.116)*(0.116) + 0.16396;

    Vector1d M_d, K_d, D_d;

    M_d(0) = 1;
    K_d(0) = 15;
    D_d(0) = 10;
    double dt = 1 / 4000.0;

    if (control_start_time_ + 0.2 == play_time_)
    {
      qddot_(0) = 0;

      qddot_temp_ = qdot_;
      q_desired_prev = q_desired_;
      torque_prev1 = torque_;
      x_state << q_(0), qdot_(0);
      x_state_prev = x_state;
      q_ddot_desired_(0) = 0;
      q_ddot_temp_ = qdot_desired_;
    }
    else if (control_start_time_ + 0.2 <= play_time_)
    {
      qddot_ = (qdot_ - qddot_temp_)*hz_;
      qddot_temp_ = qdot_;
      q_ddot_desired_ = (qdot_desired_ - q_ddot_temp_)*hz_;
      q_ddot_temp_ = qdot_desired_;
    }
    else
    {
    }

    if (control_start_time_ + 0.2 <= play_time_)
    {
      for (int i = 0; i < 1; i++)
      {
        torque_1k(i) = lowPassFilter1(torque_(i), torque_prev1(i), 1 / hz_, 0.03);
      }
      //Momentum_observer
      Vector1d l_gain, p_mom; //r_init_prev
      l_gain(0) = 8;
      p_mom = m_ * qdot_;
      r_init = r_init_prev + (torque_1k + torque_ext)*dt; //Lowpass filtered
      r_init_prev = r_init;
      torque_ext =  l_gain * (p_mom - r_init);

      qimp_ddot = q_ddot_desired_ + M_d.inverse()*((K_d * (q_desired_ - q_)) + D_d * (qdot_desired_ - qdot_) - torque_ext);

      qdot_desired_ = qd_desired_prev + qimp_ddot * dt;
      q_desired_ = q_desired_prev+qdot_desired_ * dt;
      q_desired_prev = q_desired_;
      qd_desired_prev = qdot_desired_;

      torque_prev1 = torque_1k;
      torque_ext_model(0) = -torque_(0) - m_(0) * qddot_(0);

    }

  /////////////////////////////MOMENTUM OBSERVER END/////////////////////////////////////

  /////////////////////////////POSITION BASED JOINT IMPEDANCE CONTORL/////////////////////////////////////
   Vector1d target_;
    Vector3d Quanticcc;
    Quanticcc.setZero();
    target_(0) = M_PI / 2;
    for (int i = 0; i < 1; i++)
    {
       Quanticcc = DyrosMath::QuinticSpline(play_time_, control_start_time_ + 0.2,control_start_time_ + 2,q_init_(i),0,0,target_(i),0,0);
     }
    q_desired_(0) = Quanticcc(0);
    qdot_desired_(0) = Quanticcc(1);
    q_ddot_desired_(0) = Quanticcc(2);
   


  m_(0) = 0.5385;//(8.3117 *(0.116)*(0.116)+ 0.16396)*2;
  g_(0) = 0.2973*9.81*0.26538*sin(q_(0));
    Vector1d M_d, K_d, D_d;

    M_d(0) = 5;
      K_d(0) = 0;
    D_d(0) = 3;
    double dt = 1/1000.00000;
    if (play_time_ == 0)
    {
      qddot_(0) = 0;

      qddot_temp_ = qdot_;
      q_desired_prev = q_init_desired;
      q_desired_ = q_init_desired;
      torque_prev1 = torque_;
      q_ddot_desired_(0) = 0;
      q_ddot_temp_ = qdot_desired_;
    }
    else
    {
      qddot_ = (qdot_ - qddot_temp_)*hz_;
      qddot_temp_ = qdot_;
      q_ddot_desired_ = (qdot_desired_-q_ddot_temp_)*hz_;
      q_ddot_temp_ = qdot_desired_;
    }


      torque_ext(0) = -torque_(0) - m_(0) * qddot_(0);
      x_state_prev = x_state;


      torque_ext(0) =(torque_(0) - m_(0) * qddot_(0)-g_(0));
      qimp_ddot = q_ddot_desired_+M_d.inverse()*((K_d * (q_desired_ - q_)) + D_d * (qdot_desired_ - qdot_) - torque_ext);
      qimp_ddot =M_d.inverse()*((K_d * (q_desired_ - q_)) + D_d * ( - qdot_) - torque_ext);

      torque_prev1 = torque_1k;

      q_command_ = qdot_desired_;
      qdot_desired_ = qd_desired_prev +qimp_ddot*dt;
      q_desired_ = q_desired_prev+qdot_desired_ * dt;
      q_desired_prev = q_desired_;
      qd_desired_prev = qdot_desired_;


    /////////////////////////////END/////////////////////////////////////

  /////////////////////////////DISTURBANCE OBSERVER/////////////////////////////////////

      if (play_time_ == 0)
      {
        qddot_(0) = 0;

        qddot_temp_ = qdot_;
        q_desired_prev = q_init_desired;
        q_desired_ = q_init_desired;
        q_prev = q_;

      }


  //---position-dob---//
  u_hat = (1.0/(0.001*48.0))*(q_ + (0.001*48.0 - 1.0) * q_prev);
  
  d_p = u_hat - q_desired_;
  d_p_pre = d_p;
  dist_hat = 0.9 * d_p; //k_dist //0.005

 
  q_desired_ = q_ + dist_hat;
  
  q_prev = q_;


  /////////////////////////////DISTURBANCE OBSERVER END/////////////////////////////////////

  //printState();
}


void ArmController::printState()
{
  // TODO: Modify this method to debug your code

  static int DBG_CNT = 0;
  if (DBG_CNT++ > hz_ / 20.) //20
  {
    DBG_CNT = 0;
    //    cout << "torque  :\t";
    //cout << torque_(0) << endl;
    /*cout << "current  :\t";
    cout << std::fixed << std::setprecision(3) << torque_1k+g_ << endl;
    */

 //   cout << d_p(0) << endl;
  }
}



// Controller Core Methods ----------------------------

void ArmController::setMode(const std::string & mode)
{
  is_mode_changed_ = true;
  control_mode_ = mode;
  cout << "Current mode (changed) : " << mode << endl;
}
void ArmController::initDimension()
{
  dof_ = 1;
  qimp_ddot(0) = 0; torque_1k(0) = 0; qddot_(0) = 0;
  torque_prev1(0) = 0;  qddot_temp_(0) = 0;
  x_state_prev.setZero();
  x_state.setZero();
  q_desired_(0) = 0;
  q_desired_prev(0) = 0;
  qd_desired_prev(0) = 0;
  torque_ext(0) = 0;
  r_init_prev(0) = 0;
  r_init(0) = 0;
  torque_ext_model(0) = 0;
  q_ddot_desired_(0) = 0;


  //dob
  d_p(0) = 0;
  d_p_pre(0) = 0;
}

void ArmController::initModel()
{

}

void ArmController::readData(const Vector1d &position, const Vector1d &velocity, const Vector1d &torque)
{
  for (size_t i = 0; i < dof_; i++)
  {
    q_(i) = position(i);
    qdot_(i) = velocity(i);
    torque_(i) = torque(i);
  }


}
void ArmController::readData(const Vector1d &position, const Vector1d &velocity)
{
  for (size_t i = 0; i < dof_; i++)
  {
    q_(i) = position(i);
    qdot_(i) = velocity(i);
    torque_(i) = 0;
  }
}

const Vector1d & ArmController::getDesiredPosition()
{
  return q_desired_;
}

const Vector1d & ArmController::getDesiredTorque()
{
  return torque_desired_;
}

double ArmController::lowPassFilter1(double input, double prev, double ts, double tau)

{

  return (tau*prev + ts * input) / (tau + ts);

}

void ArmController::initPosition()
{
    q_init_ = q_;
    q_desired_ = q_init_;
}

ArmController::ArmController()
{

};

ArmController::~ArmController(){};
