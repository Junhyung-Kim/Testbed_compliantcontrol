#include <Eigen/Dense>
#include <stdio.h>
#include <fstream>
#include <iostream>

#include <rbdl/rbdl.h>

using namespace Eigen;

#define DOF 12
#define DEG2RAD  0.01745329251
#define RAD2DEG  57.2957795147

//#define pi       3.141592653589793238462

typedef Eigen::Matrix<double,6,1> Vector6d;

class WalkingCtrl
{
public:
 /*   WalkingCtrl();

    ~WalkingCtrl();*/
public:
  /*int _wmode;// sjh
	int _wmode2;
	int _wmode3;

	int _nstep;
	int _t_ws;

    VectorXd _q_leg;
    VectorXd _dq_leg;
    VectorXd _p_rf;
    Vector3d _p_lf;
    Matrix3d _o_rf;
    Matrix3d _o_lf;
    Matrix3d _do_rf;
    Matrix3d _do_lf;
    Matrix3d _to_rf;
    Matrix3d _to_lf;

    Vector3d _ip_rf;
    Vector3d _ip_lf;
    Vector3d _tp_rf;
    Vector3d _tp_lf;
    Vector3d _dp_rf;
    Vector3d _dp_lf;
    Vector6d _dq_rf;
    Vector6d _dq_lf;


    Vector3d _ip_pe;
    Vector3d _tp_pe;
    Vector3d _dp_pe;
    Vector3d _p_pe;

    Matrix3d _o_pe;
    Matrix3d _to_pe;

    Vector6d rq;
    Vector6d lq;
    Matrix3d r_rpehp;
    Matrix3d r_lpehp;
    Matrix3d r_rhpf;
    Matrix3d r_lhpf;


    Vector3d leg;
    Vector3d off;
    Vector3d p_rfh;
    Vector3d p_lfh;
    Matrix3d r_rhppe;
    Matrix3d r_lhppe;

    Matrix3d _Rotate_wth_Z;
    Matrix3d _Rotate_wth_Y;
    Matrix3d _Rotate_wth_X;

    double Rotate_wth_Z[3][3];
    double Rotate_wth_Y[3][3];
    double Rotate_wth_X[3][3];

    double Kx;
    double Ky;

    double t_d_s;
    double t_c_s;
    double t;
    double start_t;

    double lf_y;
    double wn;

    int t_d;
    int t_c;
    int tstep;
    double l_step;
    double h_step;
/*
	void nwalk(int t_s,int t_c,int t_d,double l_step,double h_step);
	void forwardk();
	void inversek();
    void foot_traj(const double start_t2, const double t2, const double td2, const double TT2, const double h0, const double B, const Vector3d X_init, const Vector3d X_current, Vector3d& foot_tra);//
    void com_traj(double start_t2, double t2, double td2, double TT2, const Vector3d COM_init, const double B, const double AA, const double wn, Vector3d& COM_trajectory, bool i);
*/

public:
/*	int Hz;
    int _cnt; //time tick
    int _walk_cnt; //time tick
    VectorXd _q; //Current Joint angle
    VectorXd _desired_q; // desired Joint angle
    Vector6d _LFT;
    Vector6d _RFT;
    Vector3d _Gyro;
    VectorXd _init_q;
    Vector3d CoM_trajectory;

    enum MODE_OF_WALKING
    {
        INIT = 1,
        SINGLE_INIT = 2,
        SINGLE = 3,
        WALKING = 4,
    };*/

public:
/*    double Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f);
    double Cubic_dot(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f);
*/    void _initialize();
  /*  void Init_walking_pose(VectorXd& outputP, VectorXd& outputV, int slave);
    void Init_S_walking_pose_down(VectorXd& outputP, VectorXd& outputV, int slave);
    void Init_S_walking_pose_up(VectorXd& outputP, VectorXd& outputV, int slave);
    void Init_S_walking_pose_up2(VectorXd& outputP, VectorXd& outputV, int slave);
    void compute(VectorXd& output);
    void Count();*//*
    Matrix3d Rotate_with_X(double rAngle);
    Matrix3d Rotate_with_Y(double rAngle);
    Matrix3d Rotate_with_Z(double rAngle);
*/};
