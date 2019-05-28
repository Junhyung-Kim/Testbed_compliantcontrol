#include "walking_controller.h"
/*
WalkingCtrl::WalkingCtrl(){};

WalkingCtrl::~WalkingCtrl(){};*/

void WalkingCtrl::_initialize()
{
  /*  Hz = 4000;
    _q.resize(DOF);  _q.setZero();
    _init_q.resize(DOF);    _init_q.setZero();
    _desired_q.resize(DOF);  _desired_q.setZero();
    _Gyro.setZero();

    _cnt=0;
    _walk_cnt=0;

    _wmode=0;
    _wmode2=0;
    _wmode3=0;
    _nstep=0;
    _t_ws=0;

    _q_leg.resize(DOF);
    _q_leg.setZero();
    _dq_leg.resize(DOF);
    _dq_leg.setZero();

    _dp_rf.setZero();
    _dp_lf.setZero();
    _dq_rf.setZero();
    _dq_lf.setZero();

    _do_lf.setIdentity();
    _do_rf.setIdentity();

    _tp_rf.setZero();
    _tp_lf.setZero();
    _to_lf.setIdentity();
    _to_rf.setIdentity();
    _ip_rf.setZero();
    _ip_lf.setZero();

    _p_rf.setZero();
    _p_lf.setZero();
    _o_rf.setIdentity();
    _o_lf.setIdentity();

    _ip_pe.setZero();
    _tp_pe.setZero();
    _dp_pe.setZero();
    _p_pe.setZero();

    _o_pe.setIdentity();
    _to_pe.setIdentity();

    _Rotate_wth_Z.resize(3,3);  _Rotate_wth_Z.setZero();
    _Rotate_wth_Y.resize(3,3);  _Rotate_wth_Y.setZero();
    _Rotate_wth_X.resize(3,3);  _Rotate_wth_X.setZero();

    leg.setZero();
    leg(2)=-0.35;
    off.setZero();
    off(1)=0.1025;

    rq.setZero();
    lq.setZero();

    r_rpehp.setIdentity();
    r_lpehp.setIdentity();
    r_rhpf.setIdentity();
    r_lhpf.setIdentity();

    p_rfh.setZero();
    p_lfh.setZero();

    r_rhppe.setIdentity();
    r_lhppe.setIdentity();*/
}
/*
void WalkingCtrl::forwardk()
{
    for(int i=0;i<6;i++){
        rq(i)=_q(i+6);
        lq(i)=_q(i);
    }

    r_rpehp=Rotate_with_Z(-rq(0))*Rotate_with_X(rq(1))*Rotate_with_Y(rq(2));
    r_lpehp=Rotate_with_Z(-lq(0))*Rotate_with_X(lq(1))*Rotate_with_Y(-lq(2));
    r_rhpf=Rotate_with_Y(rq(3))*Rotate_with_Y(-rq(4))*Rotate_with_X(-rq(5));
    r_lhpf=Rotate_with_Y(-lq(3))*Rotate_with_Y(lq(4))*Rotate_with_X(-lq(5));

    _o_rf=r_rpehp*r_rhpf;
    _o_lf=r_lpehp*r_lhpf;
    _p_rf=-off+r_rpehp*leg+r_rpehp*Rotate_with_Y(rq(3))*leg;
    _p_lf=off+r_lpehp*leg+r_lpehp*Rotate_with_Y(-lq(3))*leg;
}


void WalkingCtrl::inversek()
{
    p_rfh=-off-_dp_rf;
    p_lfh=off-_dp_lf;

    _dq_rf(5)=-1.0*atan2(p_rfh(1),p_rfh(2));
    _dq_lf(5)=-1.0*atan2(p_lfh(1),p_lfh(2));

    _dq_rf(3)=acos(((p_rfh(2)*p_rfh(2)+p_rfh(0)*p_rfh(0))/(2.0*0.35*0.35*cos(_dq_rf(5))*cos(_dq_rf(5))))-1.0);
    _dq_lf(3)=-acos(((p_lfh(2)*p_lfh(2)+p_lfh(0)*p_lfh(0))/(2.0*0.35*0.35*cos(_dq_rf(5))*cos(_dq_rf(5))))-1.0);

    _dq_rf(4)=-1.0*(atan2(-1.0*p_rfh(0),p_rfh(2))-(_dq_rf(3)/2.0));
    _dq_lf(4)=atan2(-1.0*p_lfh(0),p_lfh(2))+(_dq_lf(3)/2.0);


    r_rhppe= Rotate_with_Y(_dq_rf(3))*Rotate_with_Y(-1.0*_dq_rf(4))*Rotate_with_X(-_dq_rf(5));
    r_lhppe= Rotate_with_Y(-1.0*_dq_lf(3))*Rotate_with_Y(_dq_lf(4))*Rotate_with_X(-_dq_lf(5));

    _dq_rf(1)=asin(r_rhppe(1,2));
    _dq_lf(1)=asin(r_lhppe(1,2));

    _dq_rf(2)=-1.0*asin(r_rhppe(0,2)/cos(_dq_rf(1)));
    _dq_lf(2)=asin(r_lhppe(0,2)/cos(_dq_lf(1)));

    _dq_rf(0)=asin(r_rhppe(1,0)/cos(_dq_rf(1)));
    _dq_lf(0)=asin(r_lhppe(1,0)/cos(_dq_lf(1)));

    for(int i=0; i<6;i++){
        _dq_leg(i+6)= _dq_rf(i);
        _dq_leg(i)= _dq_lf(i);
    }
}

void WalkingCtrl::Init_walking_pose(VectorXd &outputP, VectorXd &outputV,int slave)
{
    VectorXd target_q;
    target_q.resize(DOF);
    target_q.setZero();
    int index = 0;
    //right leg0-5
    target_q(index++) = 0*DEG2RAD;
    target_q(index++) = 0*DEG2RAD;
    target_q(index++) = 20*DEG2RAD;
    target_q(index++) = -40*DEG2RAD;
    target_q(index++) = -20*DEG2RAD;
    target_q(index++) = 0*DEG2RAD;
    //left leg 6~11
    target_q(index++) = 0*DEG2RAD;
    target_q(index++) = 0*DEG2RAD;
    target_q(index++) = -20*DEG2RAD;
    target_q(index++) = 40*DEG2RAD;
    target_q(index++) = 20*DEG2RAD;
    target_q(index++) = 0*DEG2RAD;

    outputP(slave) = Cubic(_cnt,0.0,5.0*Hz,_init_q(slave),0.0,target_q(slave),0.0);
    outputV(slave) = Cubic_dot(_cnt,0.0,5.0*Hz,_init_q(slave),0.0,target_q(slave),0.0);
}
void WalkingCtrl::Init_S_walking_pose_down(VectorXd &outputP, VectorXd &outputV,int slave)
{
    VectorXd target_q;
    target_q.resize(DOF);
    target_q.setZero();
    int index = 0;
    //right leg0-5
    target_q(index++) = 0*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;
    target_q(index++) = 20*DEG2RAD;
    target_q(index++) = -40*DEG2RAD;
    target_q(index++) = -20*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;
    //left leg 6~11
    target_q(index++) = 0*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;
    target_q(index++) = -20*DEG2RAD;
    target_q(index++) = 40*DEG2RAD;
    target_q(index++) = 20*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;

    outputP(slave) = Cubic(_cnt,0.0,4.0*Hz,_init_q(slave),0.0,target_q(slave),0.0);
    outputV(slave) = Cubic_dot(_cnt,0.0,4.0*Hz,_init_q(slave),0.0,target_q(slave),0.0);
}

void WalkingCtrl::Init_S_walking_pose_up(VectorXd &outputP, VectorXd &outputV,int slave)
{
    VectorXd target_q;
    target_q.resize(DOF);
    target_q.setZero();
    int index = 0;
    //right leg0-5
    target_q(index++) = 0*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;
    target_q(index++) = 20*DEG2RAD;
    target_q(index++) = -40*DEG2RAD;
    target_q(index++) = -20*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;
    //left leg 6~11
    target_q(index++) = 0*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;
    target_q(index++) = -45*DEG2RAD;
    target_q(index++) = 90*DEG2RAD;
    target_q(index++) = 45*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;

    outputP(slave) = Cubic(_cnt,0.0,2.0*Hz,_init_q(slave),0.0,target_q(slave),0.0);
    outputV(slave) = Cubic_dot(_cnt,0.0,2.0*Hz,_init_q(slave),0.0,target_q(slave),0.0);
}

void WalkingCtrl::Init_S_walking_pose_up2(VectorXd &outputP, VectorXd &outputV,int slave)
{
    VectorXd target_q;
    target_q.resize(DOF);
    target_q.setZero();
    int index = 0;
    //right leg0-5
    target_q(index++) = 0*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;
    target_q(index++) = 30*DEG2RAD;
    target_q(index++) = -60*DEG2RAD;
    target_q(index++) = -30*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;
    //left leg 6~11
    target_q(index++) = 0*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;
    target_q(index++) = -45*DEG2RAD;
    target_q(index++) = 90*DEG2RAD;
    target_q(index++) = 45*DEG2RAD;
    target_q(index++) = -11*DEG2RAD;

    outputP(slave) = Cubic(_cnt,0.0,2.0*Hz,_init_q(slave),0.0,target_q(slave),0.0);
    outputV(slave) = Cubic_dot(_cnt,0.0,2.0*Hz,_init_q(slave),0.0,target_q(slave),0.0);
}

void WalkingCtrl::foot_traj(const double t_s, const double t2, const double t_d, const double t_c, const double h_step, const double l_step, const Vector3d X_init, const Vector3d X_current, Vector3d& foot_tra)
{
    foot_tra=X_init;

    if ((t2) > (t_d+t_s) && (t2)<(t_c-t_d+t_s))
    {
        foot_tra(0)= X_init(0)+(l_step/pi)*(2*pi*(t2-t_s-t_d)/(t_c-2*t_d)-sin(2*pi*(t2-t_s-t_d)/(t_c-2*t_d)));
        foot_tra(2)= X_init(2)+(h_step/2)*(1-cos(2*pi*(t2-t_s-t_d)/(t_c-2*t_d)));
    }
    else if ((t2)>=(t_c-t_d+t_s) && (t2)<=(t_c+t_s))
    {
        foot_tra(0) = 2*l_step+X_init(0);
        foot_tra(2) = X_init(2);
    }
}

void WalkingCtrl::com_traj(double t_s, double t2, double t_d, double t_c, const Vector3d COM_init, const double l_step, const double lf_y, const double wn, Vector3d& COM_trajectory, bool i)
{
    t_d_s = t_d/Hz;
    t_c_s = t_c/Hz;
    t = t2/Hz;
    start_t=t_s/Hz;

    Kx = l_step*t_d_s*wn/(t_d_s*wn+tanh(wn*(t_c_s/2-t_d_s)));
    Ky = lf_y*t_d_s*wn*tanh(wn*(t_c_s/2-t_d_s))/(1+t_d_s*wn*tanh(wn*(t_c_s/2-t_d_s)));

    COM_trajectory=COM_init;

    if (t2<= t_s+t_d)
    {
        COM_trajectory(0)=(Kx/t_d_s)*(t-start_t)+COM_init(0);

        if (i == 0)
            COM_trajectory(1)=-(Ky/t_d_s)*(t-start_t)+COM_init(1);
        else
            COM_trajectory(1)=(Ky/t_d_s)*(t-start_t)+COM_init(1);
    }
    else if (t2>(t_d+t_s) && (t2)<(t_c-t_d+t_s))
    {

        COM_trajectory(0)=(Kx-l_step)*cosh(wn*(t-start_t-t_d_s))+Kx/(t_d_s*wn)*sinh(wn*(t-start_t-t_d_s))+l_step+COM_init(0);
        if (i ==0)
            COM_trajectory(1) = -(Ky-lf_y)*cosh(wn*(t-start_t-t_d_s))-(Ky/(t_d_s*wn))*sinh(wn*(t-start_t-t_d_s))-lf_y+COM_init(1);
        else
            COM_trajectory(1) = (Ky-lf_y)*cosh(wn*(t-start_t-t_d_s))+(Ky/(t_d_s*wn))*sinh(wn*(t-start_t-t_d_s))+lf_y+COM_init(1);
    }
    else if ((t2)>=(t_c-t_d+t_s) && (t2)<=(t_c+t_s))
    {
        COM_trajectory(0)=(2*l_step-Kx)+(Kx/t_d_s)*(t-start_t-t_c_s+t_d_s)+COM_init(0);

        if (i == 0)
            COM_trajectory(1)= -(Ky/t_d_s)*(t_c_s-(t-start_t))+COM_init(1);
        else
            COM_trajectory(1)= (Ky/t_d_s)*(t_c_s-(t-start_t))+COM_init(1);
    }
    else
    {
        COM_trajectory=COM_init;
    }
}

void WalkingCtrl::nwalk(int t_s,int t_c,int t_d ,double l_step,double h_step)
{
    if(_wmode==0)
    {
        _ip_pe = _p_pe;
        _ip_rf = _p_rf;
        _ip_lf = _p_lf;
        _wmode=1;
    }

    lf_y = (_ip_lf(1) - _ip_rf(1))/2.0;
    wn = sqrt(9.8/(-_ip_rf(2)-0.38));

    if(_wmode3==0)
    {
        com_traj(t_s,_walk_cnt,t_d, t_c/2.0, _ip_pe, l_step/4.0, lf_y, wn, _dp_pe, 0);
        foot_traj(t_s, _walk_cnt,t_d, t_c/2.0, h_step, l_step/2.0, _ip_lf, _p_lf, _tp_lf);
        _tp_rf=_ip_rf;
    }
    else if(_wmode2==0 && _wmode3==1)
    {
        com_traj(t_s,_walk_cnt,t_d, t_c, _ip_pe, l_step/2.0, lf_y, wn, _dp_pe, _wmode2);
        foot_traj(t_s, _walk_cnt,t_d, t_c, h_step, l_step, _ip_lf, _p_lf, _tp_lf);
        _tp_rf=_ip_rf;
    }
    else if (_wmode2==1 && _wmode3==1)
    {
        com_traj(t_s,_walk_cnt,t_d, t_c, _ip_pe, l_step/2.0, lf_y, wn, _dp_pe, _wmode2);
        foot_traj(t_s,_walk_cnt,t_d, t_c, h_step, l_step, _ip_rf, _p_rf, _tp_rf);
        _tp_lf=_ip_lf;
    }

    else if(_wmode2==0 && _wmode3==2)
    {
        com_traj(t_s,_walk_cnt,t_d, t_c, _ip_pe, l_step/4.0, lf_y, wn, _dp_pe, _wmode2);
        foot_traj(t_s, _walk_cnt,t_d, t_c, h_step, l_step/2, _ip_lf, _p_lf, _tp_lf);
        _tp_rf=_ip_rf;
    }
    else if(_wmode2==1 && _wmode3==2)
    {
        com_traj(t_s,_walk_cnt,t_d, t_c/2.0, _ip_pe, l_step/4.0, lf_y, wn, _dp_pe, _wmode2);
        foot_traj(t_s, _walk_cnt,t_d, t_c/2.0, h_step, l_step/2.0, _ip_rf, _p_rf, _tp_rf);
        _tp_lf=_ip_lf;
    }

    _dp_lf= _tp_lf-_dp_pe;
    _dp_rf =_tp_rf-_dp_pe;
}

void WalkingCtrl::compute(VectorXd& output)
{
    t_d = 0.8*Hz;
    t_c = 5.0*Hz;
    tstep=30;
    l_step = 0.0;
    h_step = 0.05;

    forwardk();

    if(_walk_cnt%t_c == t_c/2)
    {
        _nstep++;
        _wmode=0;
        _wmode2=!_wmode2;
        _t_ws=_walk_cnt;
        _wmode3=1;
    }

    if (_nstep<tstep)
    {
        nwalk(_t_ws,t_c,t_d,l_step,h_step);
    }
    else if(_nstep==tstep)
    {
        _wmode3=2;
        nwalk(_t_ws,t_c,t_d,l_step,h_step);
    }

    if(_walk_cnt== t_c*tstep+t_c){
        tstep-=1;
    }

    inversek();

    output = _dq_leg;
}

double WalkingCtrl::Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
{
    double rx_t ;
    if(rT<rT_0)
    {
        rx_t = rx_0;
    }
    else if(rT>=rT_0 && rT<rT_f)
    {
        rx_t = rx_0 + rx_dot_0*(rT-rT_0)
            + (3 * (rx_f - rx_0)/((rT_f-rT_0) * (rT_f-rT_0)) - 2 * rx_dot_0/(rT_f-rT_0) - rx_dot_f/(rT_f-rT_0))*(rT-rT_0)*(rT-rT_0)
            + (-2 * (rx_f - rx_0)/((rT_f-rT_0) * (rT_f-rT_0) * (rT_f-rT_0)) + (rx_dot_0 + rx_dot_f)/((rT_f-rT_0) * (rT_f-rT_0)))*(rT-rT_0)*(rT-rT_0)*(rT-rT_0);
    }
    else
    {
        rx_t = rx_f;
    }
    return (rx_t);
}

double WalkingCtrl::Cubic_dot(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
{
    double rxdot_t ;
    if(rT<rT_0)
    {
        rxdot_t = 0.0;
    }
    else if(rT>=rT_0 && rT<rT_f)
    {
        rT_f = rT_f/Hz;
        rT_0 = rT_0/Hz;
        rT = rT/Hz;

        rxdot_t =rx_dot_0
                + (3 * (rx_f - rx_0)/((rT_f-rT_0) * (rT_f-rT_0)) - 2 * rx_dot_0/((rT_f-rT_0) * (rT_f-rT_0)) - rx_dot_f/((rT_f-rT_0) * (rT_f-rT_0)))*(rT-rT_0)*2
                + (-2 * (rx_f - rx_0)/((rT_f-rT_0) * (rT_f-rT_0) * (rT_f-rT_0)) + (rx_dot_0 + rx_dot_f)/((rT_f-rT_0) * (rT_f-rT_0) * (rT_f-rT_0)))*(rT-rT_0)*(rT-rT_0)*3;
    }
    else
    {
        rxdot_t = 0.0;
    }

    return (rxdot_t);
}

Matrix3d WalkingCtrl::Rotate_with_X(double rAngle)
{
    _Rotate_wth_X(0, 0) = 1.0;
    _Rotate_wth_X(1, 0) = 0.0;
    _Rotate_wth_X(2, 0) = 0.0;

    _Rotate_wth_X(0, 1) = 0.0;
    _Rotate_wth_X(1, 1) = cos(rAngle);
    _Rotate_wth_X(2, 1) = sin(rAngle);

    _Rotate_wth_X(0, 2) = 0.0;
    _Rotate_wth_X(1, 2) = -sin(rAngle);
    _Rotate_wth_X(2, 2) = cos(rAngle);

    return(_Rotate_wth_X);
}

Matrix3d WalkingCtrl::Rotate_with_Y(double rAngle)
{
    _Rotate_wth_Y(0, 0) = cos(rAngle);
    _Rotate_wth_Y(1, 0) = 0.0;
    _Rotate_wth_Y(2, 0) = -sin(rAngle);

    _Rotate_wth_Y(0, 1) = 0.0;
    _Rotate_wth_Y(1, 1) = 1.0;
    _Rotate_wth_Y(2, 1) = 0.0;

    _Rotate_wth_Y(0, 2) = sin(rAngle);
    _Rotate_wth_Y(1, 2) = 0.0;
    _Rotate_wth_Y(2, 2) = cos(rAngle);

    return(_Rotate_wth_Y);
}

Matrix3d WalkingCtrl::Rotate_with_Z(double rAngle)
{
    _Rotate_wth_Z(0, 0) = cos(rAngle);
    _Rotate_wth_Z(1, 0) = sin(rAngle);
    _Rotate_wth_Z(2, 0) = 0.0;

    _Rotate_wth_Z(0, 1) = -sin(rAngle);
    _Rotate_wth_Z(1, 1) = cos(rAngle);
    _Rotate_wth_Z(2, 1) = 0.0;

    _Rotate_wth_Z(0, 2) = 0.0;
    _Rotate_wth_Z(1, 2) = 0.0;
    _Rotate_wth_Z(2, 2) = 1.0;

    return(_Rotate_wth_Z);
}

void WalkingCtrl::Count()
{
    _cnt++;
    _walk_cnt++;
}


*/
















