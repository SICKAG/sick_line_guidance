#include "sick_line_guidance_demo/pid.h"
//using namespace std;

/* Object contstructor */ 
PID_Controller::PID_Controller(double dt, double kp, double ki, double kd){
    _dt = dt;
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

/* Function to calculate controller output */
double PID_Controller::calculate_output(double setpoint_value, double actual_value){
    _e_z = _e; // store old deviation in e_z (e*z⁻1)
    _e = setpoint_value - actual_value; // calculate deviation
    _e_int += _e; // integrate
    _e_diff = _e - _e_z; //derivate

    _yp = _kp * _e; // calculate P-Controller output
    _yi = _ki * _dt * _e_int; // calculate I-Controller output
    _yd = _kd * _e_diff/_dt; // calculate D-Controller output

    return (_yp + _yi + _yd); // calculate PID-Controller output
}

/* Function to set controller parameter */
void PID_Controller::setParams(double dt, double kp, double ki, double kd){
    _dt = dt;
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID_Controller::getParams(double& dt, double& kp, double& ki, double& kd){
    dt = _dt;
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void PID_Controller::reset(){
    _dt = 0;
    _kp = 0;
    _ki = 0;
    _kd = 0;
    _e = 0;
    _e_int = 0;
    _e_diff = 0;
    _e_z = 0;
    _yp = 0;
    _yi = 0;
    _yd = 0;
}

void PID_Controller::clear_state(){
    double params[4];
    getParams(params[0], params[1], params[2], params[3]);
    reset();
    setParams(params[0], params[1], params[2], params[3]);
}

PID_Controller::~PID_Controller(){}