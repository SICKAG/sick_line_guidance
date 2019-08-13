#ifndef PID_H
#define PID_H

// using namespace std;

class PID_Controller
{
    public:

        PID_Controller(double dt, double kp, double ki, double kd); //Contstructor

        double calculate_output(double setpoint_value, double actual_value);
        void setParams(double dt, double kp, double ki, double kd);
        void getParams(double& dt, double& kp, double& ki, double& kd);
        void reset();
        void clear_state();
        ~PID_Controller(); //Destructor

    private:
        double _kp; // Proportional Factor
        double _ki; // Integrate Factor
        double _kd; // Derivation Factor
        double _dt; // Sample Time
        double _e = 0; // controller deviation
        double _e_int = 0; // controller integrator
        double _e_diff = 0; // controller derivator
        double _e_z = 0; // e * z⁻1
        double _yp = 0; // P-Controller Output
        double _yi = 0; // I-Controller Output
        double _yd = 0; // D-Controller Output
};
#endif