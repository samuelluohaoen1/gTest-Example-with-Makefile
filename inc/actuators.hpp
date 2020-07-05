#ifndef __ACTUATORS_H
#define __ACTUATORS_H

#include "common.hpp"
#include "sensors.hpp"


class GrSim_Console{
private:
    typedef boost::asio::ip::udp udp;
    typedef boost::asio::io_service io_service;
    typedef boost::shared_ptr<udp::socket> socket_ptr; // smart pointer(no need to mannually deallocate
    static const unsigned int BUF_SIZE = 256;

    io_service *ios;
    udp::endpoint *ep;
    socket_ptr socket;

public:
    GrSim_Console(io_service& io_srvs, udp::endpoint& endpoint);

    ~GrSim_Console();

    void send_command(bool is_team_yellow, int id, 
                      float upper_left_wheel_speed, 
                      float lower_left_wheel_speed,
                      float lower_right_wheel_speed, 
                      float upper_right_wheel_speed,
                      // float x, float y, float omega, 
                      float kick_speed_x, float kick_speed_y, 
                      bool spinner);
};


class Actuator_System {
private:
    typedef boost::asio::ip::udp udp;
    typedef boost::asio::io_service io_service;
    typedef boost::shared_ptr<GrSim_Console> GrSim_Console_ptr;
    typedef boost::shared_ptr<boost::thread> thread_ptr;
    typedef boost::shared_ptr<boost::asio::deadline_timer> timer_ptr;

    team_color_t color;
    int id;
    GrSim_Console_ptr console;
    thread_ptr v_thread;
    boost::mutex mu;
    boost::condition_variable_any cond_init_finished;
    unsigned int ctrl_period_ms = 10; // milliseconds  
    timer_ptr timer;

    arma::vec unit_vec_A = {0, 0}; // left-upper direction
    arma::vec unit_vec_B = {0, 0}; // right upper direction
    double max_trans = 0.00;
    double max_rot = 0.00;
    bool param_loaded = false;

    void send_cmd_thread(udp::endpoint& c_ep);
    void timer_expire_callback();

public:
    float wheel_upper_left_vel = 0.00, 
          wheel_lower_left_vel = 0.00, 
          wheel_lower_right_vel = 0.00, 
          wheel_upper_right_vel = 0.00;
    float kick_speed_x = 0.00, kick_speed_y = 0.00;
    bool dribbler_on = false;

    Actuator_System(team_color_t color, int robot_id, udp::endpoint& grsim_console_ep);
    void load_robot_params(arma::vec left_vec, arma::vec right_vec, 
                           double max_trans_mms, double max_rotat_ds);
    void set_ctrl_freq(float freq_Hz);
    void set_ctrl_period(float period_ms);

    // unit: rad/s
    inline void set_wheels_speeds(float upper_left, float lower_left, 
                                  float lower_right, float upper_right) {
        wheel_upper_left_vel = upper_left;
        wheel_lower_left_vel = lower_left;
        wheel_lower_right_vel = lower_right;
        wheel_upper_right_vel = upper_right;
    }

    inline void turn_on_dribbler() { dribbler_on = true; }
    inline void turn_off_dribbler() { dribbler_on = false; }
    inline void kick(float speed_x, float speed_y) { 
        kick_speed_x = speed_x; 
        kick_speed_y = speed_y; 
    }
    
    void stop();
    void rotate(float angular_velocity); 
    void move(arma::vec vec_2d); // cartesian vector
    void move(float angle, float speed); // polor coord

    /* Note: when coupled with pid, move is called by mobilize & sprint
     */


};


#endif