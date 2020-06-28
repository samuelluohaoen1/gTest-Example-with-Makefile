#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <boost/asio.hpp>


class GrSim_Console{
    typedef boost::asio::ip::udp udp;
    typedef boost::asio::io_service io_service;
    typedef boost::shared_ptr<udp::socket> socket_ptr; // smart pointer(no need to mannually deallocate
    static const unsigned int BUF_SIZE = 256;

private:
    io_service *ios;
    udp::endpoint *ep;
    socket_ptr socket;

public:
    GrSim_Console(io_service& io_srvs, udp::endpoint& endpoint);

    ~GrSim_Console() {}

    void send_command(bool is_team_yellow, int id, 
                      float upper_left_wheel_speed, 
                      float lower_left_wheel_speed,
                      float lower_right_wheel_speed, 
                      float upper_right_wheel_speed, 
                      float kick_speed_x, float kick_speed_y, 
                      bool spinner);
};


#endif