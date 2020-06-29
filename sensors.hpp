#ifndef __SENSORS_H
#define __SENSORS_H

#include "common.hpp"


class GrSim_Vision { 
private:   
    typedef boost::asio::ip::udp udp;
    typedef boost::asio::io_service io_service;

    static const unsigned int BUF_SIZE = 115200;
    static arma::vec blue_loc_vecs[NUM_ROBOTS];
    static arma::vec yellow_loc_vecs[NUM_ROBOTS];
    typedef boost::shared_ptr<udp::socket> socket_ptr; // smart pointer(no need to mannually deallocate
    typedef boost::shared_ptr<boost::array<char, BUF_SIZE>> buffer_array_ptr;

    io_service *ios;
    udp::endpoint *ep;
    socket_ptr socket;
    buffer_array_ptr receive_buffer; 
    udp::endpoint *local_listen_ep;
    boost::mutex mu;

    void publish_robots_vinfo(const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>& robots,
                             team_color_t team_color);
    
public:

    GrSim_Vision(io_service& io_srvs, udp::endpoint& grsim_endpoint);
    ~GrSim_Vision();

    void receive_packet();
    arma::vec& get_robot_loc_vec(team_color_t color, int robot_id);
    arma::vec get_robot_location(team_color_t color, int robot_id);
    float get_robot_orientation(team_color_t color, int robot_id);
    static arma::vec *get_robots_loc_vecs(team_color_t color);
    static void print_robot_vinfo(const SSL_DetectionRobot& robot);
};

std::ostream& operator<<(std::ostream& os, const arma::vec& v);

class Sensor_System { // corresponding to one particular robot, though multiple robots shared the same vision data source
private:
    typedef boost::asio::ip::udp udp;
    typedef boost::asio::io_service io_service;
    typedef boost::shared_ptr<GrSim_Vision> GrSim_Vision_ptr;
    typedef boost::shared_ptr<boost::thread> thread_ptr;

    team_color_t color;
    int id;
    GrSim_Vision_ptr vision;
    thread_ptr v_thread;
    boost::mutex mu;
    boost::condition_variable_any cond_init_finished;

    void vision_thread(udp::endpoint& v_ep);

public:
    Sensor_System(team_color_t color, int robot_id, udp::endpoint& grsim_vision_ep);
    arma::vec& get_location_vector();
 };


#endif