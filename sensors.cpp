#include "sensors.hpp"



using namespace std;
using namespace arma;
using namespace boost;
using namespace boost::asio;
using byte = unsigned char;



vec GrSim_Vision::blue_loc_vecs[NUM_ROBOTS];
vec GrSim_Vision::yellow_loc_vecs[NUM_ROBOTS];

void GrSim_Vision::publish_robots_vinfo(
    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>& robots,
    team_color_t team_color) 
{
    
    for(auto& bot : robots) {
        mu.lock();
        if(team_color == BLUE) {
            blue_loc_vecs[bot.robot_id()] = {bot.pixel_x(), bot.pixel_y(), bot.orientation()};
            // print_robot_vinfo(bot); // for debugging
        }
        if(team_color == YELLOW) {
            yellow_loc_vecs[bot.robot_id()] = {bot.x(), bot.y(), bot.orientation()};
            // print_robot_vinfo(bot); // for debugging
        }
        mu.unlock();
    }
}
    

GrSim_Vision::GrSim_Vision(io_service& io_srvs, udp::endpoint& grsim_endpoint) {
    this->ios = &io_srvs;
    this->ep = &grsim_endpoint;
    this->receive_buffer = buffer_array_ptr(new boost::array<char, BUF_SIZE>());
    this->socket = socket_ptr(new udp::socket(io_srvs));

    socket->open(grsim_endpoint.protocol());
    socket->set_option(udp::socket::reuse_address(true));
    socket->bind(grsim_endpoint);
    socket->set_option(ip::multicast::join_group(grsim_endpoint.address()));
    mu.lock();
    for(auto& vec : blue_loc_vecs) {
        vec = arma::vec("0 0 0");
    }
    for(auto& vec : yellow_loc_vecs) {
        vec = arma::vec("0 0 0");
    }
    mu.unlock();
}
GrSim_Vision::~GrSim_Vision() {}

void GrSim_Vision::receive_packet() {
    size_t num_bytes_received;
    std::string packet_string;
    SSL_WrapperPacket packet;
    google::protobuf::RepeatedPtrField<SSL_DetectionRobot> *blue_robots, *yellow_robots;
    try {
        num_bytes_received = socket->receive_from(asio::buffer(*receive_buffer), *ep);
        packet_string = std::string(receive_buffer->begin(), 
                                    receive_buffer->begin() + num_bytes_received);

        packet.ParseFromString(packet_string);
        
        publish_robots_vinfo(packet.detection().robots_blue(), BLUE);
        publish_robots_vinfo(packet.detection().robots_yellow(), YELLOW);

    }
    catch (std::exception& e) {
        // To-do : Exception Handling
        std::cout << "[Exception] " << e.what() << std::endl;
    }
}

vec& GrSim_Vision::get_robot_loc_vec(team_color_t color, int robot_id) {
    return color == BLUE ? GrSim_Vision::blue_loc_vecs[robot_id] 
                         : GrSim_Vision::yellow_loc_vecs[robot_id];
}

vec GrSim_Vision::get_robot_location(team_color_t color, int robot_id) {
    if(color == BLUE) {
        vec location = {GrSim_Vision::blue_loc_vecs[robot_id](0), 
                        GrSim_Vision::blue_loc_vecs[robot_id](1)};
        return location;
    }
    else {
        vec location = {GrSim_Vision::yellow_loc_vecs[robot_id](0), 
                        GrSim_Vision::yellow_loc_vecs[robot_id](1)};
        return location;
    }
}

float GrSim_Vision::get_robot_orientation(team_color_t color, int robot_id) {
    return color == BLUE ? GrSim_Vision::blue_loc_vecs[robot_id](2) 
                         : GrSim_Vision::yellow_loc_vecs[robot_id](2);
}


void GrSim_Vision::print_robot_vinfo(const SSL_DetectionRobot& robot) {
    // To-do : format string alignment
    std::cout << "ID[" << robot.robot_id() << "] "
                << "[<x,y>:(" << robot.x() << ", " << robot.y() << ")]"
                << "orien[" << robot.orientation() << "] "
                << "confidence[" << robot.confidence() << "]"
                << std::endl;
} 


arma::vec* GrSim_Vision::get_robots_loc_vecs(team_color_t color) {
    return color == BLUE ? GrSim_Vision::blue_loc_vecs
                         : GrSim_Vision::yellow_loc_vecs;
}

std::ostream& operator<<(std::ostream& os, const arma::vec& v)
{
    int num_rows = arma::size(v).n_rows;
    os << "<";
    for(int i = 0; i < num_rows; i++) {
        os <<  v(i);
        if(i != num_rows - 1) os << ", ";
    }
    os << ">";
    return os;
}

// ==================================================================================================== //

void Sensor_System::vision_thread(udp::endpoint& v_ep) {
    io_service ios;
    this->vision = GrSim_Vision_ptr(new GrSim_Vision(ios, v_ep));
    cond_init_finished.notify_all();
    while(1) {
        // collecting vision data packets from grSim in a background-running thread
        this->vision->receive_packet(); 
    } 
    ios.run();
}

Sensor_System::Sensor_System(team_color_t color, int robot_id, udp::endpoint& grsim_vision_ep) {
    this->color = color;
    this->id = robot_id;
    
    //save the thread_ptr copy to extend the life scope of the smart pointer thread_ptr 
    v_thread = thread_ptr(
        new boost::thread(boost::bind(&Sensor_System::vision_thread, this, grsim_vision_ep))
    );
    mu.lock();
    cond_init_finished.wait(mu);
    mu.unlock();
}

arma::vec& Sensor_System::get_location_vector() {
    return this->vision->get_robot_loc_vec(this->color, this->id);
}