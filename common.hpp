#ifndef __COMMON_H
#define __COMMON_H

#include <iostream>
#include <armadillo>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/array.hpp>

#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_refbox_log.pb.h"
#include "messages_robocup_ssl_robot_status.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"


using byte = unsigned char;
const int NUM_ROBOTS = 6;
enum team_color_t {
    BLUE,
    YELLOW
};

static constexpr const char* LOCAL_HOST = "127.0.0.1";

#endif