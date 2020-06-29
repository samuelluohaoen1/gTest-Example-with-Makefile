#include "common.hpp"
#include "systime.hpp"
#include "controller.hpp"
#include "sensors.hpp"


using namespace boost;
using namespace boost::asio;
typedef boost::asio::ip::udp udp;

int main(int argc, char **argv) {
    boost::mutex mu;
    udp::endpoint grsim_ssl_vision_ep(ip::address::from_string("224.5.23.3"), 10020);
    Sensor_System sensors(BLUE, 0, grsim_ssl_vision_ep);


    io_service ios;
    udp::endpoint ep_grsim_console(ip::address::from_string(LOCAL_HOST), 20011);
    GrSim_Console console(ios, ep_grsim_console);

    while(1) {
        // std::cout << sensors.get_location_vector() << std::endl;
        std::cout // << sensors.get_translational_displacement() << " "
                  // << sensors.get_rotational_displacement() << " " 
                  << sensors.get_rotational_velocity() << " "
                  << std::endl;

        // console.send_command(false, 0, -20, -20, 10, 10, 0, 0, false);
        console.send_command(false, 0, -10, -10, -10, -10, 0, 0, false);
        delay(10);
    }

/*s
    boost::thread *threads[6];
    for(int i = 0; i < 6; i++) {
        threads[i] = new boost::thread( [i, &mu] () -> void  {
            io_service ios;
            
            mu.lock();
            std::cout << i << std::endl;
            mu.unlock();
            udp::endpoint ep_grsim_console(ip::address::from_string(LOCAL_HOST), 20011);
            GrSim_Console console(ios, ep_grsim_console);

            udp::endpoint grsim_ssl_vision_ep(ip::address::from_string("224.5.23.3"), 10020);
            GrSim_Vision sim_vision(ios, grsim_ssl_vision_ep);

            while(1) {
                float speed = (i + 1) * 10;
                console.send_command(false, i, speed, speed, speed, speed, 0, 0, false);
                sim_vision.receive_packet();
                delay(10);
            }
        });
    }

    for(auto& thread : threads) {
        thread->join();
        delete thread;
    }
*/

  
    return 0;
}