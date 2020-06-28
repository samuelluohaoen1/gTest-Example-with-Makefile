#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "systime.hpp"
#include "controller.hpp"


using namespace boost;
using namespace boost::asio;
typedef boost::asio::ip::udp udp;
static constexpr const char* LOCAL_HOST = "127.0.0.1";

int main(int argc, char **argv) {
    io_service ios;
    boost::mutex mu;

    boost::thread *threads[6];
    for(int i = 0; i < 6; i++) {

        threads[i] = new boost::thread( [i, &mu] () -> void  {
            mu.lock();
            int _i = i;
            std::cout << _i << std::endl;
            mu.unlock();
            udp::endpoint ep_grsim_console(ip::address::from_string(LOCAL_HOST), 20011);
            io_service ios;
            GrSim_Console console(ios, ep_grsim_console);
            while(1) {
                float speed = (_i + 1) * 10;
                console.send_command(false, _i, speed, speed, speed, speed, 0, 0, false);
                delay(10);
            }
        });
    }

    ios.run();

    for(auto& thread : threads) {
        thread->join();
        delete thread;
    }


  
    return 0;
}