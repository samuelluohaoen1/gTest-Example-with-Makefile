#include "common.hpp"
#include "systime.hpp"
#include "actuators.hpp"
#include "sensors.hpp"
#include "pid.hpp"

using namespace boost;
using namespace boost::asio;
using namespace arma;
typedef boost::asio::ip::udp udp;

double millis_wrapper() { return (double)millis();}

int main(int argc, char **argv) {



/*
    PID_Controller<double> pid_test(0.00, 1.00, 0.00);
    // pid_test.init(&millis_wrapper);
    pid_test.init(100); // 100Hz --> 10ms period
    // pid_test.set_output_range(-100, 100);
    double curr_error;
    while(1) {
        std::cin >> curr_error;
        std::cout << " :: "<< pid_test.calculate(curr_error) << std::endl;
    }
*/


    
    boost::mutex mu;
    udp::endpoint grsim_ssl_vision_ep(ip::address::from_string("224.5.23.3"), 10020);
    udp::endpoint grsim_console_ep(ip::address::from_string(LOCAL_HOST), 20011);
    
    Sensor_System sensors(BLUE, 0, grsim_ssl_vision_ep);
    Actuator_System actuators(BLUE, 0, grsim_console_ep);

    delay(500); 
    sensors.init();


    double m1, m2, m3, m4;
    vec d = {0, 0}, v = {0, 0}, prev_d = {0, 0}, prev_v = {0, 0};
    double theta, omega, prev_theta = 0.00, prev_omega = 0.00;
    while(1) {
        std::cin >> m1 >> m2 >> m3 >> m4;
        sensors.set_init_displacement();
        int t0 = millis();
        while(millis() - t0 < 1000) {
            actuators.set_wheels_speeds(m1, m2, m3, m4);
            d = sensors.get_translational_displacement();
            v = sensors.get_translational_velocity();
            theta = sensors.get_rotational_displacement();
            omega = sensors.get_rotational_velocity();
            if(!arma::approx_equal(v, prev_v, "absdiff", 0,00001) || omega != prev_omega ) { 
                std::cout << d << " " << v << " "
                          << theta << " " << omega << " " << std::endl; 
            }
            prev_d = d; prev_v = v; prev_theta = theta; prev_omega = omega;
        }
        t0 = millis();
        while(millis() - t0 < 100) actuators.stop();
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


/*
 * Armadillo C++ library Citation:
 * 
 * Conrad Sanderson and Ryan Curtin.
 * Armadillo: a template-based C++ library for linear algebra.
 * Journal of Open Source Software, Vol. 1, pp. 26, 2016.
 *
 * Conrad Sanderson and Ryan Curtin.
 * A User-Friendly Hybrid Sparse Matrix Class in C++.
 * Lecture Notes in Computer Science (LNCS), Vol. 10931, pp. 422-430, 2018. 
 */
