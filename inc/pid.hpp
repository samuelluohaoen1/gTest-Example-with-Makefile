#ifndef __PID_H_
#define __PID_H_

#include <iostream>

/* C++ templated class can't not separate .hpp & .cpp file, 
   all function definition is cramped inside .hpp file due 
   to this limitation */

template <class T> // This PID code can handle (math)vector value
class PID_Controller {
private:
    
    bool is_first_time;
    bool is_fixed_time_interval;
    bool use_output_range = false;
    T integral;
    T prev_error;
    double period_ms; //unit: millisec 
    double prev_time_ms; // unit: millisec
    T out_upper_bound;
    T out_lower_bound;
    double (*millis_func)(void);
    
    T first_time_handle(T curr_error) {
        this->integral = curr_error;
        this->prev_error = curr_error;
        this->is_first_time = false;
        return bounded_output(Kp * curr_error);
    }

    T bounded_output(T output) {
        if(!use_output_range) return output;
        if(output > out_upper_bound) output = out_upper_bound;
        if(output < out_lower_bound) output = out_lower_bound;
        return output;
    }

    double get_period() {
        if(this->is_fixed_time_interval) {
            return this->period_ms;
        }
        else {
            double curr_time_ms = this->millis_func(); 
            double dt = curr_time_ms - this->prev_time_ms;
            this->prev_time_ms = curr_time_ms;
            // std::cout << dt << std::endl; // debug
            return dt;
        }
    }

public:
    // proportional, integral and derivative constants
    double Kp, Ki, Kd;
    
    PID_Controller(double Kp, double Ki, double Kd) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
    }

    // Fixed time interval mode, often coupled with a timer callback 
    void init(double frequency_Hz) {
        this->period_ms = (1.00 / frequency_Hz) * 1000.00;
        this->is_first_time = true;
        this->is_fixed_time_interval = true;
    }

    // Dynamic time interval mode, need to pass in a function handle to measure the curr time in millisec 
    void init(double (*millis)(void)) {
        this->millis_func = millis;
        this->is_first_time = true;
        this->is_fixed_time_interval = false;
    }

    // optionally config the output range    
    void set_output_range(T lower_bound, T upper_bound) {
        this->use_output_range = true;
        out_lower_bound = lower_bound;
        out_upper_bound = upper_bound;
    }


    /** calculate the PID output **/
    /* used when there is only one source of data measuring the error  */
    T calculate(T curr_error) {
        if(is_first_time) return first_time_handle(curr_error);
        double period = get_period();
        T derivative = (curr_error - prev_error) / period; 
        this->integral += curr_error * period;
        T output = (Kp * curr_error) - (Kd * derivative) + (Ki * integral);
        prev_error = curr_error;
        return bounded_output(output);
    }
    
    /* methods below are normally used when there exists 
       direct measurement of error_rate or error_sum that is 
       more accurate or stable than the calculated rate/sum 
       on noisy error measurement itself  */ 

    /* used when [error measurement] and [error derivative measurement] come
       from different data sources  */
    T calculate(T curr_error, T error_rate) {
        if(is_first_time) return first_time_handle(curr_error);
        double period = get_period(); 
        this->integral += curr_error * period;
        T output = (Kp * curr_error) - (Kd * error_rate) + (Ki * integral);
        return bounded_output(output);
    }
    
    /* used when [error] and [error integral] both 
       have their respective way of measuring directly */
    T calculate_(T curr_error, T error_sum) {
        if(is_first_time) return first_time_handle(curr_error);
        double period = get_period();
        T derivative = (curr_error - prev_error) / period; 
        T output = (Kp * curr_error) - (Kd * derivative) + (Ki * error_sum);
        return bounded_output(output);
    }

    /* used when [error], [error derivative] and [error integral] 
       all have their respective way of measuring directly */
    T calculate(T curr_error, T error_rate, T error_sum) {
        return (Kp * curr_error) - (Kd * error_rate) + (Ki * error_sum);
    }
    

};

#endif