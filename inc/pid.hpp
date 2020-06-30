#ifndef __PID_H_
#define __PID_H_


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
    
    T first_time_handle(T curr_error);
    T bounded_output(T raw_output);
    double get_period();

public:
    // proportional, integral and derivative constants
    double Kp, Ki, Kd;
    
    PID_Controller(double Kp, double Ki, double Kd);

    // Fixed time interval mode, often coupled with a timer callback 
    void init(double frequency_Hz);

    // Dynamic time interval mode, need to pass in a function handle to measure the curr time in millisec 
    void init(double (*millis)(void));

    // optionally config the output range    
    void set_output_range(T lower_bound, T upper_bound);


    /** calculate the PID output **/
    /* used when there is only one source of data measuring the error  */
    T calculate(T curr_error);
    
    /* methods below are normally used when there exists 
       direct measurement of error_rate or error_sum that is 
       more accurate or stable than the calculated rate/sum 
       on noisy error measurement itself  */ 

    /* used when [error measurement] and [error derivative measurement] come
       from different data sources  */
    T calculate(T curr_error, T error_rate);
    
    /* used when [error] and [error integral] both 
       have their respective way of measuring directly */
    T calculate(T curr_error, T error_sum);

    /* used when [error], [error derivative] and [error integral] 
       all have their respective way of measuring directly */
    T calculate(T curr_error, T error_rate, T error_sum);
    

};

#endif