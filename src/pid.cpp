#include "pid.hpp"

template <class T>
PID_Controller<T>::PID_Controller(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

// Fixed time interval mode, often coupled with a timer callback 
template <class T>
void PID_Controller<T>::init(double frequency_Hz) {
    this->period_ms = (1.00 / frequency_Hz) * 1000.00;
    this->is_first_time = true;
    this->is_fixed_time_interval = true;
}

// Dynamic time interval mode, need to pass in a function handle to measure the time in correct unit
template <class T>
void PID_Controller<T>::init(double (*millis)(void)) {
    this->millis_func = millis;
    this->is_first_time = true;
    this->is_fixed_time_interval = false;
}


// optionally config the output range
template <class T>
void PID_Controller<T>::set_output_range(T lower_bound, T upper_bound) {
    this->use_output_range = true;
    out_lower_bound = lower_bound;
    out_upper_bound = upper_bound;
}


template <class T>
T PID_Controller<T>::first_time_handle(T curr_error) {
    this->integral = curr_error;
    this->prev_error = curr_error;
    if(!is_fixed_time_interval) this->prev_time_ms = millis_func();
    this->is_first_time = false;
    return bounded_output(Kp * curr_error);
}

template <class T>
T PID_Controller<T>::bounded_output(T output) {
    if(!use_output_range) return output;
    if(output > out_upper_bound) output = out_upper_bound;
    if(outout < out_lower_bound) output = out_lower_bound;
    return output;
}

template <class T>
double PID_Controller<T>::get_period() {
    if(this->is_fixed_time_interval) {
        return this->period_ms;
    }
    else {
        return this->millis_func() - this->prev_time_ms;
    }
}


/** calculate the PID output **/
/* used when there is only one source of data measuring the error  */
template <class T>
T PID_Controller<T>::calculate(T curr_error) {
    if(is_first_time) return first_time_handle(curr_error);
    double period = get_period();
    T derivative = (curr_error - prev_error) / period; 
    this->integral += curr_error * period;
    T output = (Kp * curr_error) - (Kd * derivative) + (Ki * integral);
    return bounded_output(output);
}

/* used when [error measurement] and [error derivative measurement] come
    from different data sources  */
template <class T>
T PID_Controller<T>::calculate(T curr_error, T error_rate) {
    if(is_first_time) return first_time_handle(curr_error);
    double period = get_period();
    T output; 
    this->integral += curr_error * period;
    T output = (Kp * curr_error) - (Kd * error_rate) + (Ki * integral);
    return bounded_output(output);
}


/* used when [error] and [error integral] both 
       have their respective way of measuring directly */
template <class T>
T PID_Controller<T>::calculate(T curr_error, T error_sum) {
    if(is_first_time) return first_time_handle(curr_error);
    double period = get_period();
    T output;
    T derivative = (curr_error - prev_error) / period; 
    T output = (Kp * curr_error) - (Kd * derivative) + (Ki * error_sum);
    return bounded_output(output);
}

/* used when [error], [error derivative] and [error integral] 
    all have their respective way of measuring directly */
template <class T>
T PID_Controller<T>::calculate(T curr_error, T error_rate, T error_sum) {
    return (Kp * curr_error) - (Kd * error_rate) + (Ki * error_sum);
}
