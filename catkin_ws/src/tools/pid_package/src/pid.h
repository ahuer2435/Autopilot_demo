#ifndef PID_H
#define PID_H

#include <cmath>
#include <vector>

class PID {
private:
    int step;
    std::vector<double> changes;
    double best_error;
    double total_error;
    int index_param;

    int val_step;
    int test_step;

    int fail_counter;

    void IndexMove();

    bool need_twiddle;

public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;

    /*
    * Coefficients, the order is P, I, D
    */
    std::vector<double> parameter;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
};

#endif /* PID_H */
