#ifndef PID_H
#define PID_H

#include <stdint.h>

// lien youtube: https://www.youtube.com/watch?v=zOByx3Izf5U

struct pid {
    float Kp;
    float Ki;
    float Kd;

    float aim;

    float T;//sampling time of discrete controler in seconds
    float tau;//filter
    float differentiator;
    float integrator;

    float limMin;
    float limMax;

    float prev_error;
    float prev_value;
};

void pid_init(struct pid *pid);
float pid_compute(struct pid *pid, float value);

#endif /* PID_H */
