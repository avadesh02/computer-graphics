// This file integrates the system using euler integration scheme and a spring damper model

#include "object.h"

class Integrator{
    public:
        float T;
        float dt; // integration step
        Vector3f kp; // spring stiffness
        Vector3f kd; // spring damper
        Vector3f g; // gravity

    void step(vector<Object> &objects);
    bool check_collision(Object & object_1, Object & object_2, Vector3f &contact_force);
};


