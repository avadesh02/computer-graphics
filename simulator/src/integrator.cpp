// This file integrates the system using euler integration scheme and a spring damper model

#include "integrator.h"
#include <assert.h>     /* assert */

void Integrator::step(vector<Object> &objects){
    for (unsigned i = 0; i < objects.size(); i ++){
        if (!objects[i].is_fixed){
            if (objects[i].is_sphere){
                objects[i].displacement = objects[i].velocity*dt;
                objects[i].velocity += objects[i].mass*g*dt;
                for (unsigned j = 0; j < objects.size(); j ++){
                    if (j != i){
                        Vector3f contact_force;
                        if (check_collision(objects[i], objects[j], contact_force)){
                            objects[i].velocity += contact_force*dt;
                        };
                        
                    }
                }
            }
            objects[i].translate_object(objects[i].displacement[0], 
                        objects[i].displacement[1], objects[i].displacement[2]);
            objects[i].locate_center();
                
        }
    }
}

bool Integrator::check_collision(Object & object_1, Object & object_2, Vector3f &contact_force){
    contact_force << 0, 0, 0;
    if (object_1.box.intersects(object_2.box) && !object_1.is_fixed){
        if (object_1.is_sphere){
            if(!object_2.is_sphere){
                for (unsigned i = 0; i < 3; i ++){
                    for (int j = -1; j < 2; j += 2){
                        float plane;
                        plane = object_2.center_loc[i] + j*(object_2.box.sizes()[i])/2.0;
                        if(std::abs(object_1.center_loc[i] - plane) < 0.5*std::abs(object_1.box.sizes()[0])){
                            contact_force[i] = -kp[i]*(object_1.center_loc[i] + j*0.5*object_1.box.sizes()[0] - plane);
                            contact_force[i] -= kd[i]*object_1.velocity[i]; 
                            return true;
                        }
                    }
                }
            }
        }
    }
    else{
        return false;
    }
}
