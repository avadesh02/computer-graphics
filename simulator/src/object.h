// contains function to scale and move objects

// This file contains the functions to load meshes
// C++ include
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include "attributes.h"

#ifndef OBJECT_H
#define OBJECT_H

using namespace std;
using namespace Eigen;

struct Object{

    MatrixXd V;
    MatrixXi F;
    MatrixXf V_p;
    Vector4f bary_center;
    vector<VertexAttributes> vertices_mesh;
    vector<VertexAttributes> vertices_lines;
    bool is_fixed;
    bool is_sphere;
    Vector4f center_loc; // center of box
    Vector3f displacement; // the amount the object has moved from its initial location
    Vector3f velocity;
    AlignedBox3f box; // used to compute collision and location
    Vector3f diffuse; // diffuse color of object
    
    double mass;

    void resize_object(float sx, float sy, float sz);
    void translate_object(float px, float py, float pz);
    void locate_center();
};

#endif