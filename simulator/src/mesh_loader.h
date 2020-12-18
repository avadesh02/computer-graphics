// This file contains the functions to load meshes
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

// JSON parser library (https://github.com/nlohmann/json)
#include "json.hpp"
using json = nlohmann::json;

#include <Eigen/Core>
#include "raster.h"
#include "object.h"
#include "integrator.h"

using namespace std;
using namespace Eigen;


// loads the off file
void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F);
// computes the normal of the faces for different shading
void  compute_normal(Object& object, UniformAttributes & uniform);

void compute_normals(vector<Object>& objects, UniformAttributes & uniform);

// computes transformation matrices
void compute_transformation_matrices(vector<Object> &objects, double frameBuffer_cols, double frameBuffer_rows, 
										UniformAttributes& uniform);

void load_scene(const std::string &filename, UniformAttributes& uniform, vector<Object> &objects, Integrator &integrator);

void compute_camera_angle(UniformAttributes &uniform);

void init_object(const std::string &filename, vector<Object> &objects);
