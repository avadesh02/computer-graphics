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

using namespace std;
using namespace Eigen;


// loads the off file
void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F);
// computes the normal of the faces for different shading
void compute_normals(MatrixXd &V, MatrixXi &F, MatrixXf &V_p, std::vector<VertexAttributes> &vertices_mesh, std::vector<VertexAttributes> &vertices_lines, UniformAttributes & uniform);
// computes transformation matrices
void compute_transformation_matrices(vector<MatrixXd> &V_arr, double frameBuffer_cols, double frameBuffer_rows, UniformAttributes& uniform);

void load_scene(const std::string &filename, UniformAttributes& uniform, vector<MatrixXd> &V_arr, 
					vector<MatrixXi> &F_arr, vector<MatrixXf> &V_p_arr, 
					vector<vector<VertexAttributes>> &vertices_mesh_arr,
					vector<vector<VertexAttributes>> &vertices_lines_arr,
					double frameBuffer_cols, double frameBuffer_rows);