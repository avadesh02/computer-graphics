// This file contains the functions to load meshes
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <Eigen/Core>
#include "raster.h"

using namespace std;
using namespace Eigen;


// loads the off file
void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F);
// computes the normal of the faces for different shading
void compute_normals(MatrixXd &V, MatrixXi &F, MatrixXf &V_p, std::vector<VertexAttributes> &vertices_mesh, std::vector<VertexAttributes> &vertices_lines, UniformAttributes & uniform);
// computes transformation matrices
void compute_transformation_matrices(MatrixXd &V, double frameBuffer_cols, double frameBuffer_rows, UniformAttributes& uniform);