// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

// Utilities for the Assignment
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F) {
	std::ifstream in(filename);
	std::string token;
	in >> token;
	int nv, nf, ne;
	in >> nv >> nf >> ne;
	V.resize(nv, 3);
	F.resize(nf, 3);
	for (int i = 0; i < nv; ++i) {
		in >> V(i, 0) >> V(i, 1) >> V(i, 2);
	}
	for (int i = 0; i < nf; ++i) {
		int s;
		in >> s >> F(i, 0) >> F(i, 1) >> F(i, 2);
		assert(s == 3);
	}
}

int main() 
{

	// The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(500,500);

	// Global Constants (empty in this example)
	UniformAttributes uniform;

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		Vector4f transformed_vector;
		for(unsigned i = 0; i < 3; i ++){
			transformed_vector[i] = va.position[i];
		}
		transformed_vector[3] = 1;
		transformed_vector = uniform.M*transformed_vector;
		//replacing 4th co-ordinate with alpha
		return VertexAttributes(transformed_vector[0],transformed_vector[1],transformed_vector[2], va.position[3]);
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(1,0,0);
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	// initialising camera attributes before rendering mesh
	uniform.camera.position << 0,0,-2;
	uniform.camera.gaze_direction << 0,0,-1;
	uniform.camera.view_up << 0,1,0;
	uniform.camera.is_perspective = false;

	// loading the mesh
	MatrixXd V;
	MatrixXi F;
	std::string filename = "../data/bunny.off" ;

	load_off(filename, V, F);

	// pushing triangle information into vertices
	vector<VertexAttributes> vertices_mesh;
	for (unsigned i = 0; i < F.rows(); i++){
		for (unsigned j = 0; j < F.cols(); j ++){
			vertices_mesh.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
		}
	}
	
	// computing the transformation matrices
	// computing transformation from wold to camera
	Vector3d w, u, v;
	w = -uniform.camera.gaze_direction.normalized();
	u = (uniform.camera.view_up.cross(w)).normalized();
	v = w.cross(u);

	Matrix4f tmp;
	 tmp << u[0], v[0], w[0], uniform.camera.position[0],
					u[1], v[1], w[1], uniform.camera.position[1],
					u[2], v[2], w[2], uniform.camera.position[2],
					0, 0, 0, 1;

	uniform.M_cam = tmp.inverse();

	// compututing transformation from camera view to cannoincal view volume
	if (!uniform.camera.is_perspective){
		Vector4f lbn_world, rtf_world;
		for (unsigned i = 0; i < V.cols() ; i ++){
			lbn_world[i] = V.col(i).minCoeff();
			rtf_world[i] = V.col(i).maxCoeff();
			
		}
		lbn_world[3] = 1; rtf_world[3] = 1;
		// tranforming from world to camera frame
		uniform.lbn = (uniform.M_cam*lbn_world).head(3);
		uniform.rtf = (uniform.M_cam*rtf_world).head(3);
		// computing M_orth
		// not doing (n - f) here since, the bounded box limits are already transformed to the right axis before while
		// transforming the bounding box from the world frame to the camera frame
		uniform.M_orth << 2/(uniform.rtf(0) - uniform.lbn(0)), 0, 0, -(uniform.rtf(0) + uniform.lbn(0))/(uniform.rtf(0) - uniform.lbn(0)),
					0, 2/(uniform.rtf(1) - uniform.lbn(1)), 0, -(uniform.rtf(1) + uniform.lbn(1))/(uniform.rtf(1) - uniform.lbn(1)),
					0, 0, -2/(uniform.lbn(2) - uniform.rtf(2)), (uniform.rtf(2) + uniform.lbn(2))/(uniform.lbn(2) - uniform.rtf(2)),
					0, 0, 0, 1;
	}
	else{
		// to do for perspective
	}
	
	// M_vp is not computed as it is carried out in the rasterize triangle part
	// M_object to world is assumed to be identity 
	uniform.M = uniform.M_orth*uniform.M_cam;

	rasterize_triangles(program,uniform,vertices_mesh,frameBuffer);

	vector<uint8_t> image;
	framebuffer_to_uint8(frameBuffer,image);
	stbi_write_png("triangle.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
	
	return 0;
}
