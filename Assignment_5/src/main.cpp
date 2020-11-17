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
		Vector4f transformed_vector, transformed_normal;
		for(unsigned i = 0; i < 3; i ++){
			transformed_vector[i] = va.position[i];
			transformed_normal[i] = va.normal[i];
		}
		transformed_vector[3] = 1;
		transformed_normal[3] = 1;
		transformed_vector = uniform.M*transformed_vector;
		// transforming normal at the vertex into the canonical view volume space
		transformed_normal = uniform.M_inv*transformed_normal;
		//replacing 4th co-ordinate with alpha
		VertexAttributes out(transformed_vector[0],transformed_vector[1],transformed_vector[2], va.position[3]);
		out.normal = transformed_normal.head(3).normalized();
		return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		Vector3f Li = -(va.position.head(3) - uniform.light_source).normalized();
		// Vector3f bisector = ((uniform.light_source - va.position.head(3)) - uniform.).normalized()
		Vector3f diffuse, specular, color;
		if (uniform.light_source.dot(va.normal) > 0){
			diffuse = uniform.diffuse_color * (Li.dot(va.normal));
		}
		else{
			diffuse = uniform.diffuse_color * 0.0;
		}
		// implement specular 
		// specular = uniform.specular_color * pow(std::max(va.normal.dot(bisector), 0.0), uniform.specular_exponent);
		
		color = diffuse;
		// FragmentAttributes out(uniform.color(0),uniform.color(1),uniform.color(2),uniform.color(3));
		FragmentAttributes out(color(0),color(1),color(2),uniform.color(3));

		out.depth = va.position[2];
		return out;
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		// implementation of the z buffer
		if (fa.depth > previous.depth){
			FrameBufferAttributes out(fa.color[0]*255, fa.color[1]*255, fa.color[2]*255, fa.color[3]*255);
			out.depth = fa.depth;
			return out;
		}
		else{
			return previous;
		}
	};

	// initialising camera attributes before rendering mesh
	// TODO: check the frame buffer initialization  
	uniform.camera.position << 0,0,-2;
	uniform.camera.gaze_direction << 0,0,-1;
	uniform.camera.view_up << 0,1,0;
	uniform.camera.is_perspective = false;
	uniform.draw_wireframe = false;
	uniform.flat_shading = false;
	uniform.per_vertex_shading = true;

	uniform.color << 1,0,0,1;
	uniform.light_source << 0,2,2;
	uniform.diffuse_color << 0.8, 0.8, 0.8;
	uniform.specular_color << 0.2, 0.2, 0.2;
	uniform.specular_exponent = 265.0;

	// loading the mesh
	MatrixXd V;
	MatrixXi F;
	std::string filename = "../data/bunny.off" ;
	MatrixXf V_p;
	load_off(filename, V, F);
	if (uniform.per_vertex_shading){
		V_p.resize(V.rows(), V.cols());
		V_p.setZero();
		for (unsigned i = 0; i < F.rows(); i++){
			Vector3f u, v;
			u = V.row((F(i, 0))).cast <float> () - V.row(F(i, 1)).cast <float> ();
			v = V.row((F(i, 2))).cast <float> () - V.row(F(i, 1)).cast <float> ();
			V_p.row(F(i, 0)) = (-u.cross(v)).normalized();
			V_p.row(F(i, 1)) = (-u.cross(v)).normalized();
			V_p.row(F(i, 2)) = (-u.cross(v)).normalized();
		}
	}
	// pushing triangle information into vertices
	vector<VertexAttributes> vertices_mesh;
	// pushing line information into vertices
	vector<VertexAttributes> vertices_lines;
	for (unsigned i = 0; i < F.rows(); i++){
		for (unsigned j = 0; j < F.cols(); j ++){
			vertices_mesh.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
			if(uniform.draw_wireframe){
				if (j == 0){
					vertices_lines.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
				}
				else{
					vertices_lines.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
					vertices_lines.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
				}
			}
		}
		if (uniform.draw_wireframe){
			vertices_lines.push_back(VertexAttributes(V(F(i,0),0),V(F(i,0),1),V(F(i,0),2)));
		}
		if (uniform.flat_shading){
			// computing the face normals
			Vector3f u,v;
			u = vertices_mesh[3*i].position.head(3) - vertices_mesh[3*i + 1].position.head(3); 
			v = vertices_mesh[3*i + 2].position.head(3) - vertices_mesh[3*i + 1].position.head(3);
			vertices_mesh[3*i].normal = (-u.cross(v)).normalized(); 
			vertices_mesh[3*i+1].normal = (-u.cross(v)).normalized(); 
			vertices_mesh[3*i+2].normal = (-u.cross(v)).normalized(); 
		}
		if (uniform.per_vertex_shading){
			
			vertices_mesh[3*i].normal = V_p.row(F(i,0)).normalized(); 
			vertices_mesh[3*i+1].normal = V_p.row(F(i,1)).normalized();
			vertices_mesh[3 * i + 2].normal = V_p.row(F(i, 2)).normalized();
			// not normalising the average since this is done in the vertex shader
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
	uniform.M_inv = uniform.M.inverse();
	// storing the inverse to tranform normals computed at each vertex into the canonical view volume space
	uniform.M_cam_inv = uniform.M_cam.inverse(); 
	uniform.M_orth_inv = uniform.M_orth.inverse();
	Vector4f camera_location;
	camera_location << uniform.light_source(0), uniform.light_source(1), uniform.light_source(2), 1;
	// bringing light source into the view volume frame
	uniform.light_source = (uniform.M*camera_location).head(3);

	// First triangle
	vector<VertexAttributes> vertices_1;
	vertices_1.push_back(VertexAttributes(-1,-1,0));
	vertices_1.push_back(VertexAttributes(1,-1,0));
	vertices_1.push_back(VertexAttributes(-1,1,0));

	// Second triangle
	vector<VertexAttributes> vertices_2;
	vertices_2.push_back(VertexAttributes(-1,-1,0.5));
	vertices_2.push_back(VertexAttributes(1,-1,0.5));
	vertices_2.push_back(VertexAttributes(1,1,0.5));

	if(uniform.flat_shading || uniform.per_vertex_shading){
		rasterize_triangles(program, uniform, vertices_mesh, frameBuffer);
	}
	if (uniform.draw_wireframe){
		rasterize_lines(program, uniform, vertices_lines, 1.0, frameBuffer);
	}
	
	vector<uint8_t> image;
	framebuffer_to_uint8(frameBuffer,image);
	stbi_write_png("triangle.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
	
	return 0;
}
