// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

// Utilities for the Assignment
#include <gif.h>
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

#include "mesh_loader.h"


using namespace std;
using namespace Eigen;

int main() 
{

	// The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(500,500);

	// Global Constants (empty in this example)
	UniformAttributes uniform;

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
		Vector4f transformed_normal, transformed_vector;

		for (unsigned i = 0; i < 3; i++)
		{
			transformed_vector[i] = va.position[i];
			transformed_normal[i] = va.normal[i];
		}
		transformed_vector[3] = 1;
		transformed_normal[3] = 1;
		transformed_vector = uniform.view*uniform.bc_rot_tran * transformed_vector;

		transformed_normal = (uniform.bc_rot_tran.inverse()).transpose()* transformed_normal;

		Vector3f Li = (uniform.light_source - transformed_vector.head(3)).normalized();
		Vector3f bisector = ((uniform.light_source - transformed_vector.head(3)) - ((uniform.camera.position).cast<float> () - transformed_vector.head(3))).normalized();
		Vector3f diffuse, specular, color;
		diffuse = uniform.diffuse_color * std::max(Li.dot(transformed_normal.head(3)), float(0.0));
		// implement specular
		specular = uniform.specular_color * pow(std::max(transformed_normal.head(3).dot(bisector), float(0.0)), uniform.specular_exponent);

		color = uniform.ambient_color + diffuse + specular;

		// rotating the object and translating around the barycenter
		// cout << transformed_vector[3] << '\n';
		
		transformed_vector = uniform.M*transformed_vector;
		// transforming normal at the vertex into the canonical view volume space
		//replacing 4th co-ordinate with alpha
		VertexAttributes out(transformed_vector[0],transformed_vector[1],transformed_vector[2], transformed_vector[3]);
		out.normal = transformed_normal.head(3);
		out.color.head(3) = color;
		out.color[3] = uniform.color(3);
		return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{

		FragmentAttributes out(va.color(0),va.color(1),va.color(2),uniform.color(3));

		out.depth = va.position[2];
		return out;
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		// implementation of the z buffer
		if (fa.depth < previous.depth){
			FrameBufferAttributes out(fa.color[0]*255, fa.color[1]*255, fa.color[2]*255, fa.color[3]*255);
			out.depth = fa.depth;
			return out;
		}
		else{
			return previous;
		}
	};

	// initialising camera attributes before rendering mesh
	std::string file_name = "../data/scene.json" ;
	vector<Object> objects;

	
	load_scene(file_name, uniform, objects);
	compute_normals(objects, uniform);
	compute_transformation_matrices(objects,frameBuffer.cols(),frameBuffer.rows(), uniform);

	objects[0].resize_object(0.2, 0.2, 0.2);
	objects[0].translate_object(0.0, 0.6, 0);

	objects[1].resize_object(2.0, 0.1, 1.0);
	objects[1].translate_object(0.0, -0.3, 0);
	

	if (uniform.render_gif){
		MatrixXf trans = MatrixXf::Identity(4, 4);
		MatrixXf trans_minus = MatrixXf::Identity(4, 4);
		MatrixXf rot = MatrixXf::Identity(4, 4);
		Vector3f translate;
		translate.setZero();
		const char *fileName = "simulator.gif";
		vector<uint8_t> image;
		int delay = 25;
		GifWriter g;
		GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

		for (float i = 0; i < 1; i += 0.03)
		{
			double theta = (360*i / 180.0) * M_PI;
			trans.col(3) = uniform.bary_center;
			trans_minus.col(3) = -uniform.bary_center;
			trans_minus(3,3) = 1.0;
			rot(0, 0) = cos(theta);
			rot(0, 2) = sin(theta);
			rot(2, 2) = cos(theta);
			rot(2, 0) = -sin(theta);

			uniform.bc_rot_tran = trans * rot * trans_minus;
			// translates object after rotation

			trans.col(3).head(3) = translate;
			uniform.bc_rot_tran = trans * uniform.bc_rot_tran;

			frameBuffer.setConstant(FrameBufferAttributes());
			for (unsigned i = 0; i < objects.size(); i++){
				if (uniform.flat_shading || uniform.per_vertex_shading)
				{
					rasterize_triangles(program, uniform, objects[i].vertices_mesh, frameBuffer);
				}
				if (uniform.draw_wireframe)
				{
					rasterize_lines(program, uniform, objects[i].vertices_lines, 1.0, frameBuffer);
				}

			}
			
			framebuffer_to_uint8(frameBuffer, image);
			GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
		}

		GifEnd(&g);
		return 0;
	}
	else{
		uniform.bc_rot_tran = MatrixXf::Identity(4, 4);
		// uniform.bc_rot_tran(1,1)= 0.2;
		// uniform.bc_rot_tran(3,3) = 1;
		for (unsigned i = 0; i < objects.size(); i++){
			if (uniform.flat_shading || uniform.per_vertex_shading)
			{
				rasterize_triangles(program, uniform, objects[i].vertices_mesh, frameBuffer);
			}
			if (uniform.draw_wireframe)
			{
				rasterize_lines(program, uniform, objects[i].vertices_lines, 1.0, frameBuffer);
			}

		}
			
			
	}
	
	vector<uint8_t> image;
	framebuffer_to_uint8(frameBuffer,image);
	stbi_write_png("simulator.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
	
	return 0;
}
