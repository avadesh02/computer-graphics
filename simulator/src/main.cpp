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

	// integrator class
	Integrator integrator;

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

		Vector3f Li = (uniform.light_source - transformed_vector.head(3)).normalized();
		Vector3f bisector = ((uniform.light_source - transformed_vector.head(3)) + ((uniform.camera.position).cast<float> () - transformed_vector.head(3))).normalized();
		Vector3f diffuse, specular, color;
		diffuse = uniform.diffuse_color * std::max(Li.dot(transformed_normal.head(3)), float(0.0));
		// implement specular
		specular = uniform.specular_color * pow(std::max(transformed_normal.head(3).dot(bisector), float(0.0)), uniform.specular_exponent);

		color = uniform.ambient_color + diffuse + specular;
		// rotating the object and translating around the barycenter
		
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

	
	load_scene(file_name, uniform, objects, integrator);
	compute_normals(objects, uniform);

	compute_transformation_matrices(objects,frameBuffer.cols(),frameBuffer.rows(), uniform);
	
	init_object(file_name, objects);

	// camera params to rotate in a sphere
	float r = 200;
	float theta = 0;
	float phi = 0;
	const char *fileName = "simulator.gif";
	vector<uint8_t> image;
	int delay = 2;
	GifWriter g;
	GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);
	for (float i = 0; i < integrator.T; i += 1)
	{
		// integrates the simulation by one step
		theta += 0.002; 
		phi += 0.001;
		uniform.camera.position[0] = r*sin(phi); 
		uniform.camera.position[1] = r*sin(theta)*cos(phi); 
		uniform.camera.position[2] = r*cos(theta)*cos(phi);

		uniform.camera.gaze_direction = -uniform.camera.position;
		compute_transformation_matrices(objects,frameBuffer.cols(),frameBuffer.rows(), uniform);
	
		integrator.step(objects);

		frameBuffer.setConstant(FrameBufferAttributes());
		for (unsigned i = 0; i < objects.size(); i++){
			uniform.diffuse_color = objects[i].diffuse;
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
