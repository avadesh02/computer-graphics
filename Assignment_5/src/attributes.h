#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class VertexAttributes
{
	public:
	VertexAttributes(float x = 0, float y = 0, float z = 0, float w = 1)
	{
		position << x,y,z,w;
	}

    // Interpolates the vertex attributes
    static VertexAttributes interpolate(
        const VertexAttributes& a,
        const VertexAttributes& b,
        const VertexAttributes& c,
        const float alpha, 
        const float beta, 
        const float gamma
    ) 
    {
        VertexAttributes r;
        r.position = alpha*a.position + beta*b.position + gamma*c.position;
		r.normal =  alpha*a.normal + beta*b.normal + gamma*c.normal;
		r.color = alpha * a.color + beta * b.color + gamma * c.color;

		return r;
    }

	Eigen::Vector4f position;
	Eigen::Vector3f normal;
	Eigen::Vector4f color;
};

class FragmentAttributes
{
	public:
	FragmentAttributes(float r = 0, float g = 0, float b = 0, float a = 1)
	{
		color << r,g,b,a;
	}

	Eigen::Vector4f color;
	float depth;
};

class FrameBufferAttributes
{
	public:
	FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
	{
		color << r,g,b,a;
	}

	Eigen::Matrix<uint8_t,4,1> color;
	float depth = -1000;
};

class UniformAttributes
{
	// struct camera
	struct Camera {
		bool is_perspective;
		Eigen::Vector3d position;
		Eigen::Vector3d gaze_direction;
		Eigen::Vector3d view_up;
		double field_of_view; // between 0 and PI
		double focal_length;
		double lens_radius; // for depth of fieldlo
	};
	public:
		Camera camera;
		bool draw_wireframe; // draws wireframe
		bool flat_shading;
		bool per_vertex_shading;
		bool render_gif; // renders gif
		Eigen::Matrix4f M_orth, M_cam, M_model, M, M_inv, M_orth_inv, M_cam_inv;
		Eigen::Vector3f lbn, rtf; // lower and upper limit of camera view
		Eigen::Vector4f color;
		Eigen::Vector3f light_source;
		Eigen::Matrix4f P;

		Eigen::Vector3f diffuse_color, specular_color, ambient_color;
		float specular_exponent;
		Eigen::Vector4f bary_center;
		Eigen::MatrixXf bc_rot_tran; // bary center rotate translate
};