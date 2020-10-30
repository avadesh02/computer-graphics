// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"


// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;


bool intersection_parallelogram(Vector3d ray_origin, RowVector3d ray_direction, Vector3d pgram_origin, Vector3d u, Vector3d v, Vector3d &intersection_params) {
	// implementation of the function that checks if the ray intersects with an arbitrary the parallelogram (Answer to question 2)

	
	// u and v are the directions of the paralleogram. The rest have the same meanings as given in the original file
	// pgram_origin + a*u + b*v is the equation of the parallelogram, a ,b lie in [0,1]
	// ray is considered to have explicit form ray_origin + t*ray_direction, t >= 0

	Matrix3d A;
	Vector3d b;

	A << u[0], v[0], -ray_direction[0],
		 u[1], v[1], -ray_direction[1],
		 u[2], v[2], -ray_direction[2];

	b = ray_origin - pgram_origin;
	// solving linear equation Ax = b, where x = [a,b,t]^T
	intersection_params = A.colPivHouseholderQr().solve(b);
	// checks below are to se if the parameters satisfy th inequality constraints (t > 0, a,b lie in [0,1])
	if(intersection_params[2] >= 0 && intersection_params[0] <= 1 && intersection_params[0] >= 0 && intersection_params[1] <= 1 && intersection_params[1] >= 0){
		return true;
	} 
	else{
		return false;
	}
}

bool intersection_sphere(Vector3d ray_origin, RowVector3d ray_direction, Vector3d sphere_center, double sphere_radius, double &t){
	// computes the point of intersection for a sphere using the equations derived in the class
	// circle in implicit form and ray in explicit form

	double a = ray_direction.dot(ray_direction);
	double b = 2*ray_direction.dot(ray_origin - sphere_center);
	double c = (ray_origin - sphere_center).dot(ray_origin - sphere_center) - sphere_radius*sphere_radius;
	double t1, t2;
	// checking if solution exists
	if(b*b - 4*a*c > 0){
		if (b*b - 4*a*c == 0){
			t = -b/2*a;
		}
		else{
			// evaluating the two solutions and choosing the smaller one
			t1 = (-b - sqrt(b*b - 4*a*c))/(2*a);
			t2 = (-b + sqrt(b*b - 4*a*c))/(2*a);
			if (t1 >=0 && t2 >= 0){
				if(t1 < t2){
					t = t1;
				}
				else{
					t = t2;
				}
			}
			if (t1 >=0){
				t = t1;
			}
			else{
				t = t2;
			}
		}
		return true;
	}
	else{
		return false;
	}
}


void raytrace_sphere() {
	std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

	const std::string filename("sphere_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);

}

void raytrace_parallelogram() {
	std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

	const std::string filename("plane_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.5,-0.5,0);
	Vector3d pgram_u(1.0, 0.5, 0);
	Vector3d pgram_v(0.5, 1.0, 0);
	// This is the vector that contains [alpha, beta, t]. This is the solution to the intersection equation
	// ray_origin + t*ray_direction = pgram_origin + alpha*pgram_u + beta*pgram_v

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);
			Vector3d interesection_params;

			// Answer to question 3 (check if ray intersects with parallelogram)
			if (intersection_parallelogram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, interesection_params)) {
				//Computing the location of intersection of ray that hits the parallelogram (Answer to question 3)
				Vector3d ray_intersection;
				ray_intersection = ray_origin + interesection_params[2]*ray_direction; 
				// Computing normal at the intersection point (Answer to question 4)
				Vector3d ray_normal = (pgram_u.cross(pgram_v)).normalized();
				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;
				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);
				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_perspective() {
	std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

	const std::string filename("plane_perspective.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// This origin is used only to compute the pixel location that is analysed
	Vector3d origin(-1,1,1);
	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d camera_origin(0,0,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.8,-0.8,0);
	Vector3d pgram_u(0.5,0.2,0);
	Vector3d pgram_v(0,0.5,0.0);

	// Parameters of the sphere
	Vector3d sphere_center(0.5,-0.5,-3);
	double sphere_radius = 1.0;

	// Single light source
	const Vector3d light_position(-1,1,1);
	//screen_dist : distance of the screen from the camera in the world frame
	double screen_dist = -1.1;
			
	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// computing ray direction and origin
			Vector3d ray_origin = camera_origin;
			Vector3d pixel_location = origin + double(i)*x_displacement + double(j)*y_displacement;
			pixel_location[2] += screen_dist;
			Vector3d ray_direction = RowVector3d(0,0,0);
			ray_direction = pixel_location - ray_origin;

			Vector3d intersection_params;
			double t;
			// TODO: Check if the ray intersects with the parallelogram
			if (intersection_parallelogram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, intersection_params)) {
				// computing exact location of intersection using the solution from AX = b (please look at the paralellogram intersection
				// for more details)
				Vector3d ray_intersection(0,0,0);
				ray_intersection = ray_origin + intersection_params[2]*ray_direction; 
				// computing normal at point of intersection
				Vector3d ray_normal = (pgram_u.cross(pgram_v)).normalized();
				
				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
			if (intersection_sphere(ray_origin, ray_direction, sphere_center, sphere_radius, t)){
				Vector3d ray_intersection(0,0,0);
				// computing exact location of intersection of sphere using solved t
				ray_intersection = ray_origin + t*ray_direction; 

				Vector3d ray_normal = (ray_intersection - sphere_center).normalized();
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_shading(){
	std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

	const std::string filename("shading.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d camera_origin(0,0,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);
	// Parameters for shading
	double ambient = 0.1;
	double phong = 100000;
	double kd = 0.5;
	double ks = 1;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	// Parameters of the sphere
	Vector3d sphere_center(0.0,-0.0,-1);
	double sphere_radius = 1.0;

	//screen_dist : distance of the screen from the camera in the world frame
	double screen_dist = -1;
	
	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
	
			Vector3d ray_origin = camera_origin;
			Vector3d pixel_location = origin + double(i)*x_displacement + double(j)*y_displacement;
			pixel_location[2] += screen_dist;
			Vector3d ray_direction = RowVector3d(0,0,0);
			ray_direction = pixel_location - ray_origin;

			double t;
		
			if (intersection_sphere(ray_origin, ray_direction, sphere_center, sphere_radius, t)) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(0,0,0);
				ray_intersection = ray_origin + t*ray_direction; 

				Vector3d ray_normal = (ray_intersection - sphere_center).normalized();
				// computing the bisector for specular shading
				Vector3d bisector = ((ray_origin - ray_intersection) + (light_position - ray_intersection)).normalized();
				diffuse(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;
				specular(i,j) = pow(bisector.transpose() * ray_normal, phong);

				// Simple diffuse model
				// the parametes for all are changed above
				C(i,j) = ambient + kd*diffuse(i,j) + ks*specular(i,j);

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_shading_rgb(){
	std::cout << "Simple ray tracer, one sphere with different rgb shading" << std::endl;

	const std::string filename("rgb_shading.png");
	MatrixXd R = MatrixXd::Zero(800,800); // creating matrix to store Red color matrix after shading
	MatrixXd G = MatrixXd::Zero(800,800); // creating matrix to store Green color matrix after shading
	MatrixXd B = MatrixXd::Zero(800,800); // creating matrix to store Blue color matrix after shading

	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d camera_origin(0,0,1);
	Vector3d x_displacement(2.0/R.cols(),0,0);
	Vector3d y_displacement(0,-2.0/R.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);
	double ambient = 0.1;
	double phong = 10;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	// Parameters of the sphere
	Vector3d sphere_center(-0.5, 0.0,-1);
	double sphere_radius = 1.0;

	//screen_dist : distance of the screen from the camera in the world frame
	double screen_dist = -1;
	
	for (unsigned i=0; i < R.cols(); ++i) {
		for (unsigned j=0; j < R.rows(); ++j) {
			// Prepare the ray
	
			Vector3d ray_origin = camera_origin;
			Vector3d pixel_location = origin + double(i)*x_displacement + double(j)*y_displacement;
			pixel_location[2] += screen_dist;
			Vector3d ray_direction = RowVector3d(0,0,0);
			ray_direction = pixel_location - ray_origin;

			double t;
		
			if (intersection_sphere(ray_origin, ray_direction, sphere_center, sphere_radius, t)) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(0,0,0);
				ray_intersection = ray_origin + t*ray_direction; 

				Vector3d ray_normal = (ray_intersection - sphere_center).normalized();
				Vector3d bisector = ((ray_origin - ray_intersection) + (light_position - ray_intersection)).normalized();
				// TODO: Add shading parameter here
				diffuse(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;
				specular(i,j) = pow(bisector.transpose() * ray_normal, phong);

				// Simple diffuse model
				R(i,j) = ambient + 0.2*diffuse(i,j) + 1*specular(i,j);
				G(i,j) = ambient + 0.2*diffuse(i,j) + 1*specular(i,j);
				B(i,j) = ambient + 0.8*diffuse(i,j) + 5*specular(i,j);

				// Clamp to zero
				R(i,j) = std::max(R(i,j),0.);
				G(i,j) = std::max(G(i,j),0.);
				B(i,j) = std::max(B(i,j),0.);
				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(R,G,B,A,filename);
}


int main() {
	raytrace_sphere();
	raytrace_parallelogram();
	raytrace_perspective();
	raytrace_shading();
	raytrace_shading_rgb();

	return 0;
}
