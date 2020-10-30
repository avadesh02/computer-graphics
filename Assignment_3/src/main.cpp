////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// Eigen for matrix operations
#include <Eigen/Dense>

#include <gif.h>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"
#include "utils.h"

// JSON parser library (https://github.com/nlohmann/json)
#include "json.hpp"
using json = nlohmann::json;

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Define types & classes
////////////////////////////////////////////////////////////////////////////////

struct Ray {
	Vector3d origin;
	Vector3d direction;
	Ray() { }
	Ray(Vector3d o, Vector3d d) : origin(o), direction(d) { }
};

struct Light {
	Vector3d position;
	Vector3d intensity;
};

struct Intersection {
	Vector3d position;
	Vector3d normal;
	double ray_param;
};

struct Camera {
	bool is_perspective;
	Vector3d position;
	double field_of_view; // between 0 and PI
	double focal_length;
	double lens_radius; // for depth of field
};

struct Material {
	Vector3d ambient_color;
	Vector3d diffuse_color;
	Vector3d specular_color;
	double specular_exponent; // Also called "shininess"

	Vector3d reflection_color;
	Vector3d refraction_color;
	double refraction_index;
};

struct Object {
	Material material;
	virtual ~Object() = default; // Classes with virtual methods should have a virtual destructor!
	virtual bool intersect(const Ray &ray, Intersection &hit) = 0;
};

// We use smart pointers to hold objects as this is a virtual class
typedef std::shared_ptr<Object> ObjectPtr;

struct Sphere : public Object {
	Vector3d position;
	double radius;

	virtual ~Sphere() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Parallelogram : public Object {
	Vector3d origin;
	Vector3d u;
	Vector3d v;

	virtual ~Parallelogram() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Scene {
	Vector3d background_color;
	Vector3d ambient_light;

	Camera camera;
	std::vector<Material> materials;
	std::vector<Light> lights;
	std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////

bool Sphere::intersect(const Ray &ray, Intersection &hit) {
	// TODO:
	//
	// Compute the intersection between the ray and the sphere
	// If the ray hits the sphere, set the result of the intersection in the
	// struct 'hit'

	double a = ray.direction.dot(ray.direction);
	double b = 2*ray.direction.dot(ray.origin - position);
	double c = (ray.origin - position).dot(ray.origin - position) - radius*radius;
	double t, t1, t2;
	// checking if solution exists
	if(b*b - 4*a*c > 0){
		if (b*b - 4*a*c == 0){
			t = -b/2*a;
		}
		else{
			// evaluating the two solutions and choosing the smaller one
			t1 = (-b - sqrt(b*b - 4*a*c))/(2*a);
			t2 = (-b + sqrt(b*b - 4*a*c))/(2*a);
			if (t1 < 0 && t2 < 0){
				return false;
			}
			else{
				if (t1 >=0 && t2 >= 0){
					if(t1 < t2){
						t = t1;
					}
					else{
						t = t2;
					}
				}
				if (t1 >=0 && t2 < 0){
					t = t1;
				}
				if (t2 >=0 && t1 < 0){
					t = t2;
				}	
			}
			// updaing the information to hit variable
			hit.position = ray.origin + t*ray.direction;
			hit.normal = (hit.position - position).normalized();

			return true;
		}
	}
	else{
		return false;
	}
}


bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
	// This function checks if a ray hits a parallelogram
	Matrix3d A;
	Vector3d b;
	Vector3d intersection_params;

	A << u[0], v[0], -ray.direction[0],
		 u[1], v[1], -ray.direction[1],
		 u[2], v[2], -ray.direction[2];

	b = ray.origin - origin;
	// solving linear equation Ax = b, where x = [a,b,t]^T
	intersection_params = A.colPivHouseholderQr().solve(b);
	// checks below are to se if the parameters satisfy th inequality constraints (t > 0, a,b lie in [0,1])
	if(intersection_params[2] >= 0 && intersection_params[0] <= 1 && intersection_params[0] >= 0 && intersection_params[1] <= 1 && intersection_params[1] >= 0){
		
		hit.position = ray.origin + intersection_params[2]*ray.direction;
		hit.normal = (u.cross(v)).normalized();
		return true;
	} 
	else{		
		return false;
	}
}

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &object, const Intersection &hit, int max_bounce);
Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit);
bool is_light_visible(const Scene &scene, const Light &light, const Intersection &hit);
Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce);

// -----------------------------------------------------------------------------

Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &obj, const Intersection &hit, int max_bounce) {
	// Material for hit object
	const Material &mat = obj.material;

	// Ambient light contribution
	Vector3d ambient_color = obj.material.ambient_color.array() * scene.ambient_light.array();

	// Punctual lights contribution (direct lighting)
	Vector3d lights_color(0, 0, 0);
	for (const Light &light : scene.lights) {
		Vector3d Li = (light.position - hit.position).normalized();
		Vector3d bisector = ((light.position - hit.position) - ray.direction).normalized();
		Vector3d N = hit.normal;

		// TODO: Shoot a shadow ray to determine if the light should affect the intersection point
		if (is_light_visible(scene, light, hit)){
			Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

			// TODO: Specular contribution
			Vector3d specular = mat.specular_color * pow(std::max(N.dot(bisector), 0.0), mat.specular_exponent);

			// Attenuate lights according to the squared distance to the lights
			Vector3d D = light.position - hit.position;
			lights_color += (diffuse + specular).cwiseProduct(light.intensity) /  D.squaredNorm();
			}
		
		}

	// TODO: Compute the color of the reflected ray and add its contribution to the current point color.
	Vector3d reflection_color(0, 0, 0);
	Ray reflected_ray;
	Intersection reflected_hit;
	reflected_ray.origin = hit.position;
	// computing the reflected ray direction using the formula discussed in class
	reflected_ray.direction = ray.direction - 2*hit.normal*((hit.normal).dot(ray.direction)) ;
	// offseting the reflected ray slightly to make sure there is no collision with the same object it reflects from 
	reflected_ray.origin += 0.001*reflected_ray.direction;
	
	for (unsigned i = 0; i < max_bounce; i ++){
		// running the loop for a maximum of max_bounce
		if(Object * obj = find_nearest_object(scene, reflected_ray, reflected_hit)){
			for (const Light &light : scene.lights) {
				// computing bisector for the specular shading for the reflected ray
				Vector3d bisector = ((light.position - reflected_hit.position) - reflected_ray.direction).normalized();
				Vector3d N = reflected_hit.normal;

				// TODO: Shoot a shadow ray to determine if the light should affect the intersection point
				if (is_light_visible(scene, light, hit)){
					// had to reduce the shinning constant to make the reflection of the spheres more prominent
					// otherwise the resulting figure did not match the figure shown in the github repp
					Vector3d specular = mat.reflection_color * pow(std::max(N.dot(bisector), 0.0), 5);

					// Attenuate lights according to the squared distance to the lights
					Vector3d D = light.position - reflected_hit.position;
					reflection_color += (specular).cwiseProduct(light.intensity) /  D.squaredNorm();
				}
			}
			reflected_ray.origin = reflected_hit.position;
			// computing the reflected ray direction using the formula discussed in class
			reflected_ray.direction = reflected_ray.direction - 2*reflected_hit.normal*((reflected_hit.normal).dot(reflected_ray.direction)) ;
			// offseting the reflected ray slightly to make sure there is no collision with the same object it reflects from 
			reflected_ray.origin += 0.001*reflected_ray.direction;

		}
		else{
			// if reflected ray does not intersect any object terminate loop
			break;
		}
	}

	// TODO: Compute the color of the refracted ray and add its contribution to the current point color.
	//       Make sure to check for total internal reflection before shooting a new ray.
	Vector3d refraction_color(0, 0, 0);
	Ray refracted_ray;
	Intersection refracted_hit;
	refracted_ray.origin = hit.position;
	const double ri = mat.refraction_index;
	refracted_ray.direction = ri*ray.direction + (ri*(hit.normal.normalized().dot(ray.direction.normalized())) \
									- sqrt(1 - ri*ri*(1 - pow(hit.normal.normalized().dot(ray.direction.normalized()),2))))*hit.normal;
	refracted_ray.origin += 0.001*refracted_ray.direction;


	for (unsigned i = 0; i < max_bounce; i ++){
		if(Object * obj = find_nearest_object(scene, refracted_ray, refracted_hit)){
			for (const Light &light : scene.lights) {
				Vector3d bisector = ((light.position - refracted_hit.position) - refracted_ray.direction).normalized();
				Vector3d N = refracted_hit.normal;

				// checking for total internal reflection
				if (refracted_ray.direction.dot(N) < 0){
					break;
				}

				// TODO: Shoot a shadow ray to determine if the light should affect the intersection point
				if (is_light_visible(scene, light, hit)){
					// had to reduce the shinning constant to make the reflection of the spheres more prominent
					Vector3d specular = mat.refraction_color * pow(std::max(N.dot(bisector), 0.0), 5);

					// Attenuate lights according to the squared distance to the lights
					Vector3d D = light.position - refracted_hit.position;
					refraction_color += (specular).cwiseProduct(light.intensity) /  D.squaredNorm();
				}
			}
			refracted_ray.origin = refracted_hit.position;
			refracted_ray.direction = ri*refracted_ray.direction + (ri*(refracted_hit.normal.normalized().dot(refracted_ray.direction.normalized())) \
										 - sqrt(1 - ri*ri*(1 - pow(refracted_hit.normal.normalized().dot(refracted_ray.direction.normalized()),2))))*refracted_hit.normal;
			refracted_ray.origin += 0.001*refracted_ray.direction;

		}
		else{
			break;
		}
	}



	// Rendering equation
	Vector3d C = ambient_color + lights_color + reflection_color + refraction_color;

	return C;
}

// -----------------------------------------------------------------------------

Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
	int closest_index = -1;
	// Find the object in the scene that intersects the ray first
	// The function must return 'nullptr' if no object is hit, otherwise it must
	// return a pointer to the hit object, and set the parameters of the argument
	// 'hit' to their expected values.
	Intersection current_hit;
	for (unsigned i = 0 ; i < scene.objects.size(); i ++){
		if (scene.objects[i].get()->intersect(ray, current_hit)){
			if (closest_index >= 0){
				if ((ray.origin - current_hit.position).squaredNorm() < (ray.origin - closest_hit.position).squaredNorm()){
					closest_hit = current_hit;
					closest_index = i;
				}
			}
			else{
				closest_index = i;
				closest_hit = current_hit;
			}
		}

	}
	
	if (closest_index < 0) {
		// Return a NULL pointer
		return nullptr;
	} else {
		// Return a pointer to the hit object. Don't forget to set 'closest_hit' accordingly!
		// std::cout << closest_index << '\n';
		return scene.objects[closest_index].get();
	}
}

bool is_light_visible(const Scene &scene, const Light &light, const Intersection &hit) {
	// removed ray as input variable as it was not useful
	// TODO: Determine if the light is visible here
	double epsillon = 0.001;
	Intersection closest_hit;
	Ray shadow_ray;
	// determining the ray parameters
	shadow_ray.origin = hit.position;
	shadow_ray.direction = (light.position - hit.position);
	shadow_ray.origin += epsillon*shadow_ray.direction;
	if (Object * obj = find_nearest_object(scene, shadow_ray, closest_hit)){
		// ensuring that the intersection of the shadow ray is before the ligth source 
		// if the object lies behind the light source, then light is still visible despite the intersection
		if ((closest_hit.position - hit.position).squaredNorm() < (light.position - hit.position).squaredNorm()){
			return false;
		}
		else{
			return true;
		}
	}
	else{
		return true;
	}
}

Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce) {
	Intersection hit;
	if (Object * obj = find_nearest_object(scene, ray, hit)) {
		// 'obj' is not null and points to the object of the scene hit by the ray
		return ray_color(scene, ray, *obj, hit, max_bounce);
	} else {
		// 'obj' is null, we must return the background color
		return scene.background_color;
	}
}

////////////////////////////////////////////////////////////////////////////////

void render_scene(const Scene &scene) {
	std::cout << "Simple ray tracer." << std::endl;

	int w = 640;
	int h = 480;
	MatrixXd R = MatrixXd::Zero(w, h);
	MatrixXd G = MatrixXd::Zero(w, h);
	MatrixXd B = MatrixXd::Zero(w, h);
	MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

	// The camera always points in the direction -z
	// The sensor grid is at a distance 'focal_length' from the camera center,
	// and covers an viewing angle given by 'field_of_view'.
	double aspect_ratio = double(w) / double(h);
	double scale_y = 2*scene.camera.focal_length*tan(scene.camera.field_of_view/2.0) ;// TODO: Stretch the pixel grid by the proper amount here
	double scale_x = aspect_ratio*scale_y; //

	// The pixel grid through which we shoot rays is at a distance 'focal_length'
	// from the sensor, and is scaled from the canonical [-1,1] in order
	// to produce the target field of view.
	Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
	Vector3d x_displacement(2.0/w*scale_x, 0, 0);
	Vector3d y_displacement(0, -2.0/h*scale_y, 0);

	// for depth of field
	// number of rays cast per pixel
	int no_rays = 5;

    const float MIN_RAND = -1.0, MAX_RAND = 1.0;
	const float range = MAX_RAND - MIN_RAND;
	float random;
	
	for (unsigned i = 0; i < w; ++i) {
		for (unsigned j = 0; j < h; ++j) {
			// TODO: Implement depth of field
			for (unsigned k = 0; k < no_rays; k ++){
					Vector3d shift = grid_origin + (i)*x_displacement + (j)*y_displacement;

					// Prepare the ray
					Ray ray;

					if (scene.camera.is_perspective) {
						// Perspective camera
						// implementing depth of field
						ray.origin = scene.camera.position;
						random = range * ((((float) rand()) / (float) RAND_MAX)) + MIN_RAND ;
						ray.origin[0] += scene.camera.lens_radius*random;
						random = range * ((((float) rand()) / (float) RAND_MAX)) + MIN_RAND ;
						ray.origin[1] += scene.camera.lens_radius*random;
	
						ray.direction = shift - ray.origin;
					} else {
						// Orthographic camera
						ray.origin = scene.camera.position + Vector3d(shift[0], shift[1], 0);
						ray.direction = Vector3d(0, 0, -1);
					}
					// maximum times a relfected or refracted ray is allowed to bounce
					int max_bounce = 10;
					Vector3d C = shoot_ray(scene, ray, max_bounce);
					R(i, j) += C(0)/no_rays;
					G(i, j) += C(1)/no_rays;
					B(i, j) += C(2)/no_rays;
					A(i, j) = 1;
				}
			}
			
	}

	// Save to png
	const std::string filename("raytrace.png");
	write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

Scene load_scene(const std::string &filename) {
	Scene scene;

	// Load json data from scene file
	json data;
	std::ifstream in(filename);
	in >> data;

	// Helper function to read a Vector3d from a json array
	auto read_vec3 = [] (const json &x) {
		return Vector3d(x[0], x[1], x[2]);
	};

	// Read scene info
	scene.background_color = read_vec3(data["Scene"]["Background"]);
	scene.ambient_light = read_vec3(data["Scene"]["Ambient"]);

	// Read camera info
	scene.camera.is_perspective = data["Camera"]["IsPerspective"];
	scene.camera.position = read_vec3(data["Camera"]["Position"]);
	scene.camera.field_of_view = data["Camera"]["FieldOfView"];
	scene.camera.focal_length = data["Camera"]["FocalLength"];
	scene.camera.lens_radius = data["Camera"]["LensRadius"];

	// Read materials
	for (const auto &entry : data["Materials"]) {
		Material mat;
		mat.ambient_color = read_vec3(entry["Ambient"]);
		mat.diffuse_color = read_vec3(entry["Diffuse"]);
		mat.specular_color = read_vec3(entry["Specular"]);
		mat.reflection_color = read_vec3(entry["Mirror"]);
		mat.refraction_color = read_vec3(entry["Refraction"]);
		mat.refraction_index = entry["RefractionIndex"];
		mat.specular_exponent = entry["Shininess"];
		scene.materials.push_back(mat);
	}

	// Read lights
	for (const auto &entry : data["Lights"]) {
		Light light;
		light.position = read_vec3(entry["Position"]);
		light.intensity = read_vec3(entry["Color"]);
		scene.lights.push_back(light);
	}

	// Read objects
	for (const auto &entry : data["Objects"]) {
		ObjectPtr object;
		if (entry["Type"] == "Sphere") {
			auto sphere = std::make_shared<Sphere>();
			sphere->position = read_vec3(entry["Position"]);
			sphere->radius = entry["Radius"];
			object = sphere;
		} else if (entry["Type"] == "Parallelogram") {
			// TODO
			auto pgram = std::make_shared<Parallelogram>();
			pgram->origin = read_vec3(entry["Origin"]);
			pgram->u = read_vec3(entry["u"]);
			pgram->v = read_vec3(entry["v"]);
			object = pgram;
		}
		object->material = scene.materials[entry["Material"]];
		scene.objects.push_back(object);
	}

	return scene;
}

////////////////////////////////////////////////////////////////////////////////




int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " scene.json" << std::endl;
		return 1;
	}
	Scene scene = load_scene(argv[1]);
	render_scene(scene);
	
	return 0;
}
