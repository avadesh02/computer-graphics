// This file contains functions that load the meshes and creates vertices to be rasterized with normals
#include "mesh_loader.h"

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

void compute_camera_angle(UniformAttributes &uniform){
	// updates camera matrix based on the updated camera position
	double ax, ay, az;
	uniform.camera.gaze_direction = -uniform.camera.position.head(3);
}

void load_scene(const std::string &filename, UniformAttributes& uniform, vector<Object> &objects, Integrator &integrator){
	json data;
	std::ifstream in(filename);
	in >> data;

	// Helper function to read a Vector3d from a json array
	auto read_vec3d = [] (const json &x) {
		return Vector3d(x[0], x[1], x[2]);
	};

	auto read_vec3f = [] (const json &x) {
		return Vector3f(x[0], x[1], x[2]);
	};


	uniform.camera.position = read_vec3d(data["camera"]["position"]);
	uniform.camera.gaze_direction = -uniform.camera.position;
	uniform.camera.view_up = read_vec3d(data["camera"]["view_up"]);
	uniform.camera.field_of_view = data["camera"]["field_view"];
	uniform.camera.field_of_view *= (M_PI/180.0);
	uniform.camera.is_perspective = data["camera"]["is_perspective"];
	uniform.draw_wireframe = data["camera"]["draw_wireframe"];
	uniform.flat_shading = data["camera"]["flat_shading"];
	uniform.per_vertex_shading = data["camera"]["per_vertex_shading"];

	uniform.color << 1,0,0,1;
	uniform.light_source = read_vec3f(data["lights"]["source"]);
	uniform.diffuse_color = read_vec3f(data["lights"]["diffuse"]);
	uniform.specular_color = read_vec3f(data["lights"]["specular"]);
	uniform.specular_exponent = data["lights"]["exp"];
	uniform.ambient_color = read_vec3f(data["lights"]["ambient"]);

	uniform.render_gif = data["render_gif"];

	integrator.T = data["integrator"]["T"]; //duration of simulation
	integrator.dt = data["integrator"]["dt"];
	integrator.kp = read_vec3f(data["integrator"]["kp"]);
	integrator.kd = read_vec3f(data["integrator"]["kd"]);
	integrator.g = read_vec3f(data["integrator"]["g"]);

	for (const auto &entry : data["objects"]){
		Object object;
		std::string filename = std::string("../data/") + entry["path"].get<std::string>();
		load_off(filename, object.V, object.F);
		object.mass = entry["mass"];
		object.velocity << 0, 0, 0;
		if (entry["type"] == "sphere"){
			object.is_sphere = true; // stores if the object is a sphere or a cube
		}
		else{
			object.is_sphere = false;
		}
		if (entry["fixed"]){
			object.is_fixed = true; // stores if the object can move in space (only considering free objects)
		}
		else{
			object.is_fixed = false;
		}
		object.diffuse =  read_vec3f(entry["diffuse"]);
		objects.push_back(object);
	}
};

void init_object(const std::string &filename, vector<Object> &objects){
	json data;
	std::ifstream in(filename);
	in >> data;
	auto read_vec3f = [] (const json &x) {
		return Vector3f(x[0], x[1], x[2]);
	};
	int i = 0;
	for (const auto &entry : data["objects"]){
		Vector3f scale, translate;
		scale = read_vec3f(entry["scale"]);
		objects[i].resize_object(scale(0), scale(1), scale(2));
		translate = read_vec3f(entry["translate"]);
		objects[i].translate_object(translate(0), translate(1), translate(2));
		i ++;
	}
}

void compute_normal(Object& object, UniformAttributes & uniform){

    Vector3f traingle_center;
    object.V_p.resize(object.V.rows(), object.V.cols());
	object.V_p.setZero();
	for (unsigned i = 0; i < object.F.rows(); i++){
		Vector3f u, v;
		// computing bary center of object
		u = object.V.row((object.F(i, 0))).cast <float> () - object.V.row(object.F(i, 1)).cast <float> ();
		v = object.V.row((object.F(i, 2))).cast <float> () - object.V.row(object.F(i, 1)).cast <float> ();

		if (uniform.per_vertex_shading)
		// computing average normal at each vertex for per vertex shading
		// normalising of the entire normal is done at line 194
		{
			object.V_p.row(object.F(i, 0)) += (-u.cross(v));
			object.V_p.row(object.F(i, 1)) += (-u.cross(v));
			object.V_p.row(object.F(i, 2)) += (-u.cross(v));
		}
	}

    for (unsigned i = 0; i < object.F.rows(); i++){
		for (unsigned j = 0; j < object.F.cols(); j ++){
			object.vertices_mesh.push_back(VertexAttributes(object.V(object.F(i,j),0)
			,object.V(object.F(i,j),1),object.V(object.F(i,j),2)));
		}
		if (uniform.flat_shading){
			// computing the face normals
			Vector3f u,v;
			u = object.vertices_mesh[3*i].position.head(3) - object.vertices_mesh[3*i + 1].position.head(3); 
			v = object.vertices_mesh[3*i + 2].position.head(3) - object.vertices_mesh[3*i + 1].position.head(3);
			object.vertices_mesh[3*i].normal = (-u.cross(v)).normalized(); 
			object.vertices_mesh[3*i+1].normal = (-u.cross(v)).normalized(); 
			object.vertices_mesh[3*i+2].normal = (-u.cross(v)).normalized(); 
		}
		if (uniform.per_vertex_shading){
			// normalizing the normals at each vertex 
			object.vertices_mesh[3*i].normal = object.V_p.row(object.F(i,0)).normalized(); 
			object.vertices_mesh[3*i+1].normal = object.V_p.row(object.F(i,1)).normalized();
			object.vertices_mesh[3 * i + 2].normal = object.V_p.row(object.F(i, 2)).normalized();
		}
	}
};

void compute_normals(vector<Object>& objects, 
                    UniformAttributes & uniform){
	for (unsigned i=0; i < objects.size(); i ++){
		compute_normal(objects[i], uniform);
	};
}

void compute_transformation_matrices(vector<Object> &objects, double frameBuffer_cols, double frameBuffer_rows, 
										UniformAttributes& uniform){
    // computing the transformation matrices
	// computing transformation from wold to camera
	Vector3d w, u, v;
	w = -1*uniform.camera.gaze_direction.normalized();
	u = (uniform.camera.view_up.cross(w)).normalized();
	v = w.cross(u);

	Matrix4f tmp;
	tmp <<  u[0], v[0], w[0], uniform.camera.position[0],
			u[1], v[1], w[1], uniform.camera.position[1],
			u[2], v[2], w[2], uniform.camera.position[2],
			0, 0, 0, 1;


	uniform.M_cam = tmp.inverse();

	// compututing transformation from camera view to cannoincal view volume
	Vector4f lbn_world, rtf_world;
	lbn_world << 10000, 10000, 10000, 1;
	rtf_world << -10000, -10000, -10000, 1;
	for (unsigned j = 0; j < objects.size(); j ++){
		for (unsigned i = 0; i < objects[j].V.cols() ; i ++){
			lbn_world[i] = std::min(double (lbn_world[i]), objects[j].V.col(i).minCoeff());
			rtf_world[i] = std::max(double (rtf_world[i]), objects[j].V.col(i).maxCoeff());	
		}
	}
	//  making the box slightly bigger than the bounding box so that 
	// the bunny does not fill up the entire space
	lbn_world(0) += -3.5;
	lbn_world(1) += -3.5;
	lbn_world(2) += -3.5;
	
	rtf_world(0) += 3.5;
	rtf_world(1) += 3.5;
	rtf_world(2) += 3.5;

	// tranforming from world to camera frame
	
	uniform.lbn = (uniform.M_cam*lbn_world).head(3);
	uniform.rtf = (uniform.M_cam*rtf_world).head(3);
	uniform.lbn.head(2) = lbn_world.head(2);
	uniform.rtf.head(2) = rtf_world.head(2);
	
	uniform.M_orth << 2/(uniform.rtf(0) - uniform.lbn(0)), 0, 0, -(uniform.rtf(0) + uniform.lbn(0))/(uniform.rtf(0) - uniform.lbn(0)),
				0, 2/(uniform.rtf(1) - uniform.lbn(1)), 0, -(uniform.rtf(1) + uniform.lbn(1))/(uniform.rtf(1) - uniform.lbn(1)),
				0, 0, 2/(uniform.lbn(2) - uniform.rtf(2)), -(uniform.rtf(2) + uniform.lbn(2))/(uniform.lbn(2) - uniform.rtf(2)),
				0, 0, 0, 1;
	
	
	// M_vp is not computed as it is carried out in the rasterize triangle part
	// M_object to world is assumed to be identity 
	uniform.M = uniform.M_orth*uniform.M_cam;
	// cout << uniform.M << endl;
	// Add a transformation to compensate for the aspect ratio of the framebuffer
	float aspect_ratio = float(frameBuffer_cols)/float(frameBuffer_rows);

	uniform.view <<
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1;

	// if (aspect_ratio < 1)
	// 	uniform.view(0,0) = aspect_ratio;
	// else
	// 	uniform.view(1,1) = 1/aspect_ratio;
};

