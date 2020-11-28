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

void load_scene(const std::string &filename, UniformAttributes& uniform, vector<MatrixXd> &V_arr, 
					vector<MatrixXi> &F_arr, vector<MatrixXf> &V_p_arr, 
					vector<vector<VertexAttributes>> &vertices_mesh_arr,
					vector<vector<VertexAttributes>> &vertices_lines_arr,
					double frameBuffer_cols, double frameBuffer_rows){
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
	uniform.camera.gaze_direction  = read_vec3d(data["camera"]["gaze_direction"]);
	uniform.camera.view_up = read_vec3d(data["camera"]["view_up"]);
	uniform.camera.field_of_view = data["camera"]["field_view"];
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
	for (const auto &entry : data["objects"]){
		MatrixXd V; MatrixXi F; MatrixXf V_p;
		vector<VertexAttributes> vertices_mesh; vector<VertexAttributes> vertices_lines;
		
		std::string filename = std::string("../data/") + entry["path"].get<std::string>();
		load_off(filename, V, F);
		compute_normals(V, F, V_p, vertices_mesh, vertices_lines, uniform);
	
		V_arr.push_back(V); F_arr.push_back(F); V_p_arr.push_back(V_p);
		vertices_mesh_arr.push_back(vertices_mesh); vertices_lines_arr.push_back(vertices_lines);
	}
	compute_transformation_matrices(V_arr,frameBuffer_cols,frameBuffer_rows, uniform);

};


void compute_normals(MatrixXd &V, MatrixXi &F, MatrixXf &V_p, 
					std::vector<VertexAttributes> &vertices_mesh, 
					std::vector<VertexAttributes> &vertices_lines, 
					UniformAttributes & uniform){

    Vector3f traingle_center;
    V_p.resize(V.rows(), V.cols());
	V_p.setZero();
	double area = 0, triangle_area;
    uniform.bary_center.setZero();
	for (unsigned i = 0; i < F.rows(); i++){
		Vector3f u, v;
		// computing bary center of object
		u = V.row((F(i, 0))).cast <float> () - V.row(F(i, 1)).cast <float> ();
		v = V.row((F(i, 2))).cast <float> () - V.row(F(i, 1)).cast <float> ();
		triangle_area = 0.5*(u.cross(v).norm());
		traingle_center = (1/3.0)*(V.row((F(i, 0))).cast<float>() + V.row(F(i, 1)).cast<float>() + V.row(F(i, 1)).cast<float>());
		uniform.bary_center.head(3) += traingle_center*triangle_area;
		area += triangle_area;

		if (uniform.per_vertex_shading)
		// computing average normal at each vertex for per vertex shading
		// normalising of the entire normal is done at line 194
		{
			V_p.row(F(i, 0)) += (u.cross(v)).normalized();
			V_p.row(F(i, 1)) += (u.cross(v)).normalized();
			V_p.row(F(i, 2)) += (u.cross(v)).normalized();
		}
	}

	uniform.bary_center.head(3) = uniform.bary_center.head(3)/area;
	uniform.bary_center[3] = 1.0;

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
			vertices_mesh[3*i].normal = (u.cross(v)).normalized(); 
			vertices_mesh[3*i+1].normal = (u.cross(v)).normalized(); 
			vertices_mesh[3*i+2].normal = (u.cross(v)).normalized(); 
		}
		if (uniform.per_vertex_shading){
			// normalizing the normals at each vertex 
			vertices_mesh[3*i].normal = V_p.row(F(i,0)).normalized(); 
			vertices_mesh[3*i+1].normal = V_p.row(F(i,1)).normalized();
			vertices_mesh[3 * i + 2].normal = V_p.row(F(i, 2)).normalized();
		}
	}
};

void compute_transformation_matrices(vector<MatrixXd> &V_arr, double frameBuffer_cols, double frameBuffer_rows, 
										UniformAttributes& uniform){
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
	Vector4f lbn_world, rtf_world;
	lbn_world << 10000, 10000, 10000, 1;
	rtf_world << -10000, -10000, -10000, 1;
	for (unsigned j = 0; j < V_arr.size(); j ++){
		for (unsigned i = 0; i < V_arr[j].cols() ; i ++){
			lbn_world[i] = std::min(double (lbn_world[i]), V_arr[j].col(i).minCoeff());
			rtf_world[i] = std::max(double (rtf_world[i]), V_arr[j].col(i).maxCoeff());	
		}
	}
	//  making the box slightly bigger than the bounding box so that 
	// the bunny does not fill up the entire space
	lbn_world(0) -= 0.1;
	lbn_world(1) -= 0.1;
	lbn_world(2) -= 0.1;
	
	rtf_world(0) += 0.1;
	rtf_world(1) += 0.1;
	rtf_world(2) += 0.1;

	// tranforming from world to camera frame
	uniform.lbn = (uniform.M_cam*lbn_world).head(3);
	uniform.rtf = (uniform.M_cam*rtf_world).head(3);
	
	// computing M_orth
	// not doing (n - f) here since, the bounded box limits are already transformed to the right axis before while
	// transforming the bounding box from the world frame to the camera frame
	if (uniform.camera.is_perspective)
	{
		// to do for perspective
		uniform.rtf(1) = std::abs(uniform.lbn(0))*tan(uniform.camera.field_of_view/2);
		uniform.rtf(0) = (1.0*frameBuffer_cols/frameBuffer_rows)*uniform.rtf(1);
		// uniform.rtf(2) = -2*uniform.lbn(2);
		uniform.P << uniform.lbn(2), 0, 0, 0,
			0, uniform.lbn(2), 0, 0,
			0, 0, uniform.lbn(2) + uniform.rtf(2), -uniform.lbn(2) * uniform.rtf(2), 
			0, 0, 1, 0;
	}
	
	uniform.M_orth << 2/(uniform.rtf(0) - uniform.lbn(0)), 0, 0, -(uniform.rtf(0) + uniform.lbn(0))/(uniform.rtf(0) - uniform.lbn(0)),
				0, 2/(uniform.rtf(1) - uniform.lbn(1)), 0, -(uniform.rtf(1) + uniform.lbn(1))/(uniform.rtf(1) - uniform.lbn(1)),
				0, 0, 2/(uniform.lbn(2) - uniform.rtf(2)), -(uniform.rtf(2) + uniform.lbn(2))/(uniform.lbn(2) - uniform.rtf(2)),
				0, 0, 0, 1;
	
	if (uniform.camera.is_perspective){
		uniform.M_orth = uniform.M_orth*uniform.P;
	}

	// M_vp is not computed as it is carried out in the rasterize triangle part
	// M_object to world is assumed to be identity 
	uniform.M = uniform.M_orth*uniform.M_cam;
	// storing the inverse to tranform normals computed at each vertex into the canonical view volume space
	Vector4f camera_location;
	camera_location << uniform.light_source(0), uniform.light_source(1), uniform.light_source(2), 1;


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

