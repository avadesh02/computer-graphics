// contains functions related to manipulating an object
#include <Eigen/Geometry>
#include "object.h"

void Object::resize_object(float sx, float sy, float sz){
    Matrix4f scale;
    scale << sx, 0, 0, 0,
            0, sy, 0, 0,
            0, 0, sz, 0,
            0, 0, 0, 1;
    box.setEmpty();
    Vector4f normal;
    Matrix4f t_inv = (scale.transpose().inverse()); 
    normal << 0,0,0,1;
    for (unsigned i = 0; i < vertices_mesh.size(); i ++){
        vertices_mesh[i].position = scale*vertices_mesh[i].position;
        box.extend(vertices_mesh[i].position.head(3));
    }
};

void Object::translate_object(float px, float py, float pz){
    Matrix4f translate;
    translate << 1, 0, 0, px,
            0, 1, 0, py,
            0, 0, 1, pz,
            0, 0, 0, 1;
    box.setEmpty();
    Matrix4f t_inv = (translate.transpose().inverse());
    Vector4f normal; 
    normal << 0,0,0,1;
    for (unsigned i = 0; i < vertices_mesh.size(); i ++){
        vertices_mesh[i].position = translate*vertices_mesh[i].position;
        normal.head(3) = vertices_mesh[i].normal.head(3);
        vertices_mesh[i].normal = (t_inv*normal).head(3);
        box.extend(vertices_mesh[i].position.head(3));
    }
};


void Object::locate_center(){
    center_loc.head(3) = (box.max() + box.min())/2.0;
    center_loc[3] = 1;
    displacement << 0,0,0;

};