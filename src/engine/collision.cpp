#include <Eigen/Core>
#include <sstream>

#include "collision.h"

Collision::Collision(Vertex &vertex, Face &face, Eigen::Vector3d &face_normal, Eigen::Vector3d &collision_point)
    : vertex(vertex), face(face), face_normal(face_normal), cpoint(collision_point)
{
}

std::string Collision::summary()
{
    std::stringstream ss;

    ss << "-----------------Collision summary-----------------\n";

    auto p_vert_body = vertex.pbody;
    auto p_face_body = face.pbody;
    Eigen::Vector3d initial_vertex = p_vert_body->vertices.col(vertex.vertex_index);
    Eigen::Vector3d predict_vertex = p_vert_body->predict_vertices.col(vertex.vertex_index);

    ss << "vertex index = " << vertex.vertex_index << "\n";
    ss << "face index = " << p_face_body->faces.col(face.face_index).transpose() << "\n";
    ss << "initial vertex location: " << initial_vertex.transpose() << "\n";
    ss << "collision point: " << cpoint.transpose() << "\n";
    ss << "predict vertex location: " << predict_vertex.transpose() << "\n";
    ss << "face normal: " << face_normal.transpose() << "\n";

    return ss.str();
}