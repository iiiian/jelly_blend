#include <Eigen/Core>
#include <array>
#include <cassert>
#include <cmath>
#include <sstream>

#include "collision_constrain.h"
#include "fixedbody.h"
#include "softbody.h"

FixedFaceCollisionConstrain::FixedFaceCollisionConstrain(const Collision &collision, const Segment &seg_vertex)
{
    size_t vertex_index;

    auto p_face_body = collision.face.pbody;
    auto p_vert_body = collision.vertex.pbody;

    assert((typeid(*p_face_body) == typeid(FixedBody)));
    assert((typeid(*p_vert_body) == typeid(SoftBody)));

    vertex_index = collision.vertex.vertex_index;
    x_start = seg_vertex.start + 3 * vertex_index;
    c1 = -collision.cpoint;
    c2 = collision.face_normal;
}

void FixedFaceCollisionConstrain::solve(Eigen::VectorXd &x)
{

    double con_val = (x(Eigen::seqN(x_start, 3)) + c1).dot(c2);

    if (con_val > 0)
    {
        return;
    }

    Eigen::Vector3d delta_x = -con_val * c2;
    x(Eigen::seqN(x_start, 3)) += delta_x;
}

std::string FixedFaceCollisionConstrain::summary()
{
    std::stringstream ss;

    ss << "-----------------FixedFaceCollisionConstrain "
          "summary-----------------\n";
    ss << "x_start = " << x_start << "\n";
    ss << "c1 = " << c1.transpose() << "\n";
    ss << "c2 = " << c2.transpose() << "\n";

    return ss.str();
}

FixedVertCollisionConstrain::FixedVertCollisionConstrain(const Collision &collision, const Segment &seg_face)
{

    auto p_face_body = collision.face.pbody;
    auto p_vert_body = collision.vertex.pbody;

    assert((typeid(*p_face_body) == typeid(SoftBody)));
    assert((typeid(*p_vert_body) == typeid(FixedBody)));

    auto p_face_soft = dynamic_cast<SoftBody *>(p_face_body);

    size_t vertex_index = collision.vertex.vertex_index;
    size_t face_index = collision.face.face_index;

    vert_coor = p_vert_body->predict_vertices.col(vertex_index);

    auto face_vert_indexes = p_face_soft->faces.col(face_index);
    for (size_t i = 0; i < 3; ++i)
    {
        face_x_start[i] = seg_face.start + 3 * face_vert_indexes(i);
    }
    for (size_t i = 0; i < 3; ++i)
    {
        face_weights[i] = 1 / p_face_soft->vertex_masses(face_vert_indexes(i));
    }
}

void FixedVertCollisionConstrain::solve(Eigen::VectorXd &x)
{

    // position of the corners of the face
    auto fp0 = x(Eigen::seqN(face_x_start[0], 3));
    auto fp1 = x(Eigen::seqN(face_x_start[1], 3));
    auto fp2 = x(Eigen::seqN(face_x_start[2], 3));

    std::array<Eigen::Vector3d, 3> delta_p;
    delta_p[0] = vert_coor - fp0; // a
    delta_p[1] = fp1 - fp0;       // b
    delta_p[2] = fp2 - fp0;       // c

    Eigen::Vector3d face_normal = delta_p[1].cross(delta_p[2]);
    double norm_length = face_normal.norm();
    double con_val = delta_p[0].dot(face_normal) / norm_length - 1e-4;
    if (con_val > 0)
    {
        return;
    }

    std::array<Eigen::Vector3d, 3> cross_cache;
    cross_cache[0] = delta_p[1].cross(delta_p[2]); // bxc
    cross_cache[1] = delta_p[2].cross(delta_p[0]); // cxa
    cross_cache[2] = delta_p[0].cross(delta_p[1]); // axb

    std::array<Eigen::Vector3d, 4> grad;
    grad[1] = -(cross_cache[0] + cross_cache[1] / norm_length / 2 + cross_cache[2] / norm_length / 2); // grad p0
    grad[2] = cross_cache[1] / norm_length / 2;                                                        // grad p1
    grad[3] = cross_cache[2] / norm_length / 2;                                                        // grad p2

    double weighted_coeff = 0;
    for (size_t i = 0; i < 3; ++i)
    {
        weighted_coeff += face_weights[i] * std::pow(grad[i + 1].norm(), 2);
    }

    x(Eigen::seqN(face_x_start[0], 3)) += -con_val * face_weights[0] / weighted_coeff * grad[1]; // p0
    x(Eigen::seqN(face_x_start[1], 3)) += -con_val * face_weights[1] / weighted_coeff * grad[2]; // p1
    x(Eigen::seqN(face_x_start[2], 3)) += -con_val * face_weights[2] / weighted_coeff * grad[3]; // p2
}

std::string FixedVertCollisionConstrain::summary()
{
    std::stringstream ss;

    ss << "-----------------FixedVertCollisionConstrain "
          "summary-----------------\n";
    ss << "vertex_coor = " << vert_coor << "\n";
    ss << "face_x_start =";
    for (size_t face_start : face_x_start)
    {
        ss << " " << face_start;
    }
    ss << "\n";
    ss << "face vert weights =";
    for (auto mass : face_weights)
    {
        ss << " " << mass;
    }
    ss << "\n";

    return ss.str();
}

DynamicCollisionConstrain::DynamicCollisionConstrain(const Collision &collision, const Segment &seg_vertex,
                                                     const Segment &seg_face)
{

    auto p_face_body = collision.face.pbody;
    auto p_vert_body = collision.vertex.pbody;

    assert((typeid(*p_face_body) == typeid(SoftBody)));
    assert((typeid(*p_vert_body) == typeid(SoftBody)));

    auto p_face_soft = dynamic_cast<SoftBody *>(p_face_body);
    auto p_vert_soft = dynamic_cast<SoftBody *>(p_vert_body);

    size_t vertex_index = collision.vertex.vertex_index;
    size_t face_index = collision.face.face_index;

    auto face_vert_indexes = p_face_soft->faces.col(face_index);

    vertex_x_start = seg_vertex.start + 3 * vertex_index;
    for (size_t i = 0; i < 3; ++i)
    {
        face_x_start[i] = seg_face.start + 3 * face_vert_indexes(i);
    }

    vertex_weight = 1 / p_vert_soft->vertex_masses(vertex_index);
    for (size_t i = 0; i < 3; ++i)
    {
        face_weights[i] = 1 / p_face_soft->vertex_masses(face_vert_indexes(i));
    }
}

void DynamicCollisionConstrain::solve(Eigen::VectorXd &x)
{

    Eigen::Matrix<double, 3, 4> p;

    // position of the vertex
    p.col(0) = x(Eigen::seqN(vertex_x_start, 3));

    // position of the corners of the face
    p.col(1) = x(Eigen::seqN(face_x_start[0], 3));
    p.col(2) = x(Eigen::seqN(face_x_start[1], 3));
    p.col(3) = x(Eigen::seqN(face_x_start[2], 3));

    Eigen::Matrix3d r;
    r.col(0) = p.col(0) - p.col(1);
    r.col(1) = p.col(2) - p.col(1);
    r.col(2) = p.col(3) - p.col(1);

    double con_val = r.col(0).dot(r.col(1).cross(r.col(2)));
    if (con_val > 0)
    {
        return;
    }

    Eigen::Matrix<double, 3, 4> grad;
    grad.col(0) = r.col(1).cross(r.col(2));
    grad.col(1) = r.col(2).cross(r.col(1)) + r.col(0).cross(r.col(2)) + r.col(1).cross(r.col(0));
    grad.col(2) = r.col(2).cross(r.col(0));
    grad.col(3) = r.col(0).cross(r.col(1));

    double weighted_coeff = vertex_weight * grad.col(0).dot(grad.col(0));
    for (size_t i = 0; i < 3; ++i)
    {
        weighted_coeff += face_weights[i] * grad.col(i).dot(grad.col(i));
    }

    x(Eigen::seqN(vertex_x_start, 3)) += -con_val * vertex_weight / weighted_coeff * grad.col(0);
    x(Eigen::seqN(face_x_start[0], 3)) += -con_val * face_weights[0] / weighted_coeff * grad.col(1);
    x(Eigen::seqN(face_x_start[1], 3)) += -con_val * face_weights[1] / weighted_coeff * grad.col(2);
    x(Eigen::seqN(face_x_start[2], 3)) += -con_val * face_weights[2] / weighted_coeff * grad.col(3);
}

// void DynamicCollisionConstrain::solve(Eigen::VectorXd &x)
// {

//     // position of the vertex
//     auto p = x(Eigen::seqN(vertex_x_start, 3));

//     // position of the corners of the face
//     auto fp0 = x(Eigen::seqN(face_x_start[0], 3));
//     auto fp1 = x(Eigen::seqN(face_x_start[1], 3));
//     auto fp2 = x(Eigen::seqN(face_x_start[2], 3));

//     std::array<Eigen::Vector3d, 3> delta_p;
//     delta_p[0] = p - fp0;   // a
//     delta_p[1] = fp1 - fp0; // b
//     delta_p[2] = fp2 - fp0; // c

//     Eigen::Vector3d face_normal = delta_p[1].cross(delta_p[2]);
//     double con_val = delta_p[0].dot(face_normal);
//     if (con_val > 0)
//     {
//         return;
//     }

//     std::array<Eigen::Vector3d, 3> cross_cache;
//     cross_cache[0] = delta_p[1].cross(delta_p[2]); // bxc
//     cross_cache[1] = delta_p[2].cross(delta_p[0]); // cxa
//     cross_cache[2] = delta_p[0].cross(delta_p[1]); // axb

//     std::array<Eigen::Vector3d, 4> grad;
//     grad[0] = cross_cache[0];                                      // grad p
//     grad[1] = -(cross_cache[0] + cross_cache[1] + cross_cache[2]); // grad p0
//     grad[2] = cross_cache[1];                                      // grad p1
//     grad[3] = cross_cache[2];                                      // grad p2

//     double weighted_coeff = vertex_weight * grad[0].dot(grad[0]);
//     for (size_t i = 0; i < 3; ++i)
//     {
//         weighted_coeff += face_weights[i] * grad[i].dot(grad[i]);
//     }

//     x(Eigen::seqN(vertex_x_start, 3)) += -con_val * vertex_weight / weighted_coeff * grad[0];    // p
//     x(Eigen::seqN(face_x_start[0], 3)) += -con_val * face_weights[0] / weighted_coeff * grad[1]; // p0
//     x(Eigen::seqN(face_x_start[1], 3)) += -con_val * face_weights[1] / weighted_coeff * grad[2]; // p1
//     x(Eigen::seqN(face_x_start[2], 3)) += -con_val * face_weights[2] / weighted_coeff * grad[3]; // p2
// }

std::string DynamicCollisionConstrain::summary()
{
    std::stringstream ss;

    ss << "-----------------DynamicCollisionConstrain "
          "summary-----------------\n";
    ss << "vertex_x_start = " << vertex_x_start << "\n";
    ss << "vertex weight = " << vertex_weight << "\n";
    ss << "face_x_start =";
    for (size_t face_start : face_x_start)
    {
        ss << " " << face_start;
    }
    ss << "\n";
    ss << "face vert weights =";
    for (auto mass : face_weights)
    {
        ss << " " << mass;
    }
    ss << "\n";

    return ss.str();
}