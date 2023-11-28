#include <Eigen/Core>
#include <sstream>

#include "collision.h"
#include "jb_exception.h"

#include "fixedbody.h"
#include <unordered_set>

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

void CollisionDetector::reset_spatial_map()
{
    for (auto &ele : spatial_map)
    {
        ele.time = -1;
        ele.is_vert_uni = true;
        ele.pvert_first = nullptr;
        ele.vert_trajectories.clear();
    }
    vert_traj_num = 0;
}

void CollisionDetector::spatial_map_housekeeping()
{
    size_t mem_occupation = vert_traj_num * sizeof(VertexTrajectory) / 1024 / 1024;
    if (mem_occupation > spatial_map_mem_limit)
    {
        reset_spatial_map();
    }
}

size_t CollisionDetector::point_to_hash(const Eigen::Ref<const Eigen::Vector3d> &point)
{

    long long x = point[0] / spatial_cell_size;
    long long y = point[1] / spatial_cell_size;
    long long z = point[2] / spatial_cell_size;

    long long temp = x * 73856093 ^ y * 19349663 ^ z * 83492791;
    size_t hash = size_t(std::abs(temp)) % spatial_map_size;
    return hash;
}

void CollisionDetector::hash_soft_to_spatial_map(const SoftBody &soft, double time)
{
    if (spatial_map.size() != spatial_map_size)
    {
        spatial_map.clear();
        spatial_map.resize(spatial_map_size);
    }

    for (size_t vert_idx = 0; vert_idx < soft.surface_vertex_num; ++vert_idx)
    {
        auto p_body = dynamic_cast<Body const *>(&soft);
        Vertex vert(p_body, vert_idx);
        size_t hash = point_to_hash(soft.predict_vertices.col(vert_idx));

        SpatialMapElement &ele = spatial_map[hash];
        size_t delta_vert_traj = 0;
        if (ele.time != time)
        {
            ele.time = time;
            delta_vert_traj -= ele.vert_trajectories.size();
            ele.vert_trajectories.clear();
            ele.is_vert_uni = true;
            ele.pvert_first = p_body;
        }

        if (ele.is_vert_uni)
        {
            ele.is_vert_uni = (ele.pvert_first == p_body);
        }

        ele.vert_trajectories.emplace_back(vert, soft.vertices.col(vert_idx), soft.predict_vertices.col(vert_idx));
        delta_vert_traj++;
        vert_traj_num += delta_vert_traj;
    }
}

std::vector<size_t> CollisionDetector::minmax_to_hash(const Eigen::Vector3d &min, const Eigen::Vector3d &max)
{
    // cast double to long long
    int64_t minx = min(0) / spatial_cell_size;
    int64_t miny = min(1) / spatial_cell_size;
    int64_t minz = min(2) / spatial_cell_size;
    int64_t maxx = max(0) / spatial_cell_size;
    int64_t maxy = max(1) / spatial_cell_size;
    int64_t maxz = max(2) / spatial_cell_size;

    size_t delta = (maxx - minx + 1) * (maxy - miny + 1) * (maxz - minz + 1);
    std::vector<size_t> hashes;

    if (delta > blow_up_limit)
    {
        throw SimBlowUp();
    }

    hashes.resize(delta);

    size_t index = 0;
    for (auto i = minx; i <= maxx; ++i)
    {
        for (auto j = miny; j <= maxy; ++j)
        {
            for (auto k = minz; k <= maxz; ++k)
            {
                long long temp = i * 73856093 ^ j * 19349663 ^ k * 83492791;
                size_t hash = size_t(std::abs(temp)) % spatial_map_size;

                hashes[index] = hash;
                index++;
            }
        }
    }

    return hashes;
}

std::vector<size_t> CollisionDetector::face_coor_to_hashes(const Eigen::Matrix3d &face_coor)
{
    Eigen::Vector3d min = face_coor.col(0);
    Eigen::Vector3d max = face_coor.col(0);

    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 1; j < 3; ++j)
        {
            if (min[i] > face_coor(i, j))
            {
                min[i] = face_coor(i, j);
            }
            if (max[i] < face_coor(i, j))
            {
                max[i] = face_coor(i, j);
            }
        }
    }

    return minmax_to_hash(min, max);
}

// calculate the barycentric coordinate of the vertex
Eigen::Vector3d CollisionDetector::cal_face_bary(const Eigen::Vector3d &vert, const Eigen::Matrix3d &face)
{
    Eigen::Vector3d normal, projection;
    return cal_face_bary(vert, face, normal, projection);
}

// calculate the barycentric coordinate of the vertex
// return the face normal and the vertex projection on the face too
Eigen::Vector3d CollisionDetector::cal_face_bary(const Eigen::Vector3d &vert, const Eigen::Matrix3d &face,
                                                 Eigen::Vector3d &normal, Eigen::Vector3d &vert_projection)
{
    Eigen::Vector3d bary;

    // e1, e2 = edge of the face
    Eigen::Vector3d e1 = face.col(1) - face.col(0);
    Eigen::Vector3d e2 = face.col(2) - face.col(0);
    // normal = normal of the face
    normal = e1.cross(e2);
    double e1xe2_norm = normal.norm();
    normal /= e1xe2_norm;

    // vertex distance to the face
    double d = (vert - face.col(0)).dot(normal);
    bary[2] = d;

    // the barycentric coordinate (b1, b2) of the vertex projection
    vert_projection = vert - d * normal; // vertex projection on the face
    Eigen::Vector3d dp = vert_projection - face.col(0);
    bary[0] = dp.cross(e2).norm() / e1xe2_norm;
    bary[1] = e1.cross(dp).norm() / e1xe2_norm;

    return bary;
}

std::vector<Collision> CollisionDetector::detect_face_collisions(const Body &body, size_t face_index, double time)
{
    Eigen::Matrix3d fcoor, fpredict_coor;
    Eigen::Vector<size_t, 3> fvert_indexes;
    for (size_t i = 0; i < 3; ++i)
    {
        fvert_indexes[i] = body.faces(i, face_index);
        fcoor.col(i) = body.vertices.col(fvert_indexes[i]);
        fpredict_coor.col(i) = body.predict_vertices.col(fvert_indexes[i]);
    }

    std::vector<size_t> hashes = face_coor_to_hashes(fpredict_coor);
    std::vector<Collision> colli;

    // detect collision between face and a vertex
    auto detect_vert_colli = [&](VertexTrajectory &vert_traj) -> void {
        Body const *p_vert_body = vert_traj.vertex.pbody;
        if (typeid(*p_vert_body) != typeid(SoftBody))
        {
            return;
        }

        auto p_vert_soft = dynamic_cast<SoftBody const *>(p_vert_body);
        Eigen::Vector3d &vcoor = vert_traj.coor;
        Eigen::Vector3d &vpredict_coor = vert_traj.predict_coor;

        // softbody (no self collide) don't collide with itself
        // softbody (self collide) don't collide with adjacent face
        if (p_vert_body == &body)
        {
            if (!p_vert_soft->detect_self_collision)
            {
                return;
            }

            // check if vertex is one of the vertex of the face
            for (size_t fvert_idx : fvert_indexes)
            {
                if (vert_traj.vertex.vertex_index == fvert_idx)
                {
                    return;
                }
            }
        }

        Eigen::Vector3d normal, projection;
        Eigen::Vector3d predict_bary = cal_face_bary(vpredict_coor, fpredict_coor, normal, projection);
        // check if vertex is not at the back side of the face
        if (predict_bary[2] > 0)
        {
            return;
        }

        bool is_predict_bary_in =
            (predict_bary[0] >= 0 && predict_bary[1] >= 0 && (predict_bary[0] + predict_bary[1] <= 1));
        // passive collision
        if (predict_bary[2] > -passive_collision_distance && is_predict_bary_in)
        {
            Face face(&body, face_index);
            colli.emplace_back(vert_traj.vertex, face, normal, projection);
            return;
        }

        Eigen::Vector3d bary = cal_face_bary(vcoor, fcoor);

        // continue if there's no trjectory penetration
        // the addition of passive collision distance is to workaround
        // penetration caused by dynamic collision constrain in last frame
        if (predict_bary[2] * (bary[2] + passive_collision_distance) > 0)
        {
            return;
        }

        Eigen::Vector3d delta_bary = predict_bary - bary;
        double coeff = -bary[2] / delta_bary[2];
        Eigen::Vector3d cbary = bary + coeff * delta_bary;

        // check if collision point is inside the face
        if (cbary[0] >= 0 && cbary[1] >= 0 && (cbary[0] + cbary[1]) <= 1)
        {
            Face face(&body, face_index);
            colli.emplace_back(vert_traj.vertex, face, normal, projection);
            return;
        }
    };

    for (size_t hash : hashes)
    {

        SpatialMapElement &ele = spatial_map[hash];
        if (ele.time != time)
        {
            continue;
        }

        // softbody that does not self collide
        if (typeid(body) == typeid(SoftBody))
        {
            auto p_face_soft = dynamic_cast<SoftBody const *>(&body);
            if ((!p_face_soft->detect_self_collision) && ele.is_vert_uni && ele.pvert_first == &body)
            {
                continue;
            }
        }

        for (VertexTrajectory &vert_traj : ele.vert_trajectories)
        {
            detect_vert_colli(vert_traj);
        }
    }

    return colli;
}

void CollisionDetector::detect_body_collisions(const Body &body, double time, std::vector<Collision> &collisons)
{

    for (size_t face_idx = 0; face_idx < body.face_num; face_idx++)
    {
        auto face_colli = detect_face_collisions(body, face_idx, time);
        for (Collision &c : face_colli)
        {
            collisons.push_back(c);
        }
    }
}

std::string CollisionDetector::summary()
{
    std::stringstream ss;

    ss << "-----------------Collision Detector summary-----------------\n";
    ss << "spatial_map_mem_limit = " << spatial_map_mem_limit << "\n";
    ss << "spatial_map_size = " << spatial_map_size << "\n";
    ss << "spatial_cell_size = " << spatial_cell_size << "\n";
    ss << "passive_collision_distance = " << passive_collision_distance << "\n";
    ss << "blow_up_limit = " << blow_up_limit << "\n";

    return ss.str();
}