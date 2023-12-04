#include <Eigen/Core>
#include <sstream>

#include "collision.h"
#include "eigen_alias.h"
#include "jb_exception.h"

Collision::Collision(const Vertex vertex, const Face face, Eigen::Vector3d face_normal, Eigen::Vector3d collision_point)
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

FaceRunTimeInfo CollisionDetector::load_face_runtime_info(const Body &body, size_t face_index)
{
    FaceRunTimeInfo info;

    info.pbody = &body;
    info.face_index = face_index;
    for (size_t i = 0; i < 3; ++i)
    {
        size_t vert_index = body.faces(i, face_index);
        info.vert_indexes[i] = vert_index;
        info.coor.col(i) = body.vertices.col(vert_index);
        info.predict_coor.col(i) = body.predict_vertices.col(vert_index);
    }

    return info;
}

void CollisionDetector::spatial_map_housekeeping()
{
    size_t mem_occupation = vert_traj_num * sizeof(VertexTrajectory) / 1024 / 1024;
    if (mem_occupation > spatial_map_mem_limit)
    {
        reset_spatial_map();
    }
}

void CollisionDetector::calcualte_spatial_cell_size(const std::vector<const SoftBody *> &p_softbodies,
                                                    const std::vector<const FixedBody *> &p_fixedbodies,
                                                    double time_delta)
{
    size_t total_edge_num = 0;
    double edge_spatial_cell_size = 0;
    double dx_spatial_cell_size = 0;

    auto get_max_dx = [time_delta](const VERTICES &velocity) {
        VERTICES dx = velocity * time_delta;
        return dx.colwise().norm().maxCoeff();
    };

    for (auto p_soft : p_softbodies)
    {
        edge_spatial_cell_size += p_soft->get_predict_edge_length_sum();
        total_edge_num += p_soft->surface_edge_num;

        double max_dx = get_max_dx(p_soft->velocity(Eigen::all, Eigen::seqN(0, p_soft->surface_vertex_num)));
        if (max_dx > dx_spatial_cell_size)
        {
            dx_spatial_cell_size = max_dx;
        }
    }

    for (auto p_fixed : p_fixedbodies)
    {
        edge_spatial_cell_size += p_fixed->get_predict_edge_length_sum();
        total_edge_num += p_fixed->edge_num;

        double max_dx = get_max_dx(p_fixed->velocity);
        if (max_dx > dx_spatial_cell_size)
        {
            dx_spatial_cell_size = max_dx;
        }
    }

    edge_spatial_cell_size /= total_edge_num;

    spatial_cell_size = (edge_spatial_cell_size > dx_spatial_cell_size) ? edge_spatial_cell_size : dx_spatial_cell_size;
}

void CollisionDetector::spatial_map_insert(size_t hash, double time, const VertexTrajectory &vert_traj)
{
    auto pbody = vert_traj.vertex.pbody;
    size_t delta_vert_traj = 0;
    SpatialMapElement &ele = spatial_map[hash];

    if (ele.time != time)
    {
        ele.time = time;
        delta_vert_traj -= ele.vert_trajectories.size();
        ele.vert_trajectories.clear();
        ele.is_vert_uni = true;
        ele.pvert_first = pbody;
    }

    if (ele.is_vert_uni)
    {
        ele.is_vert_uni = (ele.pvert_first == pbody);
    }

    ele.vert_trajectories.push_back(vert_traj);
    delta_vert_traj++;
    vert_traj_num += delta_vert_traj;
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

void CollisionDetector::cal_3x3_minmax(const Eigen::Matrix3d &coor, Eigen::Vector3d &min, Eigen::Vector3d &max)
{
    min = coor.col(0);
    max = coor.col(0);

    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 1; j < 3; ++j)
        {
            if (min[i] > coor(i, j))
            {
                min[i] = coor(i, j);
            }
            if (max[i] < coor(i, j))
            {
                max[i] = coor(i, j);
            }
        }
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

void CollisionDetector::hash_soft_to_spatial_map(const SoftBody &soft, double time)
{
    auto pbody = dynamic_cast<const Body *>(&soft);

    for (size_t vert_idx = 0; vert_idx < soft.surface_vertex_num; ++vert_idx)
    {
        Vertex vert(pbody, vert_idx);
        VertexTrajectory vert_traj(vert, soft.vertices.col(vert_idx), soft.predict_vertices.col(vert_idx));

        size_t hash = point_to_hash(soft.vertices.col(vert_idx));
        size_t predict_hash = point_to_hash(soft.predict_vertices.col(vert_idx));

        spatial_map_insert(hash, time, vert_traj);
        spatial_map_insert(predict_hash, time, vert_traj);
    }
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

bool CollisionDetector::is_traj_penetration(const Eigen::Vector3d &predict_bary, const Eigen::Vector3d &bary)
{
    // check if predict_bary and bary is at different sides of the face
    if (predict_bary[2] * bary[2] > 1e-6)
    {
        return false;
    }

    Eigen::Vector3d delta_bary = predict_bary - bary;
    double coeff = -bary[2] / delta_bary[2];
    Eigen::Vector3d cbary = bary + coeff * delta_bary;

    // check if collision point is inside the face
    if (cbary[0] >= 0 && cbary[1] >= 0 && (cbary[0] + cbary[1]) <= 1)
    {
        return true;
    }

    return false;
}

void CollisionDetector::detect_face_soft_vert_colli(const VertexTrajectory &vert_traj, const FaceRunTimeInfo &face_info)
{
    const Body *p_vert_body = vert_traj.vertex.pbody;
    assert((typeid(*p_vert_body) == typeid(SoftBody)));

    auto p_vert_soft = dynamic_cast<const SoftBody *>(p_vert_body);
    const Eigen::Vector3d &vcoor = vert_traj.coor;
    const Eigen::Vector3d &vpredict_coor = vert_traj.predict_coor;

    // a. softbody (no self collide) don't collide with itself
    // b. softbody (self collide) don't collide with adjacent face
    if (p_vert_body == face_info.pbody)
    {
        // a
        if (!p_vert_soft->detect_self_collision)
        {
            return;
        }

        // b
        for (size_t fvert_idx : face_info.vert_indexes)
        {
            if (vert_traj.vertex.vertex_index == fvert_idx)
            {
                return;
            }
        }
    }

    Eigen::Vector3d normal, projection;
    Eigen::Vector3d predict_bary = cal_face_bary(vpredict_coor, face_info.predict_coor, normal, projection);

    // check if vertex is at the back side of the face
    if (predict_bary[2] > 0)
    {
        return;
    }

    Eigen::Vector3d bary = cal_face_bary(vcoor, face_info.coor);

    // passive collision for old vertex position
    // this is to work around penetration caused by
    // dynamic collision constrain in the last frame
    bool is_bary_in = (bary[0] >= 0 && bary[1] >= 0 && (bary[0] + bary[1] <= 1));
    if (std::abs(bary[2]) < passive_collision_distance && is_bary_in)
    {
        Face face(face_info.pbody, face_info.face_index);
        collisions.emplace_back(vert_traj.vertex, face, normal, projection);
        return;
    }

    // trajectory penetration
    if (is_traj_penetration(predict_bary, bary))
    {
        Face face(face_info.pbody, face_info.face_index);
        collisions.emplace_back(vert_traj.vertex, face, normal, projection);
        return;
    }
};

void CollisionDetector::detect_face_collisions(const Body &body, size_t face_index, double time)
{
    auto f_info = load_face_runtime_info(body, face_index);

    // detect blow up
    Eigen::Vector3d fmin, fmax, fpredict_min, fpredict_max;
    cal_3x3_minmax(f_info.coor, fmin, fmax);
    cal_3x3_minmax(f_info.predict_coor, fpredict_min, fpredict_max);

    double f_minmax_d = (fmax - fmin).norm();
    double fpredict_minmax_d = (fpredict_max - fpredict_min).norm();
    if (fpredict_minmax_d / f_minmax_d > (double)blow_up_limit)
    {
        throw SimBlowUp();
    }

    std::vector<size_t> hashes = minmax_to_hash(fmin, fmax);

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
            auto p_face_soft = dynamic_cast<const SoftBody *>(&body);
            if ((!p_face_soft->detect_self_collision) && ele.is_vert_uni && ele.pvert_first == &body)
            {
                continue;
            }
        }

        for (VertexTrajectory &vert_traj : ele.vert_trajectories)
        {
            detect_face_soft_vert_colli(vert_traj, f_info);
        }
    }
}

void CollisionDetector::detect_body_collisions(const Body &body, double time)
{
    for (size_t face_idx = 0; face_idx < body.face_num; face_idx++)
    {
        detect_face_collisions(body, face_idx, time);
    }
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

void CollisionDetector::clear_spatial_map()
{
    spatial_map.clear();
}

std::vector<Collision> CollisionDetector::detect_collisions(const std::vector<const SoftBody *> &p_softbodies,
                                                            const std::vector<const FixedBody *> &p_fixedbodies,
                                                            double time, double time_delta)
{
    // update spatial map size if necessary
    size_t vert_num = 0;
    for (auto p_soft : p_softbodies)
    {
        vert_num += spatial_map_size_multiplier * p_soft->surface_vertex_num;
    }
    for (auto p_fixed : p_fixedbodies)
    {
        vert_num += spatial_map_size_multiplier * p_fixed->vertex_num;
    }
    auto get_digits = [](size_t x) { return x > 0 ? (size_t)std::log10((double)x) + 1 : 1; };
    spatial_map_size = std::pow(10, get_digits(vert_num)) - 1;
    if (spatial_map.size() != spatial_map_size)
    {
        spatial_map.resize(spatial_map_size);
    }

    spatial_map_housekeeping();

    // update spatial cell size
    if (manual_spatial_cell_size)
    {
        spatial_cell_size = manual_spatial_cell_size.value();
    }
    else
    {
        calcualte_spatial_cell_size(p_softbodies, p_fixedbodies, time_delta);
    }

    // update passive collsion distance
    if (manual_passive_collision_distance)
    {
        passive_collision_distance = manual_passive_collision_distance.value();
    }
    else
    {
        passive_collision_distance = spatial_cell_size / 5;
    }

    // hash all vertices to spatial map
    for (auto sp_soft : p_softbodies)
    {
        hash_soft_to_spatial_map(*sp_soft, time);
    }

    // iterate through all faces of softbodies to find penetrations
    collisions.clear();
    for (auto p_fixed : p_fixedbodies)
    {
        detect_body_collisions(*p_fixed, time);
    }
    for (auto p_soft : p_softbodies)
    {
        detect_body_collisions(*p_soft, time);
    }

    return collisions;
}

std::string CollisionDetector::summary()
{
    std::stringstream ss;

    ss << "-----------------Collision Detector summary-----------------\n";
    ss << "spatial_map_size_multiplier = " << spatial_map_size_multiplier << "\n";
    ss << "spatial_map_mem_limit = " << spatial_map_mem_limit << "\n";

    if (manual_spatial_cell_size)
    {
        ss << "manual_spatial_cell_size = " << manual_spatial_cell_size.value() << "\n";
    }
    else
    {
        ss << "manual_spatial_cell_size = false\n";
    }

    if (manual_passive_collision_distance)
    {
        ss << "manual_passive_collision_distance = " << manual_passive_collision_distance.value() << "\n";
    }
    else
    {
        ss << "manual_passive_collision_distance = false\n";
    }

    ss << "blow_up_limit = " << blow_up_limit << "\n";

    return ss.str();
}