#include <Eigen/Core>
#include <cassert>
#include <fstream>
#include <spdlog/spdlog.h>
#include <sstream>
#include <vector>

#include "fixedbody.h"
#include "geometry.h"
#include "jb_exception.h"
#include "physics_world.h"
#include "python_helper.h"
#include "softbody.h"
#include "utils.h"

void PhysicsWorld::clean_spatial_map()
{
    for (auto &ele : spatial_map)
    {
        ele.time = -1;
        ele.is_face_uni = true;
        ele.pbody_first = nullptr;
        ele.faces.clear();
    }
    spatial_map_mem_occupation = 0;
}

void PhysicsWorld::update_spatial_map_cellsize()
{
    size_t total_edge_num = 0;
    spatial_cell_size = 0;

    for (auto sp_fixed : sp_fixedbodies)
    {
        sp_fixed->update_avg_predict_edge_length();
        spatial_cell_size += sp_fixed->avg_predict_edge_length * sp_fixed->edge_num;
        total_edge_num += sp_fixed->edge_num;
    }
    for (auto sp_soft : sp_softbodies)
    {
        sp_soft->update_avg_predict_edge_length();
        spatial_cell_size += sp_soft->avg_predict_edge_length * sp_soft->surface_edge_num;
        total_edge_num += sp_soft->surface_edge_num;
    }
    spatial_cell_size /= total_edge_num;
}

size_t PhysicsWorld::point_to_hash(const Eigen::Ref<const Eigen::Vector3d> &point)
{
    long long x = point[0] / spatial_cell_size;
    long long y = point[1] / spatial_cell_size;
    long long z = point[2] / spatial_cell_size;

    long long temp = x * 73856093 ^ y * 19349663 ^ z * 83492791;
    size_t hash = size_t(std::abs(temp)) % spatial_map_size;

    return hash;
}

std::vector<size_t> PhysicsWorld::minmax_to_hash(const Eigen::Vector3d &min, const Eigen::Vector3d &max)
{

    // cast double to long long
    int64_t minx = min(0) / spatial_cell_size;
    int64_t miny = min(1) / spatial_cell_size;
    int64_t minz = min(2) / spatial_cell_size;
    int64_t maxx = max(0) / spatial_cell_size;
    int64_t maxy = max(1) / spatial_cell_size;
    int64_t maxz = max(2) / spatial_cell_size;

    size_t delta = (maxx - minx + 1) * (maxy - miny + 1) * (maxz - minz + 1);
    std::vector<size_t> hash_indexs;

    if (delta * sizeof(Face) > spatial_map_mem_threshold * 1024 * 1024)
    {
        throw SIMBlowUp();
    }

    hash_indexs.resize(delta);

    size_t index = 0;
    for (auto i = minx; i <= maxx; ++i)
    {
        for (auto j = miny; j <= maxy; ++j)
        {
            for (auto k = minz; k <= maxz; ++k)
            {
                long long temp = i * 73856093 ^ j * 19349663 ^ k * 83492791;
                size_t hash = size_t(std::abs(temp)) % spatial_map_size;

                hash_indexs[index] = hash;
                index++;
            }
        }
    }

    return hash_indexs;
}

void PhysicsWorld::predict_face_to_spatial_map(Body *pbody, size_t face_index)
{

    auto vert_indexes = pbody->faces.col(face_index);

    // face vertex position
    Eigen::Matrix3d fv;
    fv.col(0) = pbody->predict_vertices.col(vert_indexes[0]);
    fv.col(1) = pbody->predict_vertices.col(vert_indexes[1]);
    fv.col(2) = pbody->predict_vertices.col(vert_indexes[2]);

    Eigen::Vector3d min = fv.col(0);
    Eigen::Vector3d max = fv.col(0);

    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            if (min[j] > fv(j, i))
            {
                min[j] = fv(j, i);
            }
            if (max[j] < fv(j, i))
            {
                max[j] = fv(j, i);
            }
        }
    }

    std::vector<size_t> hash_indexs;
    hash_indexs = minmax_to_hash(min, max);

    size_t delta_face_num = 0;
    for (auto hash : hash_indexs)
    {
        assert((hash < spatial_map.size()));

        if (spatial_map[hash].time != time)
        {
            spatial_map[hash].time = time;
            delta_face_num -= spatial_map[hash].faces.size();
            spatial_map[hash].faces.clear();
            spatial_map[hash].is_face_uni = true;
            spatial_map[hash].pbody_first = pbody;
        }
        delta_face_num += 1;

        if (spatial_map[hash].is_face_uni && !spatial_map[hash].faces.empty())
        {
            spatial_map[hash].is_face_uni = (spatial_map[hash].pbody_first == pbody);
        }

        spatial_map[hash].faces.emplace_back(pbody, face_index);
    }
    spatial_map_mem_occupation += delta_face_num * sizeof(Face);
}

// calculate the barycentric coordinate of the vertex
Eigen::Vector3d PhysicsWorld::cal_face_bary(const Eigen::Vector3d &vert, const Eigen::Matrix3d &face)
{
    Eigen::Vector3d normal, projection;
    return cal_face_bary(vert, face, normal, projection);
}

// calculate the barycentric coordinate of the vertex
// return the face normal and the vertex projection on the face too
Eigen::Vector3d PhysicsWorld::cal_face_bary(const Eigen::Vector3d &vert, const Eigen::Matrix3d &face,
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

std::optional<Collision> PhysicsWorld::detect_soft_vertex_collision(SoftBody *psoft, size_t vertex_index, size_t hash)
{

    if (spatial_map[hash].time != time)
    {
        return std::nullopt;
    }

    bool detect_self_colli = psoft->detect_self_collision;
    const Body *pfirst = spatial_map[hash].pbody_first;
    bool is_face_uni = spatial_map[hash].is_face_uni;
    // softbody that does not self collide
    if ((!detect_self_colli) && is_face_uni && pfirst == psoft)
    {
        return std::nullopt;
    }

    auto vert_new = psoft->predict_vertices.col(vertex_index);
    auto vert_old = psoft->vertices.col(vertex_index);

    // exam all potential faces
    for (auto &face : spatial_map[hash].faces)
    {
        auto p_face_body = face.pbody;

        // softbody (no self collide) dont collide with itself
        // softbody (self collide) dont collide with adjacent face
        auto face_vert_indexes = p_face_body->faces.col(face.face_index);
        if (p_face_body == psoft)
        {
            if (!detect_self_colli)
            {
                continue;
            }
            bool is_adjacent_face = false;
            for (auto f_vert_index : face_vert_indexes)
            {
                if (vertex_index == f_vert_index)
                {
                    is_adjacent_face = true;
                    break;
                }
            }
            if (is_adjacent_face)
            {
                continue;
            }
        }

        Eigen::Matrix3d fc_new; // face corners coordinates
        fc_new.col(0) = p_face_body->predict_vertices.col(face_vert_indexes(0));
        fc_new.col(1) = p_face_body->predict_vertices.col(face_vert_indexes(1));
        fc_new.col(2) = p_face_body->predict_vertices.col(face_vert_indexes(2));

        Eigen::Vector3d normal, projection;
        Eigen::Vector3d bary_new = cal_face_bary(vert_new, fc_new, normal, projection);

        // // check passive collision
        // if (std::abs(bary_new[2]) < collision_distance)
        // {
        //     Vertex collision_vert = {psoft, vertex_index};
        //     return Collision(collision_vert, face, normal, projection);
        // }

        // continue of vertex is not at the back side of the face
        if (bary_new[2] > 0)
        {
            continue;
        }

        Eigen::Matrix3d fc_old; // face corners coordinates
        fc_old.col(0) = p_face_body->vertices.col(face_vert_indexes(0));
        fc_old.col(1) = p_face_body->vertices.col(face_vert_indexes(1));
        fc_old.col(2) = p_face_body->vertices.col(face_vert_indexes(2));

        Eigen::Vector3d bary_old = cal_face_bary(vert_old, fc_old);

        if (std::abs(bary_old[2]) < collision_distance && bary_old[0] >= 0 && bary_old[1] >= 0 &&
            (bary_old[0] + bary_old[1]) <= 1)
        {
            Vertex collision_vert = {psoft, vertex_index};
            return Collision(collision_vert, face, normal, projection);
        }

        // continue if there's no trjectory penetration
        if (bary_new[2] * bary_old[2] > 0)
        {
            continue;
        }

        Eigen::Vector3d dbary = bary_new - bary_old;
        double coeff = -bary_old[2] / dbary[2];
        Eigen::Vector3d cbary = bary_old + coeff * dbary;

        // check if collision point is inside the face
        if (cbary[0] >= 0 && cbary[1] >= 0 && (cbary[0] + cbary[1]) <= 1)
        {
            Vertex collision_vert = {psoft, vertex_index};
            return Collision(collision_vert, face, normal, projection);
        }
    }

    return std::nullopt;
}

void PhysicsWorld::detect_collisions()
{

    // hash all faces to spatial map
    for (auto sp_fixed : sp_fixedbodies)
    {
        for (size_t i = 0; i < sp_fixed->face_num; ++i)
        {
            predict_face_to_spatial_map(sp_fixed.get(), i);
        }
    }
    for (auto sp_soft : sp_softbodies)
    {
        for (size_t i = 0; i < sp_soft->face_num; ++i)
        {
            predict_face_to_spatial_map(sp_soft.get(), i);
        }
    }

    Eigen::Vector<size_t, Eigen::Dynamic> hashes;

    // iterate through all the vertex of softbodies to find penetrations
    for (auto sp_soft : sp_softbodies)
    {

        hashes.resize(sp_soft->vertex_num);
        for (size_t i = 0; i < sp_soft->vertex_num; ++i)
        {
            hashes(i) = point_to_hash(sp_soft->predict_vertices.col(i));
        }

        for (size_t vertex_index = 0; vertex_index < sp_soft->surface_vertex_num; ++vertex_index)
        {
            auto opt_c = detect_soft_vertex_collision(sp_soft.get(), vertex_index, hashes(vertex_index));

            if (!opt_c)
            {
                continue;
            }
            collisions.push_back(opt_c.value());
        }
    }

    // iterate through all the vertex of fixedbodies to find penetrations
    // for (auto sp_fixed : sp_fixedbodies)
    // {

    //     hashes.resize(sp_fixed->vertex_num);
    //     for (size_t i = 0; i < sp_fixed->vertex_num; ++i)
    //     {
    //         hashes(i) = point_to_hash(sp_fixed->predict_vertices.col(i));
    //     }

    //     for (size_t vertex_index = 0; vertex_index < sp_fixed->vertex_num; ++vertex_index)
    //     {

    //         auto opt_c = detect_body_vertex_collision(sp_fixed.get(), vertex_index, hashes(vertex_index));

    //         if (opt_c)
    //         {
    //             collisions.push_back(opt_c.value());
    //         }
    //     }
    // }
}

void PhysicsWorld::generate_colli_constrains()
{
    for (auto &c : collisions)
    {
        auto p_face_body = c.face.pbody;
        auto p_vert_body = c.vertex.pbody;

        if (typeid(*p_vert_body) == typeid(SoftBody) && typeid(*p_face_body) == typeid(FixedBody))
        {

            Segment seg_vertex = softbody_positions[p_vert_body];

            SPFixedFaceCollisionConstrain sp_fixed_constrain =
                std::make_shared<FixedFaceCollisionConstrain>(c, seg_vertex);
            sp_colli_constrains.push_back(std::dynamic_pointer_cast<CollisionConstrain>(sp_fixed_constrain));

            continue;
        }

        if (typeid(*p_vert_body) == typeid(FixedBody) && typeid(*p_face_body) == typeid(SoftBody))
        {

            Segment seg_face = softbody_positions[p_face_body];
            SPFixedVertCollisionConstrain sp_dyn_constrain = std::make_shared<FixedVertCollisionConstrain>(c, seg_face);
            sp_colli_constrains.push_back(std::dynamic_pointer_cast<CollisionConstrain>(sp_dyn_constrain));

            continue;
        }

        if (typeid(*p_vert_body) == typeid(SoftBody) && typeid(*p_face_body) == typeid(SoftBody))
        {
            Segment seg_vertex = softbody_positions[p_vert_body];
            Segment seg_face = softbody_positions[p_face_body];

            SPDynamicCollisionConstrain sp_dyn_constrain =
                std::make_shared<DynamicCollisionConstrain>(c, seg_vertex, seg_face);
            sp_colli_constrains.push_back(std::dynamic_pointer_cast<CollisionConstrain>(sp_dyn_constrain));

            continue;
        }

        assert(false && "unknown collision type");
    }
}

void PhysicsWorld::predict()
{
    for (auto sp_softbody : sp_softbodies)
    {
        sp_softbody->predict(time_delta);
    }

    if (!test_mode && current_frame_substep == 0)
    {
        for (auto sp_fixed : sp_fixedbodies)
        {
            sp_fixed->update_frame_vert(current_frame + 1);
        }
    }

    for (auto sp_fixedbody : sp_fixedbodies)
    {
        double progress = (double)current_frame_substep / frame_substep_num;
        sp_fixedbody->predict(progress, test_mode);
    }
}

void PhysicsWorld::prepare_simulation_vars()
{
    if (simulation_vars.size() != position_num)
    {
        simulation_vars.resize(position_num);
    }

    for (auto sp_soft : sp_softbodies)
    {
        Segment &seg = softbody_positions[sp_soft.get()];
        simulation_vars(Eigen::seqN(seg.start, seg.size)) = sp_soft->predict_vertices.reshaped();

        for (auto sp_con : sp_soft->sp_soft_constrains)
        {
            sp_con->reset();
        }
    }
}

void PhysicsWorld::solve_predict()
{
    for (auto sp_soft : sp_softbodies)
    {
        Segment soft_segment = softbody_positions[sp_soft.get()];
        auto soft_seqn = Eigen::seqN(soft_segment.start, soft_segment.size);

        for (auto sp_con : sp_soft->sp_soft_constrains)
        {
            sp_con->solve(simulation_vars(soft_seqn), time_delta / solver_substep_num);
        }
    }

    for (auto sp_con : sp_colli_constrains)
    {
        sp_con->solve(simulation_vars);
    }
}

void PhysicsWorld::update_body_predict()
{
    for (auto sp_soft : sp_softbodies)
    {
        Segment &seg = softbody_positions[sp_soft.get()];
        auto range = Eigen::seqN(seg.start, seg.size);

        sp_soft->predict_vertices = simulation_vars(range).reshaped(3, seg.size / 3);
    }
}

void PhysicsWorld::update_body_vertices()
{
    for (auto &sp_fixed : sp_fixedbodies)
    {
        sp_fixed->vertices = sp_fixed->predict_vertices;
    }

    for (auto &sp_soft : sp_softbodies)
    {
        sp_soft->velocity = sp_soft->predict_vertices - sp_soft->vertices;
        sp_soft->velocity /= time_delta;
        sp_soft->vertices = sp_soft->predict_vertices;
    }

    // damping the vibration of the softbody
    for (auto &sp_soft : sp_softbodies)
    {
        // calculate global translational velocity
        Eigen::Vector3d gb_tra_velocity = sp_soft->velocity.rowwise().sum() / sp_soft->vertex_num;

        // calculate r for all vertices
        Eigen::Vector3d avg_position = sp_soft->vertices.rowwise().sum() / sp_soft->vertex_num;
        VERTICES r = sp_soft->vertices.colwise() - avg_position;

        // calculate global angular velocity
        Eigen::VectorXd r2 = r.array().square().colwise().sum();
        double inertia = sp_soft->vertex_masses.dot(r2);
        Eigen::Vector3d ang_momentum = {0, 0, 0};
        for (size_t i = 0; i < sp_soft->vertex_num; ++i)
        {
            ang_momentum += sp_soft->vertex_masses(i) * r.col(i).cross(sp_soft->velocity.col(i));
        }
        Eigen::Vector3d ang_velocity = ang_momentum / inertia;

        VERTICES gb_velocity(3, sp_soft->vertex_num); // global velocity
        gb_velocity.colwise() = gb_tra_velocity;
        for (size_t i = 0; i < sp_soft->vertex_num; ++i)
        {
            gb_velocity.col(i) += ang_velocity.cross(r.col(i));
        }

        // damp local velocity so that local_velocity *= sp_soft->damping
        double effective_damping = std::pow(1 - sp_soft->damping, 1.f / frame_rate * frame_substep_num);
        sp_soft->velocity *= effective_damping;
        sp_soft->velocity += (1 - effective_damping) * gb_velocity;
    }

    // apply friction
    for (auto &colli : collisions)
    {
        auto p_vert_body = colli.vertex.pbody;
        if (typeid(*p_vert_body) != typeid(SoftBody))
        {
            continue;
        }
        SoftBody *p_vert_soft = dynamic_cast<SoftBody *>(p_vert_body);

        // calculate face normal
        auto p_face_body = colli.face.pbody;
        Eigen::Vector<size_t, 3> face_vert_indexes = p_face_body->faces.col(colli.face.face_index);

        auto c0 = p_face_body->vertices.col(face_vert_indexes(0));
        auto c1 = p_face_body->vertices.col(face_vert_indexes(1));
        auto c2 = p_face_body->vertices.col(face_vert_indexes(2));

        Eigen::Vector3d normal = (c1 - c0).cross(c2 - c0);
        normal.normalize();

        double effective_friction = std::pow(1 - p_vert_soft->friction, 1.f / frame_rate * frame_substep_num);
        size_t vert_index = colli.vertex.vertex_index;
        auto v = p_vert_soft->velocity.col(vert_index);

        Eigen::Vector3d v_norm = v.dot(normal) * normal;
        v *= effective_friction;
        v += (1 - effective_friction) * v_norm;
    }
}

// reindex softbody position
void PhysicsWorld::reindex_softbody_positions()
{
    softbody_positions.clear();

    size_t index = 0;
    for (auto sp_soft : sp_softbodies)
    {
        Segment seg = {index, 3 * sp_soft->vertex_num};
        softbody_positions[sp_soft.get()] = seg;
        index += 3 * sp_soft->vertex_num;
    }
}

void PhysicsWorld::update()
{
    // update softbody/fixedbody predict location
    predict();

    // load all softbodies vertices info into positions vector
    // reset softbody constrain lambdas
    prepare_simulation_vars();

    for (size_t i = 0; i < solver_substep_num; ++i)
    {
        solve_predict();
    }

    update_body_predict();

    // spatial map house keeping
    if (spatial_map.size() != spatial_map_size)
    {
        spatial_map.clear();
        spatial_map_mem_occupation = 0;
        spatial_map.resize(spatial_map_size);
    }
    if (spatial_map_mem_occupation > spatial_map_mem_threshold * 1024 * 1024)
    {
        clean_spatial_map();
    }
    update_spatial_map_cellsize();

    // detect collisions
    collisions.clear();
    detect_collisions();
    // if (!collisions.empty())
    // {
    //     spdlog::info("frame {}", current_frame);
    //     for (auto &c : collisions)
    //     {
    //         if (c.face_normal[2] > 0.1)
    //         {
    //             continue;
    //         }
    //         std::cout << c.summary() << "\n";
    //     }
    // }

    // generate collision constrains
    sp_colli_constrains.clear();
    generate_colli_constrains();
    for (auto sp_cons : sp_colli_constrains)
    {
        sp_cons->solve(simulation_vars);
    }

    update_body_predict();
    update_body_vertices();
}

void PhysicsWorld::add_softbody(SPSoftBody sp_soft)
{
    sp_softbodies.push_back(sp_soft);
    softbody_positions[sp_soft.get()] = Segment{position_num, 3 * sp_soft->vertex_num};
    position_num += 3 * sp_soft->vertex_num;
    spatial_map_size += sp_soft->surface_vertex_num * spatial_map_size_multiplier;
}

void PhysicsWorld::add_softbody_bl(pybind11::object bl_softbody, SoftBodySetting setting)
{
    SPSoftBody sp_soft = std::make_shared<SoftBody>();
    sp_soft->load_bl_softbody_mesh(bl_softbody);
    sp_soft->load_settings(setting);
    sp_soft->init();
    add_softbody(sp_soft);
}

void PhysicsWorld::add_fixedbody(SPFixedBody pfixed)
{
    sp_fixedbodies.push_back(pfixed);
    spatial_map_size += pfixed->vertex_num * spatial_map_size_multiplier;
}

void PhysicsWorld::add_fixedbody_bl(pybind11::object bl_fixedbody)
{
    SPFixedBody sp_fixed = std::make_shared<FixedBody>(bl_fixedbody);
    add_fixedbody(sp_fixed);
}

void PhysicsWorld::prepare_simulation(int frame_start, bool test_mode)
{
    this->test_mode = test_mode;
    time_delta = 1.f / (frame_rate * frame_substep_num);
    time = frame_start * (1.f / frame_rate);
    current_frame = frame_start;

    auto get_digits = [](size_t x) { return x > 0 ? (size_t)std::log10((double)x) + 1 : 1; };
    spatial_map_size = std::pow(10, get_digits(spatial_map_size)) - 1;
    clean_spatial_map();

    if (!test_mode)
    {
        for (auto sp_soft : sp_softbodies)
        {
            sp_soft->insert_mesh_keyframe(frame_start);
        }
        for (auto sp_fixed : sp_fixedbodies)
        {
            sp_fixed->update_frame_vert(frame_start);
        }
    }
}

void PhysicsWorld::next_frame()
{

    for (current_frame_substep = 0; current_frame_substep < frame_substep_num; ++current_frame_substep)
    {
        update();
        time += 1 / frame_rate / frame_substep_num;
    }

    current_frame++;

    if (test_mode)
    {
        return;
    }

    for (auto sp_soft : sp_softbodies)
    {
        sp_soft->insert_mesh_keyframe(current_frame);
    }
}

void PhysicsWorld::simulate(int frame_start, int frame_end, bool test_mode)
{

    prepare_simulation(frame_start, test_mode);

    for (int i = frame_start; i < frame_end; ++i)
    {
        spdlog::info("simulating frame {} to {}", current_frame, current_frame + 1);
        next_frame();
    }

    if (!test_mode)
    {
        bl_frame_set(frame_start);
    }

    spdlog::info("finished simulation!");
}

void PhysicsWorld::load_setting(const PhysicsWorldSetting &setting)
{
    this->solver_substep_num = setting.solver_substep_num;
    this->frame_substep_num = setting.frame_substep_num;
    this->frame_rate = setting.frame_rate;
    this->collision_distance = setting.collision_distance;
    this->spatial_map_mem_threshold = setting.spatial_map_mem_threshold;
    this->spatial_map_size_multiplier = setting.spatial_map_size_multiplier;
}

void PhysicsWorld::dump_to_file(std::string file_path)
{
    std::ofstream file_stream;
    cereal::BinaryOutputArchive oarchive(file_stream);
    file_stream.open(file_path, std::ios::binary);

    oarchive(solver_substep_num, frame_substep_num, frame_rate, collision_distance, spatial_map_mem_threshold,
             spatial_map_size_multiplier);
    oarchive(spatial_map_size, position_num);
    oarchive(sp_fixedbodies, sp_softbodies);

    // spdlog::info("dump physics world from file");
}

void PhysicsWorld::load_from_file(std::string file_path)
{
    std::ifstream file_stream;
    cereal::BinaryInputArchive iarchive(file_stream);
    file_stream.open(file_path, std::ios::binary);

    iarchive(solver_substep_num, frame_substep_num, frame_rate, collision_distance, spatial_map_mem_threshold,
             spatial_map_size_multiplier);
    iarchive(spatial_map_size, position_num);
    iarchive(sp_fixedbodies, sp_softbodies);

    reindex_softbody_positions();

    // spdlog::info("load physics world from file");
}

std::string PhysicsWorld::summary()
{
    std::stringstream ss;

    ss << "-----------------Physics world summary-----------------\n";
    ss << "Analytics:\n";
    ss << "solver_substep_num = " << solver_substep_num << "\n";
    ss << "frame_substep_num = " << frame_substep_num << "\n";
    ss << "frame_rate = " << frame_rate << "\n";
    ss << "collision_distance = " << collision_distance << "\n";
    ss << "spatial_map_mem_threshold = " << spatial_map_mem_threshold << " MB\n";
    ss << "spatial_map_size = " << spatial_map_size << "\n";
    ss << "fixedbodyies num = " << sp_fixedbodies.size() << "\n";
    ss << "softbodies num = " << sp_softbodies.size() << "\n";
    size_t fixedbody_index = 0;
    for (auto sp_fixed : sp_fixedbodies)
    {
        ss << "\nfixedbody " << fixedbody_index << " detail:\n";
        ss << sp_fixed->summary();
        fixedbody_index++;
    }

    size_t softbody_index = 0;
    Segment seg;
    for (auto sp_soft : sp_softbodies)
    {
        ss << "\nsoftbody " << softbody_index << " detail:\n";
        seg = softbody_positions[sp_soft.get()];
        ss << "segment: start = " << seg.start << " , size = " << seg.size << "\n";
        ss << sp_soft->summary();
        softbody_index++;
    }

    return ss.str();
}

void PhysicsWorld::test_func()
{
    return;
}
