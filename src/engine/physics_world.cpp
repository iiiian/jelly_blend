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
#include "segment.h"
#include "softbody.h"

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
        SoftBody *p_vert_soft = nullptr;
        for (auto sp_soft : sp_softbodies)
        {
            if (sp_soft.get() == p_vert_body)
            {
                p_vert_soft = sp_soft.get();
            }
        }
        assert((p_vert_soft != nullptr));

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

    // detect collisions
    collisions.clear();

    std::vector<const SoftBody *> p_softbodies;
    for (auto sp_soft : sp_softbodies)
    {
        p_softbodies.push_back(sp_soft.get());
    }
    std::vector<const FixedBody *> p_fixedbodies;
    for (auto sp_fixed : sp_fixedbodies)
    {
        p_fixedbodies.push_back(sp_fixed.get());
    }

    collisions = collision_detector.detect_collisions(p_softbodies, p_fixedbodies, time);
    // if (!collisions.empty())
    // {
    //     spdlog::info("frame {}", current_frame);
    //     for (auto &c : collisions)
    //     {
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

    collision_detector.reset_spatial_map();

    if (!test_mode)
    {
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
        time += time_delta;
    }

    current_frame++;
}

void PhysicsWorld::insert_softbody_shapekey()
{
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
        // spdlog::info("simulating frame {} to {}", current_frame, current_frame + 1);
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

    // settings for collision detector
    collision_detector.spatial_map_size_multiplier = setting.spatial_map_size_multiplier;
    collision_detector.spatial_map_mem_limit = setting.spatial_map_mem_limit;

    auto set_opt = []<typename T>(std::optional<T> &opt, bool if_exist, T value) {
        if (if_exist)
        {
            opt = value;
        }
        else
        {
            opt.reset();
        }
    };

    set_opt(collision_detector.manual_spatial_cell_size, setting.manual_spatial_cell_size, setting.spatial_cell_size);
    set_opt(collision_detector.manual_passive_collision_distance, setting.manual_passive_collision_distance,
            setting.passive_collision_distance);
}

void PhysicsWorld::dump_to_file(std::string file_path)
{
    std::ofstream file_stream;
    cereal::BinaryOutputArchive oarchive(file_stream);
    file_stream.open(file_path, std::ios::binary);

    oarchive(solver_substep_num, frame_substep_num, frame_rate, position_num);
    oarchive(sp_fixedbodies, sp_softbodies);
    oarchive(collision_detector);

    // spdlog::info("dump physics world from file");
}

void PhysicsWorld::load_from_file(std::string file_path)
{
    std::ifstream file_stream;
    cereal::BinaryInputArchive iarchive(file_stream);
    file_stream.open(file_path, std::ios::binary);

    iarchive(solver_substep_num, frame_substep_num, frame_rate, position_num);
    iarchive(sp_fixedbodies, sp_softbodies);
    iarchive(collision_detector);

    reindex_softbody_positions();

    // spdlog::info("load physics world from file");
}

std::vector<SoftBodyMesh> PhysicsWorld::export_softbody_meshes()
{
    std::vector<SoftBodyMesh> ans;

    for (auto spsoft : sp_softbodies)
    {
        ans.emplace_back(*spsoft);
    }

    return ans;
}

std::string PhysicsWorld::summary()
{
    std::stringstream ss;

    ss << "-----------------Physics world summary-----------------\n";
    ss << "Analytics:\n";
    ss << "solver_substep_num = " << solver_substep_num << "\n";
    ss << "frame_substep_num = " << frame_substep_num << "\n";
    ss << "frame_rate = " << frame_rate << "\n";
    ss << "fixedbodyies num = " << sp_fixedbodies.size() << "\n";
    ss << "softbodies num = " << sp_softbodies.size() << "\n";
    ss << collision_detector.summary() << "\n";
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
