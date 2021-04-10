
#define COW
//#define CTHULHU



#include "skinning.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "skinning_loader.hpp"
#include "helper_skeleton.hpp"
#include <fstream>

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Weffc++"
#endif
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

#ifdef _WIN32
#pragma warning(disable : 4267)
#endif


using namespace vcl;


vec3 deformation_flappy_linear_speed(float w_flappy, vec3 const& speed)
{
    vec3 const deformation = - w_flappy * speed;

    return deformation;
}


vec3 deformation_squashy_linear_speed(float w_squashy, vec3 const& speed, float const speed_norm, vec3 const& p_vertex, vec3 const& center_of_mass)
{
    float const squash_factor = w_squashy * speed_norm;
    float const elongation_scaling = 1+squash_factor;
    float const squeeze_scaling = 1/std::sqrt(1+squash_factor);

    mat3 const Id = mat3::identity();
    mat3 const S = {elongation_scaling,0,0, 0,squeeze_scaling,0, 0,0,squeeze_scaling}; // Scaling
    mat3 const R = rotation_between_vector_mat3({1,0,0}, speed); // Rotation to the correct frame
    mat3 const T = R * S * transpose(R); // Deformation matrix


    vec3 const deformation = (T-Id) * (p_vertex-center_of_mass);

    return deformation;
}

vec3 deformation_flappy_rotation_speed(float w_flappy, vec3 const& p_vertex, vec3 const& p_joint,
                                       vec3 const& un_angular_speed, float vertex_speed_norm, float flappy_max_angle)
{
    vec3 const u_joint_vertex = p_vertex - p_joint;
    mat3 const Id = mat3::identity();

    vec3 const rotation_center = p_joint + dot(u_joint_vertex, un_angular_speed)*un_angular_speed;
    float rotation_angle = vertex_speed_norm * w_flappy ;

    if(rotation_angle>flappy_max_angle)
        rotation_angle = flappy_max_angle + (rotation_angle-flappy_max_angle)/vertex_speed_norm;


    mat3 const R = rotation_from_axis_angle_mat3( un_angular_speed, -rotation_angle );
    vec3 const deformation = (R-Id) * (p_vertex-rotation_center);

    return deformation;
}

vec3 deformation_squashy_rotation_speed(float squashing_power,
                                        vec3 const& p_vertex, vec3 const& p_joint,
                                        vec3 const& un_medial, vec3 const& un_angular_speed,
                                        float vertex_speed_norm, vec3 const& center_of_mass, int squash_type)
{
    mat3 const Id = mat3::identity();

    // Scaling matrix
    float const squash_factor = squashing_power * vertex_speed_norm;
    float const elongation = 1+squash_factor;
    float const squeeze = 1/(1+squash_factor);
    mat3 const S = { elongation,0,0, 0,squeeze,0, 0,0,squeeze };

    // Rotate to correct frame
    vec3 const u_elongation = cross(un_medial, un_angular_speed);
    if( norm(u_elongation)< 1e-2f)
        return {0,0,0};

    vec3 const un_elongation = normalize(u_elongation);
    vec3 const un_squeeze = cross(un_medial, un_elongation);
    mat3 const R = mat3(un_elongation, un_squeeze, un_medial);

    // Deformation matrix
    mat3 const T = R*S*transpose(R);

    // Center of scaling
    vec3 const p_medial = squash_type==0?
                p_joint + dot(p_vertex-p_joint,un_medial)*un_medial:
                center_of_mass;

    vec3 const deformation = (T-Id) * (p_vertex-p_medial);

    return deformation;
}



void scene_model::velocity_skinning(float magnitude)
{
    float const default_squashy_factor = 0.1f * magnitude;
    float const default_flappy_factor = 0.4f * magnitude;

    float const w_squashy = default_squashy_factor * squashing_power;

    // reset deformation
    deformation_per_vertex.fill({0,0,0});

    assert_vcl_no_msg(weight_flappy.size()==skinning.rest_pose.size());
    size_t const N_joint = skeleton_current.size();

    for(size_t joint=0; joint<N_joint; ++joint)
    {
        vec3 const& p_joint = skeleton_speed_per_joint.data[joint].center;
        vec3 const linear_speed = (skeleton_speed_per_joint.data[joint].linear_speed / timer_skeleton.scale + skeleton_fake_speed_per_joint.data[joint].linear_speed);
        vec3 const angular_speed = (skeleton_speed_per_joint.data[joint].angular_speed / timer_skeleton.scale + skeleton_fake_speed_per_joint.data[joint].angular_speed);
        vec3 const un_angular_speed = normalize(angular_speed);

        float const linear_speed_norm = norm(linear_speed);
        float const angular_speed_norm = norm(angular_speed);

        vec3 const& center_of_mass = position_center_of_mass(joint);//center_of_mass_per_joint[joint];
        vec3 const u_medial = center_of_mass - p_joint;
        if(norm(u_medial)<1e-4f)
            std::cout<<"small medial direction"<<std::endl;
        vec3 const un_medial = normalize(u_medial);

        buffer<int> const& vertices = vertex_depending_on_joint.data[joint];
        buffer<float> const& vertices_weights = vertex_weight_depending_on_joint.data[joint];
        size_t const N_vertex_dependency = vertices.size();





        // Linear motion
        if(linear_speed_norm>1e-3f)
        {
            for(size_t k_vertex=0; k_vertex<N_vertex_dependency; ++k_vertex)
            {
                size_t const vertex = vertices.data[k_vertex];
                float const w_skinning = vertices_weights.data[k_vertex];

                vec3 const& p_vertex = save_skinning.data[vertex];
                float const w_flappy = default_flappy_factor * flapping_power * weight_flappy.data[vertex];


                vec3 const flappy  = deformation_flappy_linear_speed(w_flappy, linear_speed);
                vec3 const squashy = deformation_squashy_linear_speed(w_squashy, linear_speed, linear_speed_norm, p_vertex, center_of_mass);

                deformation_per_vertex.data[vertex] += w_skinning * (flappy + squashy);
            }
        }

        // Rotation motion
        if(angular_speed_norm>1e-3f)
        {
            for(size_t k_vertex=0; k_vertex<N_vertex_dependency; ++k_vertex)
            {
                size_t const vertex = vertices.data[k_vertex];
                float const w_skinning = vertices_weights.data[k_vertex];

                vec3 const& p_vertex = save_skinning.data[vertex];
                float const w_flappy = default_flappy_factor * flapping_power * weight_flappy.data[vertex];

                vec3 const u_joint_vertex = p_vertex - p_joint;
                vec3 const vertex_speed = cross(angular_speed, u_joint_vertex);
                float const vertex_speed_norm = norm(vertex_speed);


                // Flappy
                vec3 const flappy  = deformation_flappy_rotation_speed(w_flappy, p_vertex, p_joint, un_angular_speed, vertex_speed_norm, gui_param.flappy_max_angle);
                vec3 const squashy = deformation_squashy_rotation_speed(w_squashy, p_vertex, p_joint, un_medial,un_angular_speed, vertex_speed_norm, center_of_mass, gui_param.squash_around);

                deformation_per_vertex.data[vertex] += w_skinning * (flappy + squashy);
            }

        }

    }



    for(size_t k=0; k<deformation_per_vertex.size(); ++k)
        skinning.deformed.position[k] += deformation_per_vertex[k];




}










void scene_model::frame_draw(std::map<std::string,GLuint>& , scene_structure& _scene, gui_structure& )
{
    //std::cout << picking.selected_joint_true << std::endl;

    timer_measurement["full"].tic();

    scene = _scene;
    timer.update();

    if(gui_param.animation)
        timer_skeleton.update();

    set_gui();

    float const t = timer.t;

    
    // Camera animation
    /*
#ifdef COW
    static bool first = true;
    if (first)
    {

        timer.t = -60;
        first = false;
        gui_param.animation = true;
        gui_param.display_skeleton_pyramid = false;
        gui_param.display_skeleton_bones = false;
        gui_param.display_skeleton_frames = false;
        gui_param.display_skeleton_joints = false;
    }
    float ut = t / 20.0f;
    //std::cout << t << std::endl;
    if (ut < 0) {
        scene.camera.orientation = rotation_from_axis_angle_mat3(vec3(1, 0, 0), 3.14f / 2.0f - 3.14 / 12.0f);
        scene.camera.translation.y = 140 ;
        scene.camera.translation.z = -1.7f;
    }
    else if (ut <= 1.0f)
    {
        scene.camera.orientation = rotation_from_axis_angle_mat3(vec3(1, 0, 0), 3.14f / 2.0f - 3.14 / 12.0f);
        scene.camera.translation.y = 140-150*std::min(ut,0.75f)*ut;
        scene.camera.translation.z = -1.7f;
    }
    else
    {
        float ut2 = ut - 1.0f;
        scene.camera.orientation = rotation_from_axis_angle_mat3(vec3(1, 0, 0), 3.14f / 2.0f - 3.14 / 12.0f - 2*3.14 / 12.0f*ut2);
        scene.camera.translation.y = 140 - 150*(0.75f) + 300*ut2*ut2 + 150 * ut2;
        
        scene.camera.translation.z = -1.7f - ut2 * ut2 * 480;
    }
    //std::cout << scene.camera.translation.y << std::endl;
#endif
*/


    if (timer_skeleton.t >= timer_skeleton.t_max)
        timer_skeleton.t = 0;


    //std::cout << "interpolate skeleton" << std::endl;
    interpolate_skeleton_at_time_with_constraints(skeleton_local_current_before_deformation,
                                                  timer_skeleton.t,
                                                  skeleton.anim,
                                                  gui_param.interpolate,
                                                  record_joint_fixed);


    //    // case of recording -- fix some joints
    //    if(gui_param.record_anim && record_joint_fixed.size()>0)
    //    {
    //        for(int k=0; k<record_joint_fixed.size(); ++k){
    //            int j = record_joint_fixed[k];
    //            skeleton_local_current[j].r = record_joint_fixed_rotation[k];
    //            skeleton_local_current[j].p = record_joint_fixed_position[k];
    //        }
    //    }


    skeleton_local_current = skeleton_local_current_before_deformation;

    // symmetrize
    /*
    if(gui_param.symmetry)
    {
        int joint = picking.selected_joint_true;
        if(joint!=-1)
        {
            quaternion const qx = quaternion::axis_angle({1,0,0},3.14f);
            if(symmetrical_joint.find(joint)!=symmetrical_joint.end())
            {
                int const js = symmetrical_joint[joint];
                skeleton_local_interactive_deformation[js].r = qx * skeleton_local_interactive_deformation[joint].r * conjugate(qx);
            }
        }
    }
    */

#ifdef CTHULHU
    // CTHULHU Anim
    
    {

        ////wings
        quaternion q3 = quaternion::axis_angle(vec3(1, 0, 1), 0.20 * std::sin(3 * t));
        skeleton_local_current[34].r = q3 * skeleton_local_current[34].r;
        quaternion q4 = quaternion::axis_angle(vec3(1, 0, 1), 0.20 * std::sin(3 * t));
        skeleton_local_current[35].r = q4 * skeleton_local_current[35].r;

        ////queue
        quaternion q = quaternion::axis_angle(vec3(1, 0, 0), 0.1 * std::sin(3 * t));
        skeleton_local_current[5].r = q * skeleton_local_current[5].r;

        // head
        quaternion q2 = quaternion::axis_angle(vec3(0, 1, 0), 0.2 * std::sin(1.5f * t));
        quaternion q20 = quaternion::axis_angle(vec3(1, 0, 0), 0.15 * std::sin(3.0f * t));
        skeleton_local_current[6].r = q20 * q2 * skeleton_local_current[6].r;



        quaternion q5 = quaternion::axis_angle(vec3(1, 0.5, 0), -0.25 * std::sin(3 * t - 0.1));
        skeleton_local_current[14].r = q5 * skeleton_local_current[14].r;

        //before wing
        quaternion q8 = quaternion::axis_angle(vec3(1, 0.0, 0), 0.25 * std::sin(3 * t));
        skeleton_local_current[16].r = q8 * skeleton_local_current[16].r;
        skeleton_local_current[25].r = q8 * skeleton_local_current[25].r;

        ////arm right
        quaternion q9 = quaternion::axis_angle(vec3(1, 0, 0), -0.3 * std::sin(3 * t));
        skeleton_local_current[27].r = q9 * skeleton_local_current[27].r;

        ////leg
        quaternion q6 = quaternion::axis_angle(vec3(0, 1, 0), 0.2 * std::sin(3 * t));
        skeleton_local_current[18].r = quaternion::axis_angle(vec3(0, 0, 1), 0.5) * q6 * skeleton_local_current[18].r;
        quaternion q7 = quaternion::axis_angle(vec3(1, 0, 0), -0.3 * std::sin(3 * t));
        skeleton_local_current[28].r = q7 * skeleton_local_current[28].r;

        ////leg-right
        quaternion q10 = quaternion::axis_angle(vec3(1, 0, 0), 0.1 * std::sin(3 * t));
        skeleton_local_current[19].r = q10 * skeleton_local_current[19].r;


        //// position
        skeleton_local_current[0].p.y += 0.2f*std::cos(3 * t - 0.0f);
        quaternion q11 = quaternion::axis_angle(vec3(1, 0, 0), -0.1 * std::sin(3 * t));
        skeleton_local_current[0].r = q11 * skeleton_local_current[0].r;

    }
    
#endif

    
#ifdef COW
    ground.uniform.shading.ambiant = 0.95f;
    ground.uniform.shading.diffuse = 0.0f;
    ground.uniform.shading.specular = 0.0f;
    ground.uniform.color = vec3(0.9f, 1.0f, 0.5f);
    draw(ground, scene.camera);
#endif

    // apply deformation to skeleton
    //std::cout << "apply deformation skeleton" << std::endl;
    for(size_t k=0; k<skeleton_local_current.size(); ++k) {
        skeleton_local_current[k].p += skeleton_local_interactive_deformation[k].p;
        skeleton_local_current[k].r = skeleton_local_interactive_deformation[k].r * skeleton_local_current[k].r;
    }




    skeleton_current = local_to_global(skeleton_local_current, skeleton.connectivity);
    if(is_rotating){
        skeleton_current[0].r = qq;
    }




    // Update per-join velocity
    //std::cout << "per-join velocity" << std::endl;
    for(int k=0; k<int(skeleton_current.size()); ++k) {
        skeleton_joint_speed[k].add( skeleton_local_current[k].p, t );

        skeleton_joint_rotation_speed[k].add( skeleton_local_current[k].r, t );
        
        mat3 R_parent = mat3::identity();
        if(k>0)
            R_parent = skeleton_current[ skeleton.connectivity[k].parent ].r.matrix();

        skeleton_speed_per_joint[k].center = skeleton_current[k].p;
        skeleton_speed_per_joint[k].linear_speed  = R_parent * skeleton_joint_speed[k].avg_speed;
        skeleton_speed_per_joint[k].angular_speed = R_parent * skeleton_joint_rotation_speed[k].avg_rotation_speed;
    }



    if(gui_param.display_rest_pose)
        skeleton_current = skeleton_rest_pose;


    //std::cout << "skeleton to gpu" << std::endl;
    std::vector<mat4> skeleton_to_gpu(skeleton_current.size());
    for (int k = 0; k<int(skeleton_current.size()); ++k) {
        const mat3& R = skeleton_current[k].r.matrix();
        const vec3& p = skeleton_current[k].p;
        const mat4 T = mat4::from_mat3_vec3(R, p);
        skeleton_to_gpu[k] = T;
    }

    std::vector<mat4> skeleton_rest_to_gpu(skeleton_current.size());
    for (int k = 0; k<int(skeleton_rest_pose.size()); ++k) {
        const mat3& R = skeleton_rest_pose[k].r.matrix();
        const vec3& p = skeleton_rest_pose[k].p;
        const mat4 T = mat4::from_mat3_vec3(R, p);
        skeleton_rest_to_gpu[k] = T;
    }

    std::vector<vec3> angular_velocity(skeleton_current.size());
    std::vector<vec3> linear_velocity(skeleton_current.size());
    for (int k = 0; k<int(skeleton_rest_pose.size()); ++k) {
        angular_velocity[k] = (skeleton_speed_per_joint.data[k].angular_speed / timer_skeleton.scale + skeleton_fake_speed_per_joint.data[k].angular_speed);
        linear_velocity[k] = (skeleton_speed_per_joint.data[k].linear_speed / timer_skeleton.scale + skeleton_fake_speed_per_joint.data[k].linear_speed);
    }



    timer_measurement["render"].tic();
    auto shader = shaders["velocity_skinning_raw"];
    opengl_debug();
    glUseProgram(shader); opengl_debug();

    opengl_debug();

    


    uniform(shaders["velocity_skinning_raw"], "t", timer_skeleton.t/timer_skeleton.t_max*(skeleton.anim[0].size()-1)); opengl_debug();
    uniform(shaders["velocity_skinning_raw"], "N_time", int(skeleton.anim[0].size())); opengl_debug();


    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, texture_skinning_weights);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_BUFFER, texture_velocity_skinning_weights);
    glActiveTexture(GL_TEXTURE2); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, tex_sk0); opengl_debug();
    glActiveTexture(GL_TEXTURE3); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, tex_sk); opengl_debug();
    glActiveTexture(GL_TEXTURE4); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, tex_angular_velocity); opengl_debug();
    glActiveTexture(GL_TEXTURE20); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, tex_linear_velocity); opengl_debug();

    glActiveTexture(GL_TEXTURE5); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, texture_floppy_weights_raw); opengl_debug();


    glActiveTexture(GL_TEXTURE6); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, texture_rig_joint); opengl_debug();
    glActiveTexture(GL_TEXTURE7); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, texture_rig_weight); opengl_debug();
    glActiveTexture(GL_TEXTURE8); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, texture_rig_cumulative_index); opengl_debug();
    glActiveTexture(GL_TEXTURE9); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, texture_rig_size); opengl_debug();


    glActiveTexture(GL_TEXTURE10); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, texture_vs_rig_joint); opengl_debug();
    glActiveTexture(GL_TEXTURE11); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, texture_vs_rig_weight); opengl_debug();
    glActiveTexture(GL_TEXTURE12); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, texture_vs_rig_cumulative_index); opengl_debug();
    glActiveTexture(GL_TEXTURE13); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, texture_vs_rig_size); opengl_debug();

    glActiveTexture(GL_TEXTURE14); opengl_debug();
    glBindTexture(GL_TEXTURE_2D, character_visual.texture_id); opengl_debug();
    //glBindBuffer(GL_TEXTURE_BUFFER, 0);





    {
        size_t const N_joint = skeleton_current.size();
        for (int k = 0; k < N_joint; ++k)
        {
            //buffer_tbo_sk0[k] = skeleton_rest_to_gpu[k];
            buffer_tbo_sk[k] = skeleton_to_gpu[k];
            buffer_tbo_angular_velocity[k] = angular_velocity[k];
            buffer_tbo_linear_velocity[k] = linear_velocity[k];
        }


        /*glBindBuffer(GL_TEXTURE_BUFFER, tbo_sk0);
        glBufferSubData(GL_TEXTURE_BUFFER, 0, sizeof(float) * buffer_tbo_sk0.size() * 16, &buffer_tbo_sk0[0][0]);*/
        glBindBuffer(GL_TEXTURE_BUFFER, tbo_sk);
        glBufferSubData(GL_TEXTURE_BUFFER, 0, sizeof(float) * buffer_tbo_sk.size() * 16, &buffer_tbo_sk[0][0]);
        glBindBuffer(GL_TEXTURE_BUFFER, tbo_angular_velocity);
        glBufferSubData(GL_TEXTURE_BUFFER, 0, sizeof(float) * buffer_tbo_angular_velocity.size() * 3, &buffer_tbo_angular_velocity[0].x);
        glBindBuffer(GL_TEXTURE_BUFFER, tbo_linear_velocity);
        glBufferSubData(GL_TEXTURE_BUFFER, 0, sizeof(float) * buffer_tbo_linear_velocity.size() * 3, &buffer_tbo_linear_velocity[0].x);
    }


    uniform(shader, "N_vertex", int(point_set_raw.size())); opengl_debug();
    uniform(shader, "N_joint", int(skeleton_current.size())); opengl_debug();
    uniform(shader, "floppy_power", GLfloat(flapping_power)); opengl_debug();
    uniform(shader, "squashy_power", GLfloat(squashing_power)); opengl_debug();







    mat3 Id = mat3::identity();
    uniform(shader, "rotation", Id);            opengl_debug();
    uniform(shader, "translation", vec3(0,0,0));      opengl_debug();
    uniform(shader, "color", vec3(1,1,1));                            opengl_debug();
    uniform(shader, "color_alpha", 1.0f);                opengl_debug();
    uniform(shader, "scaling", 1.0f);              opengl_debug();
    uniform(shader, "scaling_axis", vec3(1,1,1));    opengl_debug();

    uniform(shader, "perspective", scene.camera.perspective.matrix());         opengl_debug();
    uniform(shader, "view", scene.camera.view_matrix());                       opengl_debug();
    uniform(shader, "camera_position", scene.camera.camera_position());        opengl_debug();

    uniform(shader, "ambiant", 0.3f);      opengl_debug();
    uniform(shader, "diffuse", 0.6f);      opengl_debug();
    uniform(shader, "specular", 0.8f);    opengl_debug();
    uniform(shader, "specular_exponent", 128); opengl_debug();

    //uniform bool floppy_weight_active = false;
    //uniform bool floppy_limit = false;
    //uniform bool gpu_velocity_skinning = true;
    //uniform bool color_weight = false;
    uniform(shader, "gpu_velocity_skinning", gui_param.gpu_velocity_skinning); opengl_debug();
    uniform(shader, "floppy_weight_active", gui_param.gpu_floppy_weight_active); opengl_debug();
    uniform(shader, "floppy_limit", gui_param.gpu_floppy_limit); opengl_debug();
    uniform(shader, "color_weight", gui_param.gpu_color_floppy_weight); opengl_debug();


    glActiveTexture(GL_TEXTURE17); opengl_debug();
    glBindTexture(GL_TEXTURE_BUFFER, texture_T_anim); opengl_debug();

    glBindVertexArray(vao_raw); opengl_debug();
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer_connectivity_raw); opengl_debug();
#ifdef COW
    glDrawElementsInstanced(GL_TRIANGLES, GLsizei(connectivity_raw.size() * 3), GL_UNSIGNED_INT, nullptr, N_cow); opengl_debug();
#endif
#ifdef CTHULHU
    glDrawElements(GL_TRIANGLES, GLsizei(connectivity_raw.size() * 3), GL_UNSIGNED_INT, nullptr); opengl_debug();
#endif
    glBindVertexArray(0);

    if (gui_param.display_wireframe) {
        shader = shaders["wireframe"];
        glUseProgram(shaders["wireframe"]); opengl_debug();
        uniform(shader, "rotation", Id);            opengl_debug();
        uniform(shader, "translation", vec3(0, 0, 0));      opengl_debug();
        uniform(shader, "scaling", 1.0f);              opengl_debug();
        uniform(shader, "scaling_axis", vec3(1, 1, 1));    opengl_debug();
        uniform(shader, "perspective", scene.camera.perspective.matrix());         opengl_debug();
        uniform(shader, "view", scene.camera.view_matrix());                       opengl_debug();
        glBindVertexArray(vao_raw); opengl_debug();
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer_connectivity_raw); opengl_debug();
        glDrawElements(GL_TRIANGLES, GLsizei(connectivity_raw.size() * 3), GL_UNSIGNED_INT, nullptr); opengl_debug();
        glBindVertexArray(0);
    }

    glActiveTexture(GL_TEXTURE4); opengl_debug();
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE3); opengl_debug();
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE2); opengl_debug();
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE1); opengl_debug();
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0); opengl_debug();
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
    timer_measurement["render"].toc();
    
    



    //recording
    {
        if(gui_param.record_anim){
            if(local_time_record<timer_skeleton.t_max)
                timer_skeleton.t = std::min(local_time_record, timer_skeleton.t_max-0.01f);
        }


        local_time_record += timer_recording.update();
        if(timer_recording.event)
        {

            record_rotation.push_back(skeleton_local_current[recorded_joint].r);
            record_position.push_back(skeleton_local_current[recorded_joint].p);

        }



    }


    if(gui_param.x_ray)
        glClear(GL_DEPTH_BUFFER_BIT);

    if(gui_param.display_skeleton_bones)
        display_skeleton(skeleton_current, skeleton.connectivity, shaders, scene, segment_drawer);
    if(gui_param.display_skeleton_joints)
        display_joints(skeleton_current, scene, sphere_visual);
    if(gui_param.display_skeleton_frames){
        frame.uniform.transform.scaling = 0.08f*gui_param.frame_scaling;
        display_frames(skeleton_current, scene, frame);
    }

    if(gui_param.display_vertex_to_bone_correspondance) {
        display_vertex_to_bone_correspondance(skinning.deformed.position, skeleton_current, vertex_to_bone_correspondance, segment_drawer, shaders["segment_im"], scene.camera);
    }


    if(gui_param.display_skeleton_pyramid)
    {
        const size_t N = skeleton_current.size();
        for(size_t k=1; k<N; ++k)
        {
            //std::string const name = joint_name_raw[k];
            //if (name[0] != 'F')
            {
                int parent = skeleton.connectivity[k].parent;

                vec3 const& p0 = skeleton_current[parent].p;
                vec3 const& p1 = skeleton_current[k].p;

                vec3 const u = p1 - p0;
                const float L = norm(p1 - p0);
                const float Lr = std::min(L, 0.2f);
                vec3 const un = u / L;

                if (picking.joint_hover == int(k))
                    pyramid_skeleton_visual.uniform.color = { 0.8f, 0.3f, 0.3f };
                else
                    pyramid_skeleton_visual.uniform.color = { 0.3f, 0.3f, 0.3f };
                pyramid_skeleton_visual.uniform.transform.translation = p0;
                pyramid_skeleton_visual.uniform.transform.scaling_axis = { Lr,Lr,L };
                pyramid_skeleton_visual.uniform.transform.rotation = rotation_between_vector_mat3({ 0,0,1 }, un);

                draw(pyramid_skeleton_visual, scene.camera);
            }
        }
    }


    {
        segment_drawer.uniform_parameter.color = {1,0,0};
        int N_joint = skeleton_current.size();
        for(int k_joint=0; k_joint<N_joint; ++k_joint)
        {
            vec3 const& p = skeleton_current[k_joint].p;

            if(gui_param.display_joint_linear_speed)
            {
                vec3 const& v = skeleton_speed_per_joint[k_joint].linear_speed + skeleton_fake_speed_per_joint[k_joint].linear_speed;
                arrow.draw(p, p+0.2f*v, {0.3f,0.6f,1.0f}, scene.camera);
            }
            if(gui_param.display_joint_angular_speed)
            {
                vec3 const& v = skeleton_speed_per_joint[k_joint].angular_speed + skeleton_fake_speed_per_joint[k_joint].angular_speed;
                arrow.draw(p, p+0.2f*v, {0.6f,1.0f,0.3f}, scene.camera);
            }
        }


        if(gui_param.display_center_of_mass)
        {
            bool is_translation = (gui_param.type_deformation==0 || picking.selected_joint==0);
            int joint = picking.selected_joint_true;
            if(joint!=-1)
            {
                vec3 const p_com = position_center_of_mass(joint);//center_of_mass_per_joint[joint];
                sphere_visual.uniform.color = {1.0f,0.4f,1.0f};
                sphere_visual.uniform.transform.scaling = 0.02f;
                sphere_visual.uniform.transform.translation = p_com;
                draw(sphere_visual, scene.camera);

                vec3 const& p_joint = skeleton_current[joint].p;
                vec3 const u_joint_com = p_com-p_joint;
                if(!is_translation && gui_param.squash_around==0)
                    arrow_line.draw(p_joint - u_joint_com*0.5f, p_joint + u_joint_com*2.0f, {0.8f,0.2f,0.8f}, scene.camera);

            }
        }
    }



    //  X-RAY display beside this point
    glClear(GL_DEPTH_BUFFER_BIT);








    segment_drawer.uniform_parameter.color = {0,0,0};

    // Display sphere when hovering over joint ready to be picked
    if(picking.joint_hover!=-1)
        display_sphere_hover(sphere_visual, picking.joint_hover, skeleton_current, scene.camera);

    if(picking.is_selected)
        display_sphere_selected(sphere_visual, picking.p_clicked, picking.p_current, scene.camera);


    if(gui_param.painting.activated)
        if(picking.painting_selected_vertex>-1)
            display_painting_cursor(picking.painting_selected_vertex, skinning, gui_param.painting.radius, gui_param.painting.threshold_percentage, painting_cursor, sphere_visual, scene.camera);



    glClear(GL_DEPTH_BUFFER_BIT);
    if(gui_param.record_anim)
    {
        vec3 e1 = scene.camera.camera_matrix().mat3().row(0);
        vec3 e2 = scene.camera.camera_matrix().mat3().row(1);
        vec3 e3 = scene.camera.camera_matrix().mat3().row(2);

        sphere_visual.uniform.shading = {1.0f, 0.0f, 0.0f};
        sphere_visual.uniform.transform.scaling = 0.02f;
        sphere_visual.uniform.transform.translation = scene.camera.camera_position() - 2.0f*e3 + 0.65f*e2 + 0.75f*e1;

        sphere_visual.uniform.color = {1,0,0};
        draw(sphere_visual, scene.camera);
    }


    //timer_measurement["drawing"].toc();

    timer_measurement["full"].toc();



    /*
    if (gui_param.x_ray)
        glClear(GL_DEPTH_BUFFER_BIT);

    if (gui_param.display_skeleton_bones)
        display_skeleton(skeleton_current, skeleton.connectivity, shaders, scene, segment_drawer);
    if (gui_param.display_skeleton_joints)
        display_joints(skeleton_current, scene, sphere_visual);
    if (gui_param.display_skeleton_frames) {
        frame.uniform.transform.scaling = 0.08f * gui_param.frame_scaling;
        display_frames(skeleton_current, scene, frame);
    }*/

}



void scene_model::diffuse_weight()
{
    if(gui_param.painting.display_weights==1)
        weight_flappy = diffuse_laplacien_weights(weight_flappy, one_ring, 0.1f, 10);
    if(gui_param.painting.display_weights==2)
        weight_squashy = diffuse_laplacien_weights(weight_squashy, one_ring, 0.1f, 10);
    update_painted_color();
}


void scene_model::set_gui()
{
    ImGui::SetWindowFontScale(1.2f);

    ImGui::Checkbox("GPU velocity skinning", &gui_param.gpu_velocity_skinning);
    ImGui::Checkbox("GPU floppy weight", &gui_param.gpu_floppy_weight_active);
    ImGui::Checkbox("GPU floppy limit", &gui_param.gpu_floppy_limit);
    ImGui::Checkbox("GPU color weight", &gui_param.gpu_color_floppy_weight);
    


    if(ImGui::CollapsingHeader("Animation"))
    {
        ImGui::Text("Animation:"); ImGui::SameLine();
        ImGui::Checkbox("Run", &gui_param.animation); ImGui::SameLine();
        bool const stop  = ImGui::Button("Stop"); ImGui::SameLine();
        bool const start = ImGui::Button("Start");
        if(stop)  timer.stop();
        if(start) timer.start();

        ImGui::SliderFloat("Timer",  &timer_skeleton.t, timer_skeleton.t_min, timer_skeleton.t_max, "%.2f s");
        ImGui::SliderFloat("Time scale", &timer_skeleton.scale, 0.005f, 3.0f, "%.3f s");
    }


    if(ImGui::CollapsingHeader("Display Mesh"))
    {
        ImGui::Checkbox("Mesh", &gui_param.display_mesh); ImGui::SameLine();
        ImGui::Checkbox("Wireframe", &gui_param.display_wireframe); ImGui::SameLine();
        ImGui::Checkbox("Texture", &gui_param.display_texture);
    }

    if(ImGui::CollapsingHeader("Sk. display"))
    {
        ImGui::Checkbox("Bones", &gui_param.display_skeleton_bones);  ImGui::SameLine();
        ImGui::Checkbox("Bones pyramid", &gui_param.display_skeleton_pyramid);
        ImGui::Checkbox("Joints", &gui_param.display_skeleton_joints);  ImGui::SameLine();
        ImGui::Checkbox("Frames", &gui_param.display_skeleton_frames);
        ImGui::SliderFloat("Frame Scaling", &gui_param.frame_scaling, 0.1f, 3.0f);
        ImGui::Checkbox("X-Ray", &gui_param.x_ray);

    }

    if(ImGui::CollapsingHeader("Skinning param."))
    {
        ImGui::Checkbox("Rest pose", &gui_param.display_rest_pose);
        ImGui::Checkbox("Interpolate skeleton", &gui_param.interpolate);
        ImGui::Checkbox("Dual quaternion", &gui_param.dual_quaternion);
    }





    if(ImGui::CollapsingHeader("Sk. Interaction"))
    {
        ImGui::Checkbox("Symmetry", &gui_param.symmetry);

        ImGui::RadioButton("Translation", &gui_param.type_deformation, 0); ImGui::SameLine();
        ImGui::RadioButton("Rotation", &gui_param.type_deformation, 1);

    }


    if(ImGui::CollapsingHeader("Squashy"))
    {
        ImGui::Text("Parameters");


        ImGui::SliderFloat("Flapping power", &flapping_power, 0.0f, 5.0f, "%.2f s", 2.0f);
        ImGui::SliderFloat("Squashing power", &squashing_power, 0.0f, 5.0f, "%.2f s", 2.0f);
        ImGui::SliderFloat("Flapping max angle", &gui_param.flappy_max_angle, 0, 3.14f);

        ImGui::Text("Squash around:"); ImGui::SameLine();
        ImGui::RadioButton("Axis",&gui_param.squash_around,0); ImGui::SameLine();
        ImGui::RadioButton("Center",&gui_param.squash_around,1);
        ImGui::Checkbox("Center of mass", &gui_param.display_center_of_mass);



        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::Text("Debug:");
        //ImGui::Checkbox("Vertex to bone correspondance", &gui_param.display_vertex_to_bone_correspondance);

        ImGui::Checkbox("Fake Speed", &gui_param.fake_speed);
        ImGui::Text("Display deformation:"); ImGui::SameLine();
        ImGui::Checkbox("Surface", &gui_param.display_deformed_surface); ImGui::SameLine();
        ImGui::Checkbox("Arrows",&gui_param.display_deformation_arrows); ImGui::SameLine();
        ImGui::Checkbox("Target",&gui_param.display_deformation_target);
        ImGui::SliderInt("Vertex Offset", &gui_param.display_deformation_arrow_vertex_offset, 1,15);
        ImGui::Checkbox("Curved trajectory", &gui_param.curved_trajectory);

        ImGui::Text("Display joint velocity:"); ImGui::SameLine();
        ImGui::Checkbox("Linear", &gui_param.display_joint_linear_speed); ImGui::SameLine();
        ImGui::Checkbox("Angular", &gui_param.display_joint_angular_speed);



        /*
        ImGui::Checkbox("Flapping", &is_flapping);
        ImGui::Checkbox("Basic Flapping", &basic_flapping);
        ImGui::Checkbox("Cylinder Flapping", &cylinder_flapping);
        ImGui::Checkbox("Display angular speed", &display_angular_speed);
        ImGui::Checkbox("Is rotating", &is_rotating);
        */
    }






    /*
    if(ImGui::Button("Non negative weights"))
    {
        for(int k=0; k<skinning.deformed.position.size(); ++k)
            weight_flappy[k] = std::abs(weight_flappy[k]);
    }

    if( ImGui::Button("Color white") ) {
        for(int k=0; k<skinning.deformed.position.size(); ++k) {
            skinning.deformed.color[k] = {1.0f, 1.0f, 1.0f, 0};
            character_visual.update_color(skinning.deformed.color);
        }
    }
    ImGui::SameLine();
    if( ImGui::Button("Color flappy") ) {
        for(int k=0; k<skinning.deformed.position.size(); ++k) {
            skinning.deformed.color[k] = {1-std::max(weight_flappy[k],0.0f),1+std::min(weight_flappy[k],0.0f),1,0};
            character_visual.update_color(skinning.deformed.color);
        }
    }
    ImGui::SameLine();
    if( ImGui::Button("Color squashy") ) {
        for(int k=0; k<skinning.deformed.position.size(); ++k) {
            skinning.deformed.color[k] = {std::max(weight_squashy[k],0.0f),-std::min(weight_squashy[k],0.0f),0,0};
            character_visual.update_color(skinning.deformed.color);
        }
    }
    */






    if(ImGui::CollapsingHeader("Painting")){
        bool activate = ImGui::Checkbox("Activate Painting Mode", &gui_param.painting.activated);
        if(activate) {
            if(gui_param.painting.activated==true)
                gui_param.painting.display_weights = 1;
            else
                gui_param.painting.display_weights = 0;
            update_painted_color();
        }

        ImGui::Text("Display color");
        bool color_none = ImGui::RadioButton("None", &gui_param.painting.display_weights, 0); ImGui::SameLine();
        bool color_flappy = ImGui::RadioButton("Flappy", &gui_param.painting.display_weights, 1);
        //ImGui::RadioButton("Squashy", &gui_param.painting.display_weights, 2);
        if(color_none || color_flappy)
            update_painted_color();



        ImGui::Text("Value to paint:");
        ImGui::SliderFloat("Value", &gui_param.painting.value, -1.0f, 1.0f); ImGui::SameLine();
        bool zero = ImGui::Button("Zero");
        bool fill = ImGui::Button("Fill");
        if(zero) gui_param.painting.value = 0.0f;
        if(fill) { weight_flappy.fill(gui_param.painting.value); update_painted_color(); }
        ImGui::SliderFloat("Radius", &gui_param.painting.radius, 0.01f, 0.2f);
        ImGui::SliderFloat("Threshold", &gui_param.painting.threshold_percentage, 0.0f, 1.0f);


        bool diffuse = ImGui::Button("Diffuse");
        if(diffuse) diffuse_weight();
    }



    if(ImGui::CollapsingHeader("Model")){

        bool click_sphere    = ImGui::RadioButton("Sphere", &gui_param.display_type, display_sphere); ImGui::SameLine();
        bool click_rondinella = ImGui::RadioButton("Rondinella", &gui_param.display_type, display_rondinella);

        bool click_cylinder_bending   = ImGui::RadioButton("Cylinder B.", &gui_param.display_type, display_cylinder_bending); ImGui::SameLine();
        bool click_cylinder_translate = ImGui::RadioButton("(Tr.)", &gui_param.display_type, display_cylinder_translate); ImGui::SameLine();
        bool click_bar       = ImGui::RadioButton("Bar", &gui_param.display_type, display_bar);

        bool click_character = ImGui::RadioButton("Character", &gui_param.display_type, display_character);
        bool click_girafe = ImGui::RadioButton("Girafe", &gui_param.display_type, display_girafe);
        bool click_spot = ImGui::RadioButton("Spot", &gui_param.display_type, display_spot);
        bool click_dragon = ImGui::RadioButton("Dragon", &gui_param.display_type, display_dragon);
        bool click_snail = ImGui::RadioButton("Snail", &gui_param.display_type, display_snail);
        bool click_armadillo = ImGui::RadioButton("Armadillo", &gui_param.display_type, display_armadillo);


        if(click_sphere)    load_sphere_data(skeleton, skinning, weight_flappy, character_visual, timer_skeleton, shader_mesh);
        if(click_cylinder_bending)  load_bending_twisting_cylinder_data(skeleton, skinning, weight_flappy, weight_squashy, character_visual, timer_skeleton, shader_mesh);
        if(click_cylinder_translate)  load_diagonal_translate_cylinder_data(skeleton, skinning, weight_flappy, character_visual, timer_skeleton, shader_mesh);
        if(click_rondinella)  load_rondinella_data(skeleton, skinning, weight_flappy, weight_squashy, character_visual, timer_skeleton, shader_mesh);

        if(click_bar)       load_rectangle_data(skeleton, skinning, character_visual, timer_skeleton, shader_mesh);

        if(click_character) load_character_data(skeleton, skinning, weight_flappy, character_visual, timer_skeleton, shader_mesh);

        data_loaded data_load;
        if(click_girafe)  data_load = load_girafe2_data(shaders["mesh"]);
        if(click_spot)    data_load = load_spot_data(shaders["mesh"]);
        if(click_dragon)  data_load = load_dragon_data(shaders["mesh"]);
        if(click_snail)   data_load = load_snail_data(shaders["mesh"]);
        if(click_armadillo)   data_load = load_armadillo_data(shaders["mesh"]);


        if(click_girafe || click_spot || click_dragon || click_snail || click_armadillo)
        {
            skeleton = data_load.skeleton;
            skinning.influence = data_load.skinning_rig;
            skinning.deformed = data_load.shape;
            skinning.rest_pose = data_load.shape.position;
            skinning.rest_pose_normal = data_load.shape.normal;
            symmetrical_joint = data_load.symmetry;
            weight_flappy = data_load.weight_flappy;
            character_visual = data_load.shape;
            character_visual.shader = data_load.shader;
            character_visual.texture_id = data_load.texture_id;
            timer_skeleton.t_max = data_load.anim_time_max;
        }

        if(click_sphere || click_cylinder_bending || click_cylinder_translate || click_rondinella ||
                click_bar || click_character || click_girafe || click_spot ||  click_dragon || click_snail || click_armadillo) {
            resize_structure();
        }
    }

    if(ImGui::CollapsingHeader("Timings")){

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#pragma GCC diagnostic ignored "-Wformat"
#endif

#define S 64

        static float total = 0;
        static timer_event timer_update_timing;
        static std::map<float,std::string> ordered_time; // reversed timing ordered per time
        timer_update_timing.periodic_event_time_step = 2;
        timer_update_timing.update();
        if(timer_update_timing.event)
        {
            total = timer_measurement["full"].t;
            ordered_time.clear();
            for(auto const& timing: timer_measurement)
                ordered_time[timing.second.t] = timing.first;
        }



        char buffer[S];
        snprintf(buffer, S, "Total : %.1fs", double(total));
        ImGui::Text( buffer );
        for(auto it=ordered_time.rbegin(); it!=ordered_time.rend(); ++it)
        {
            std::string const& name = it->second;
            float const t = it->first;
            float const avg = timer_measurement[name].average_timing;
            if(name!="full")
            {
                char buffer_timing[S];
                snprintf(buffer_timing, S, "[%d perc] [%.1fms] %s - %.1fs  ", int(t/total*100), double(1000*avg), name.c_str(), double(t));
                ImGui::Text( buffer_timing );
            }
        }

        bool reset = ImGui::Button("Reset timer");
        if(reset)
        {
            timer_measurement.clear();
            ordered_time.clear();
            total = 0;
        }

#undef S
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

    }

    //    {
    //        static int counter = 0;
    //        if(counter%500==0)
    //        {
    //            std::cout<<"\nTimings : "<<std::endl;
    //            for(auto const& elapsed : timer_measurement)
    //            {
    //                std::cout<<elapsed.first<<" : "<<elapsed.second.t<<" "<<int(elapsed.second.t/timer_measurement["full"].t*100)<<"%" <<std::endl;
    //            }
    //            std::cout<<std::endl;
    //        }
    //        counter ++;
    //    }

}


void scene_model::resize_structure()
{
    skeleton_local_current_before_deformation = interpolate_skeleton_at_time(0, skeleton.anim, gui_param.interpolate);
    skeleton_local_current = skeleton_local_current_before_deformation;
    int const N_joint = skeleton_local_current.size();
    int const N_vertex = skinning.rest_pose.size();
    skeleton_local_interactive_deformation.resize(N_joint);


    skeleton_current = local_to_global(skeleton_local_current, skeleton.connectivity);
    //skeleton_speed.resize(skeleton_current.size());

    //skeleton_acceleration.resize(skeleton_current.size());
    //skeleton_velocity_tracker.resize(skeleton_current.size());
    skeleton_rest_pose = local_to_global(skeleton.rest_pose, skeleton.connectivity);
    //skeleton_angular_velocity_tracker.resize(skeleton_current.size());


    //previous_angular_velocity.resize(skinning.rest_pose.size());
    //previous_speed.resize(skinning.rest_pose.size());

    if (weight_squashy.size() != skinning.rest_pose.size()) {
        weight_squashy.resize(skinning.rest_pose.size());
        weight_squashy.fill(1.0f);
    }
    if (weight_flappy.size() != skinning.rest_pose.size()) {
        weight_flappy.resize(skinning.rest_pose.size());
        weight_flappy.fill(0.0f);
    }

    //vertex_speed.clear(); vertex_speed.resize(N_vertex);
    skeleton_joint_speed.clear(); skeleton_joint_speed.resize(N_joint);

    skeleton_joint_rotation_speed.resize_clear(skeleton_current.size());
    skeleton_speed_per_joint.resize_clear(N_joint);
    skeleton_fake_speed_per_joint.resize_clear(N_joint);

    // build one ring
    {
        size_t const N = skinning.deformed.position.size();
        auto const& connectivity = skinning.deformed.connectivity;
        one_ring.clear();
        one_ring.resize(N);

        size_t const N_tri = connectivity.size();
        for (size_t k_tri = 0; k_tri < N_tri; ++k_tri)
        {
            unsigned int const u0 = connectivity[k_tri][0];
            unsigned int const u1 = connectivity[k_tri][1];
            unsigned int const u2 = connectivity[k_tri][2];

            one_ring[u0].insert(u1); one_ring[u0].insert(u2);
            one_ring[u1].insert(u0); one_ring[u1].insert(u2);
            one_ring[u2].insert(u0); one_ring[u2].insert(u1);
        }
    }

    // Build triangle around vertex
    {
        size_t const N_triangle = skinning.deformed.connectivity.size();
        triangle_around_vertex.resize_clear(N_vertex);
        for (size_t kt = 0; kt < N_triangle; ++kt)
        {
            uint3 const& tri = skinning.deformed.connectivity[kt];
            unsigned int a = tri[0], b = tri[1], c = tri[2];
            triangle_around_vertex[a].push_back(kt);
            triangle_around_vertex[b].push_back(kt);
            triangle_around_vertex[c].push_back(kt);
        }
    }

    update_painted_color();

    skeleton_son_connectivity = compute_joint_sons(skeleton.connectivity);
    vertex_to_bone_correspondance = compute_bone_correspondance(skinning.rest_pose, skinning.influence, skeleton_son_connectivity, skeleton_rest_pose);


    // Rig extended to ancestor
    {
        rig_extended_to_ancestor_joint.resize_clear(N_vertex);
        rig_extended_to_ancestor_weight.resize_clear(N_vertex);

        for (int kv = 0; kv < N_vertex; ++kv)
        {
            auto const& influence = skinning.influence[kv];
            int const N_influence = influence.size();

            std::map<int, float> cumulative_weight_per_joint;

            for (int k_influence = 0; k_influence < N_influence; ++k_influence)
            {
                int current_joint = influence[k_influence].joint;
                float weight = influence[k_influence].weight;

                while (current_joint != -1)
                {

                    if (cumulative_weight_per_joint.find(current_joint) != cumulative_weight_per_joint.end())
                        cumulative_weight_per_joint[current_joint] += weight;
                    else
                        cumulative_weight_per_joint[current_joint] = weight;

                    current_joint = skeleton.connectivity[current_joint].parent;
                }
            }

            for (auto const& it : cumulative_weight_per_joint)
            {
                rig_extended_to_ancestor_joint[kv].push_back(it.first);
                rig_extended_to_ancestor_weight[kv].push_back(it.second);
            }

        }

        //        for(int kv=0; kv<N_vertex; ++kv)
        //        {
        //            for(int k=0; k<rig_extended_to_ancestor_joint[kv].size(); ++k)
        //            {
        //                std::cout<<kv<<" - "<<rig_extended_to_ancestor_joint[kv][k]<<","<<rig_extended_to_ancestor_weight[kv][k]<<std::endl;
        //            }
        //            std::cout<<std::endl;
        //        }
    }

    vertex_depending_on_joint.resize_clear(N_joint);
    vertex_weight_depending_on_joint.resize_clear(N_joint);

    for (int k_vertex = 0; k_vertex < N_vertex; ++k_vertex)
    {
        buffer<int> const& joint_dependencies = rig_extended_to_ancestor_joint[k_vertex];
        for (int k_joint = 0; k_joint<int(joint_dependencies.size()); ++k_joint)
        {
            int joint = rig_extended_to_ancestor_joint[k_vertex][k_joint];
            float weight = rig_extended_to_ancestor_weight[k_vertex][k_joint];
            vertex_depending_on_joint[joint].push_back(k_vertex);
            vertex_weight_depending_on_joint[joint].push_back(weight);
        }
    }

    skinning_weights_per_joint_per_vertex.resize_clear(N_joint);
    for (int kj = 0; kj < N_joint; ++kj)
    {
        skinning_weights_per_joint_per_vertex[kj].resize_clear(N_vertex);
        for (size_t k = 0; k < vertex_depending_on_joint[kj].size(); ++k)
        {
            int vertex_id = vertex_depending_on_joint[kj][k];
            float w = vertex_weight_depending_on_joint[kj][k];
            skinning_weights_per_joint_per_vertex[kj][vertex_id] = w;
        }

    }



    vcl::buffer<std::set<int> > triangle_set_depending_on_joint(N_joint);
    int const N_triangle = skinning.deformed.connectivity.size();
    for (int k_triangle = 0; k_triangle < N_triangle; ++k_triangle)
    {
        uint3 const& tri = skinning.deformed.connectivity[k_triangle];
        unsigned int a = tri[0];
        unsigned int b = tri[1];
        unsigned int c = tri[2];

        buffer<int> const& joints_a = rig_extended_to_ancestor_joint[a];
        buffer<int> const& joints_b = rig_extended_to_ancestor_joint[b];
        buffer<int> const& joints_c = rig_extended_to_ancestor_joint[c];

        for (int j : joints_a)
            triangle_set_depending_on_joint[j].insert(k_triangle);
        for (int j : joints_b)
            triangle_set_depending_on_joint[j].insert(k_triangle);
        for (int j : joints_c)
            triangle_set_depending_on_joint[j].insert(k_triangle);
    }

    triangle_depending_on_joint.resize_clear(N_joint);
    for (int kj = 0; kj < N_joint; ++kj)
    {
        for (int t : triangle_set_depending_on_joint[kj])
            triangle_depending_on_joint[kj].push_back(t);
    }


    triangle_area.resize_clear(N_triangle);
    triangle_center.resize_clear(N_triangle);


    center_of_mass_per_joint.resize_clear(N_joint);
    center_of_mass_per_joint_manual_offset.resize_clear(N_joint);

    timer_skeleton.t = timer_skeleton.t_min;

    picking.is_selected = false;
    picking.selected_joint = -1;
    picking.selected_joint_true = -1;
    recorded_joint = -1;

    deformation_per_vertex.resize_clear(N_vertex);


    character_visual.shader = shaders["skinning"];

    // Texture
    {
        const size_t N_vertex = skinning.rest_pose.size();
        const size_t N_joint = skeleton.rest_pose.size();
        vcl::buffer2D<float> grid;
        grid.resize({ N_vertex, N_joint });
        grid.fill(0.0f);

        for (size_t k_vertex = 0; k_vertex < N_vertex; ++k_vertex)
        {
            size_t const N_influence = skinning.influence[k_vertex].size();
            for (size_t k_influence = 0; k_influence < N_influence; ++k_influence) {
                int joint = skinning.influence[k_vertex][k_influence].joint;
                float w = skinning.influence[k_vertex][k_influence].weight;

                grid(k_vertex, joint) = w;
            }
        }

        if (character_visual.texture_id == 0)
            character_visual.texture_id = scene.texture_white;
        /*
        const size_t N_vertex = skinning.rest_pose.size();
        const size_t N_joint = skeleton.rest_pose.size();
        vcl::buffer2D<vec3> grid;
        grid.resize({ N_vertex, N_joint });
        grid.fill(vec3(0, 0, 0));

        for (size_t k_vertex = 0; k_vertex < N_vertex; ++k_vertex)
        {
            size_t const N_influence = skinning.influence[k_vertex].size();
            for (size_t k_influence = 0; k_influence < N_influence; ++k_influence) {
                int joint = skinning.influence[k_vertex][k_influence].joint;
                float w = skinning.influence[k_vertex][k_influence].weight;

                grid(k_vertex, joint).x = w;
            }
        }*/

        {
            buffer_skinning_weights = 0;
            glGenBuffers(1, &buffer_skinning_weights);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_skinning_weights);
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float)*grid.size(), &grid(0,0), GL_STATIC_DRAW);
        }

        {
            //velocity skinning weights
            {
                grid.resize({ N_vertex, N_joint });
                grid.fill(0.0f);

                for (size_t k_vertex = 0; k_vertex < N_vertex; ++k_vertex)
                {

                    size_t const N_influence = rig_extended_to_ancestor_joint[k_vertex].size();
                    for (size_t k_influence = 0; k_influence < N_influence; ++k_influence) {
                        int joint = rig_extended_to_ancestor_joint[k_vertex][k_influence];
                        float w = rig_extended_to_ancestor_weight[k_vertex][k_influence];

                        grid(k_vertex, joint) = w;
                    }
                }

                buffer_velocity_skinning_weights = 0;
                glGenBuffers(1, &buffer_velocity_skinning_weights);
                glBindBuffer(GL_TEXTURE_BUFFER, buffer_velocity_skinning_weights);
                glBufferData(GL_TEXTURE_BUFFER, sizeof(float) * grid.size(), &grid(0, 0), GL_STATIC_DRAW);

            }
        }


        {
            // TBO for skeleton
            size_t const N_vertex = skeleton_rest_pose.size();
            
            buffer_tbo_sk0.resize(N_vertex);
            buffer_tbo_sk.resize(N_vertex);
            buffer_tbo_angular_velocity.resize(N_vertex);
            

            glGenBuffers(1, &tbo_sk0);
            glGenBuffers(1, &tbo_sk);
            glGenBuffers(1, &tbo_angular_velocity);

            
            glBindBuffer(GL_TEXTURE_BUFFER, tbo_sk0);
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float) * buffer_tbo_sk0.size() * 16, &buffer_tbo_sk0[0][0], GL_STATIC_DRAW);
            glBindBuffer(GL_TEXTURE_BUFFER, tbo_sk);
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float) * buffer_tbo_sk.size() * 16, &buffer_tbo_sk[0][0], GL_STATIC_DRAW);
            glBindBuffer(GL_TEXTURE_BUFFER, tbo_angular_velocity);
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float) * buffer_tbo_angular_velocity.size() * 3, &buffer_tbo_angular_velocity[0].x, GL_STATIC_DRAW);


        }

        {
            glGenTextures(1, &texture_skinning_weights);
            glGenTextures(1, &texture_velocity_skinning_weights);

            glUseProgram(shaders["velocity_skinning"]);
            glBindVertexArray(character_visual.data.vao); opengl_debug();

            glActiveTexture(GL_TEXTURE1); opengl_debug();
            glBindTexture(GL_TEXTURE_2D, character_visual.texture_id); opengl_debug();
            uniform(shaders["velocity_skinning"], "texture_sampler", int(1)); opengl_debug();
            
            glActiveTexture(GL_TEXTURE0); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_skinning_weights); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_skinning_weights); opengl_debug();
            uniform(shaders["velocity_skinning"], "tbo_weights", int(0)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glActiveTexture(GL_TEXTURE2); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_velocity_skinning_weights); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_velocity_skinning_weights); opengl_debug();
            uniform(shaders["velocity_skinning"], "tbo_velocity_weights", int(2)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);
            


            glGenTextures(1, &tex_sk0);
            glGenTextures(1, &tex_sk);
            glGenTextures(1, &tex_angular_velocity);

            glActiveTexture(GL_TEXTURE3); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, tex_sk0); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, tbo_sk0); opengl_debug();
            uniform(shaders["velocity_skinning"], "tbo_sk0", int(3)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);
            
            glActiveTexture(GL_TEXTURE4); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, tex_sk); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, tbo_sk); opengl_debug();
            uniform(shaders["velocity_skinning"], "tbo_sk", int(4)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glActiveTexture(GL_TEXTURE5); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, tex_angular_velocity); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, tbo_angular_velocity); opengl_debug();
            uniform(shaders["velocity_skinning"], "tbo_angular_velocity", int(5)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);
           
        }
    }
}

void scene_model::setup_data(std::map<std::string,GLuint>& _shaders, scene_structure& _scene, gui_structure& gui)
{

    scene = _scene;
    shaders = _shaders;
    shader_mesh = shaders["mesh_bf"];

    shaders["depth_map"] = create_shader_program("scenes/shared_assets/shaders/depth_map/shader.vert.glsl",
                                                 "scenes/shared_assets/shaders/depth_map/shader.frag.glsl");

    shaders["mesh_shadow"] = create_shader_program("scenes/shared_assets/shaders/mesh_shadow/shader.vert.glsl",
                                                   "scenes/shared_assets/shaders/mesh_shadow/shader.frag.glsl");

    shaders["velocity_skinning"] = create_shader_program("scenes/shared_assets/shaders/velocity_skinning/shader.vert.glsl",
                                             "scenes/shared_assets/shaders/velocity_skinning/shader.frag.glsl");

    shaders["velocity_skinning_raw"] = create_shader_program("velocity_skinning_shader/shader.vert.glsl",
        "velocity_skinning_shader/shader.frag.glsl");

    //_scene.texture_white = create_texture_gpu(image_load_png("assets/spot/texture.png"));


    glUseProgram(shaders["mesh_shadow"]);
    GLint texLoc = glGetUniformLocation(shaders["mesh_shadow"], "texture_sampler");
    glUniform1i(texLoc, 0);
    texLoc = glGetUniformLocation(shaders["mesh_shadow"], "shadowMap");
    glUniform1i(texLoc, 1);
    glUseProgram(0);


    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0.0f,0.0f,0.0f};
    glEnable(GL_POLYGON_OFFSET_FILL);


    // Sphere used to display joints
    sphere_visual = mesh_primitive_sphere(1.0f);
    sphere_visual.shader = shader_mesh;

    frame = mesh_primitive_frame();
    frame.uniform.transform.scaling = 0.02f;
    frame.shader = shaders["mesh"];

    // Load initial model
    //load_sphere_data(skeleton, skinning, weight_flappy, character_visual, timer_skeleton, shader_mesh);
    //load_bending_cylinder_data(skeleton, skinning, weight_flappy, weight_squashy, character_visual, timer_skeleton, shader_mesh);
    gui_param.display_type = display_cylinder_bending;

    // Setup cursor
    painting_cursor = curve_primitve_circle(40, 1.0f, {0,0,0}, {0,0,1});
    painting_cursor.shader = shaders["curve"];
    painting_cursor.uniform.color = {0.7f, 0.7f, 0.7f};
    painting_cursor.uniform.transform.scaling = gui_param.painting.radius;


    pyramid_skeleton_visual = mesh_primitive_pyramid(0.25f,1.0f);
    pyramid_skeleton_visual.shader = shaders["mesh"];

    gui.show_frame_camera = false;

    quad = mesh_primitive_quad();

    size_t size_data_send_gpu = 0;
    size_t size_data_send_gpu_LBS_only = 0;

#ifdef COW

    {
        data_loaded data_load;
        data_load = load_spot_data(shaders["mesh"]);
        
        skeleton = data_load.skeleton;
        skinning.influence = data_load.skinning_rig;
        skinning.deformed = data_load.shape;
        skinning.rest_pose = data_load.shape.position;
        skinning.rest_pose_normal = data_load.shape.normal;
        //symmetrical_joint = data_load.symmetry;
        weight_flappy = data_load.weight_flappy;
        character_visual = data_load.shape;
        character_visual.shader = data_load.shader;
        character_visual.texture_id = data_load.texture_id;
        timer_skeleton.t_max = data_load.anim_time_max;

        int N_vertex = skinning.rest_pose.size();
        point_set_raw = skinning.rest_pose;
        normal_raw = skinning.rest_pose_normal;
        texture_uv_raw = skinning.deformed.texture_uv;
        floppy_weights_raw = weight_flappy;
        connectivity_raw = skinning.deformed.connectivity;


        //std::cout << skeleton.anim.size() << std::endl;
        skeleton_local_current_before_deformation = skeleton.rest_pose;
    }
#endif


    {
#ifdef CTHULHU
        //std::string const filename_point_set = "assets/armadillo/binary/point_set.dat";
        //std::string const filename_connectivity = "assets/armadillo/binary/connectivity.dat";
        //std::string const filename_size = "assets/armadillo/binary/size.txt";

        
        std::string const filename_point_set = "assets/cthulhu/binary/point_set.dat";
        std::string const filename_normal = "assets/cthulhu/binary/normal.dat";
        std::string const filename_floppy_weights = "assets/cthulhu/binary/floppy_weights";
        std::string const filename_connectivity = "assets/cthulhu/binary/connectivity.dat";
        std::string const filename_size = "assets/cthulhu/binary/size.txt";

        int N_vertex = 0;
        int N_connectivity = 0;
        {
            std::ifstream s(filename_size);
            s >> N_vertex >> N_connectivity;
            s.close();
        }
        
        std::cout << "Load structure (N_vertex="<<N_vertex<<", N_connectivity="<<N_connectivity<<") ..." << std::endl;
        point_set_raw.resize(N_vertex);
        normal_raw.resize(N_vertex);
        floppy_weights_raw.resize(N_vertex);
        connectivity_raw.resize(N_connectivity);
        texture_uv_raw.resize(N_vertex);

        {
            std::ifstream s(filename_point_set, std::ios::in | std::ios::binary);
            s.read( (char*)&point_set_raw[0].x, 3 * N_vertex * sizeof(float));
        }
        {
            std::ifstream s(filename_normal, std::ios::in | std::ios::binary);
            s.read((char*)&normal_raw[0][0], 3 * N_vertex * sizeof(float));
        }
        {
            std::ifstream s(filename_connectivity, std::ios::in | std::ios::binary);
            s.read((char*)&connectivity_raw[0][0], 3 * N_connectivity * sizeof(unsigned int));
        }

        {
            std::ifstream s(filename_floppy_weights, std::ios::in | std::ios::binary);
            s.read((char*)&floppy_weights_raw[0], N_vertex * sizeof(float));

            for (auto& w : floppy_weights_raw)
            {
                w = std::abs(2*w);
                w = std::min(w, 1.0f);
                w = 1 - w;
            }
        }
        std::cout << "\t OK" << std::endl;

        //for (int k = 0; k < 10; ++k)
        //{
        //    std::cout <<k<<" : "<< point_set_raw[k] << " ; " << floppy_weights_raw[k] << std::endl;
        //}
#endif

        std::cout << "Compute one ring ...";
        // build one ring
        {
            size_t const N = point_set_raw.size();
            auto const& connectivity = connectivity_raw;
            one_ring.clear();
            one_ring.resize(N);

            size_t const N_tri = connectivity_raw.size();
            for (size_t k_tri = 0; k_tri < N_tri; ++k_tri)
            {
                unsigned int const u0 = connectivity[k_tri][0];
                unsigned int const u1 = connectivity[k_tri][1];
                unsigned int const u2 = connectivity[k_tri][2];

                one_ring[u0].insert(u1); one_ring[u0].insert(u2);
                one_ring[u1].insert(u0); one_ring[u1].insert(u2);
                one_ring[u2].insert(u0); one_ring[u2].insert(u1);
            }
        }
        std::cout << "\tOK" << std::endl;

#ifdef CTHULHU
        std::cout << "Diffuse ...";
        floppy_weights_raw = diffuse_laplacien_weights(floppy_weights_raw, one_ring, 0.1, 25);
        std::cout << "\tOK"<< std::endl;

        

        




        //mat3 R = rotation_from_axis_angle_mat3({ 1,0,0 }, 3.14f / 2.0f);
        //for (auto& p : point_set_raw)
        //    p = R * p;
        //for (auto& n : normal_raw)
        //    n = R * n;

        std::cout << "Compute normals ..." << std::endl;
        vcl::normal(point_set_raw, connectivity_raw, normal_raw);
        std::cout << "\t OK" << std::endl;

        //{
        //    std::ofstream s("assets/cthulhu/binary/normal.dat", std::ios::out | std::ios::binary);
        //    s.write((char*)&normal_raw[0].x, 3 * N_vertex * sizeof(float));
        //}
#endif


        std::cout << "Send data to GPU ..." << std::endl;
        // Fill VBO for position
        glGenBuffers(1, &buffer_vertex_raw);
        glBindBuffer(GL_ARRAY_BUFFER, buffer_vertex_raw);
        glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(point_set_raw.size() * sizeof(GLfloat) * 3), &point_set_raw[0].x, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        size_data_send_gpu += GLsizeiptr(point_set_raw.size() * sizeof(GLfloat) * 3);
        size_data_send_gpu_LBS_only += GLsizeiptr(point_set_raw.size() * sizeof(GLfloat) * 3);

        // Fill VBO for normal
        glGenBuffers(1, &buffer_normal_raw);
        glBindBuffer(GL_ARRAY_BUFFER, buffer_normal_raw);
        glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(normal_raw.size() * sizeof(GLfloat) * 3), &normal_raw[0].x, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        size_data_send_gpu += GLsizeiptr(normal_raw.size() * sizeof(GLfloat) * 3);
        size_data_send_gpu_LBS_only += GLsizeiptr(normal_raw.size() * sizeof(GLfloat) * 3);

        // Fill VBO for uv
        glGenBuffers(1, &buffer_texture_uv_raw);
        glBindBuffer(GL_ARRAY_BUFFER, buffer_texture_uv_raw);
        glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(texture_uv_raw.size() * sizeof(GLfloat) * 2), &texture_uv_raw[0].x, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        size_data_send_gpu += GLsizeiptr(texture_uv_raw.size() * sizeof(GLfloat) * 2);
        size_data_send_gpu_LBS_only += GLsizeiptr(texture_uv_raw.size() * sizeof(GLfloat) * 2);


        // Fill Buffer connectivity
        glGenBuffers(1, &buffer_connectivity_raw);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer_connectivity_raw);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(connectivity_raw.size() * sizeof(GLuint) * 3), &connectivity_raw[0][0], GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        size_data_send_gpu += GLsizeiptr(connectivity_raw.size() * sizeof(GLuint) * 3);
        size_data_send_gpu_LBS_only += GLsizeiptr(connectivity_raw.size() * sizeof(GLuint) * 3);

        //number_triangles = static_cast<unsigned int>(mesh_cpu.connectivity.size());
        glGenVertexArrays(1, &vao_raw);
        glBindVertexArray(vao_raw);

        // position at layout 0
        glBindBuffer(GL_ARRAY_BUFFER, buffer_vertex_raw);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        // normals at layout 1
        glBindBuffer(GL_ARRAY_BUFFER, buffer_normal_raw);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        // texture at layout 2
        glBindBuffer(GL_ARRAY_BUFFER, buffer_texture_uv_raw);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        std::cout << "\t OK" << std::endl;


        //exit(0);
    }






    {
        std::cout << "Read skeleton  ..." << std::endl;




#ifdef COW




        int N_joint = skeleton.rest_pose.size();
        buffer<std::string> joint_name;
        for (int k = 0; k < N_joint; ++k)
        {
            joint_name.push_back(std::to_string(k));
        }
        //buffer<int> joint_parent = skeleton.connectivity;




        buffer<float> timing_anim; read_from_file("scene_web/anim.txt", timing_anim);

        
        buffer<vec4> rotation_anim; read_from_file("scene_web/rotation.txt", rotation_anim);
        buffer<quaternion> quaternion_anim;
        for (int k = 0; k < rotation_anim.size(); ++k)
        {
            quaternion_anim.push_back({ rotation_anim[k].x, rotation_anim[k].y, rotation_anim[k].z, rotation_anim[k].w });
        }

        buffer<vec3> translation_anim; read_from_file("scene_web/position.txt", translation_anim);

        
        int const Nt = timing_anim.size();
        skeleton.anim.resize(N_joint);
        for (int kb = 0; kb < N_joint; ++kb)
        {
            skeleton.anim[kb].resize(Nt);
            for (int kt = 0; kt < Nt; ++kt)
            {
                skeleton.anim[kb][kt].time = timing_anim[kt];
                skeleton.anim[kb][kt].geometry.r = quaternion_anim[kb + N_joint * kt];
                skeleton.anim[kb][kt].geometry.p = translation_anim[kb + N_joint * kt];
            }
        }
        timer_skeleton.t_min = 0.0;
        timer_skeleton.t_max = timing_anim[timing_anim.size() - 1];

#endif


        // RESET TO 0
        //int const Nt = timing_anim.size();
        //skeleton.anim.resize(N_joint);
        //for (int kb = 0; kb < N_joint; ++kb)
        //{
        //    skeleton.anim[kb].resize(Nt);
        //    for (int kt = 0; kt < Nt; ++kt)
        //    {
        //        skeleton.anim[kb][kt].time = (1.0f*kt) / Nt;// timing_anim[kt];
        //        skeleton.anim[kb][kt].geometry.r = quaternion_anim[kb+N_joint*0];
        //        skeleton.anim[kb][kt].geometry.p = translation_anim[kb + N_joint*0];
        //    }
        //}
        //
        //timer_skeleton.t_min = 0.0;
        //timer_skeleton.t_max = 1.0f;// timing_anim[timing_anim.size() - 1];
        



#ifdef CTHULHU


        std::string const dir = "assets/cthulhu/rig/";
        buffer<std::string> joint_name;      read_from_file(dir + "skeleton_joint_name", joint_name);
        buffer<int> joint_parent;            read_from_file(dir + "skeleton_connectivity_parent", joint_parent);
        buffer<vec3> joint_translation;      read_from_file(dir + "skeleton_rest_pose_translation", joint_translation);
        buffer<quaternion> joint_quaternion; read_from_file(dir + "skeleton_rest_pose_rotation", joint_quaternion);
        std::cout << "\tOK" << std::endl;

        std::cout << "Read weights  ..." << std::endl;
        buffer<buffer<int>> rig_joint;       read_from_file(dir + "rig_joint", rig_joint);
        buffer<buffer<float>> rig_weights;   read_from_file(dir + "rig_weights", rig_weights);
        std::cout << "\tOK" << std::endl;
        
        
        
        joint_name_raw = joint_name;

        

        int N_joint = joint_name.size();
        assert_vcl_no_msg(N_joint > 0);
        for (int k = 0; k < N_joint; ++k)
        {
            skeleton.rest_pose.push_back({ joint_translation[k], joint_quaternion[k] });
            skeleton.connectivity.push_back({ joint_parent[k], joint_name[k] });
        }
        skeleton.anim.resize(N_joint);
        for (int k = 0; k < N_joint; ++k)
        {
            skeleton.anim[k].resize(3);
            skeleton.anim[k][0].time = 0;
            skeleton.anim[k][1].time = 1;
            skeleton.anim[k][2].time = 2;

            skeleton.anim[k][0].geometry = skeleton.rest_pose[k];
            skeleton.anim[k][1].geometry = skeleton.rest_pose[k];
            skeleton.anim[k][2].geometry = skeleton.rest_pose[k];
        }

        //    data.skeleton.anim[9][0].geometry.r = quaternion::axis_angle({1,0,0},-3.14f/4.0f);
        //    data.skeleton.anim[9][1].geometry.r = quaternion::axis_angle({1,0,0},+3.14f/4.0f);
        //    data.skeleton.anim[9][2].geometry.r = data.skeleton.anim[9][0].geometry.r;

#endif

        skeleton_local_current_before_deformation = interpolate_skeleton_at_time(0, skeleton.anim, gui_param.interpolate);
        skeleton_local_current = skeleton_local_current_before_deformation;
        skeleton_local_interactive_deformation.resize(N_joint);

        skeleton_joint_speed.resize_clear(N_joint);
        skeleton_joint_rotation_speed.resize_clear(N_joint);
        skeleton_speed_per_joint.resize_clear(N_joint);
        skeleton_fake_speed_per_joint.resize_clear(N_joint);

        std::cout << "Read skeleton  OK" << std::endl;
        

        skeleton_rest_pose = local_to_global(skeleton.rest_pose, skeleton.connectivity);
        std::vector<mat4> skeleton_rest_to_gpu(skeleton_rest_pose.size());
        for (int k = 0; k<int(skeleton_rest_pose.size()); ++k) {
            const mat3& R = skeleton_rest_pose[k].r.matrix();
            const vec3& p = skeleton_rest_pose[k].p;
            const mat4 T = mat4::from_mat3_vec3(transpose(R), -transpose(R)*p);
            skeleton_rest_to_gpu[k] = T;
        }
        buffer_tbo_sk0 = skeleton_rest_to_gpu;
        
        
        std::cout << "Weights" << std::endl;
        //int N_vertex = point_set_raw.size();
        //skinning.influence.resize(N_vertex);
        //for (int k = 0; k< N_vertex; ++k)
        //{
        //    skinning.influence[k].resize(1);
        //    skinning.influence[k][0].joint = 0;
        //    skinning.influence[k][0].weight = 1.0f;
        //}
        
#ifdef CTHULHU
        //assert_vcl_no_msg(rig_joint.size() == rig_weights.size());
        skinning.influence.resize(point_set_raw.size());
        for (int k = 0; k< point_set_raw.size(); ++k)
        {
            int N_dependencies = rig_joint[k].size();
            skinning.influence[k].resize(N_dependencies);
            for (int kb = 0; kb < N_dependencies; ++kb)
            {
                skinning.influence[k][kb].joint = rig_joint[k][kb];
                skinning.influence[k][kb].weight = rig_weights[k][kb];
            }
        }
        std::cout << "\tOK" << std::endl;
#endif

#ifdef COW
        int N_vertex = skinning.influence.size();
        buffer<buffer<int>> rig_joint(N_vertex);
        buffer<buffer<float>> rig_weights(N_vertex);
        for (int k = 0; k < N_vertex; ++k)
        {
            int N_influence = skinning.influence[k].size();
            rig_joint[k].resize(N_influence);
            rig_weights[k].resize(N_influence);
            for (int ki = 0; ki < N_influence; ++ki)
            {
                int j = skinning.influence[k][ki].joint;
                float w = skinning.influence[k][ki].weight;
                rig_joint[k][ki] = j;
                rig_weights[k][ki] = w;
            }
        }
#endif

        std::cout << "Velocity skinning weights" << std::endl;
        vcl::buffer<vcl::buffer<int>> rig_velocity_skinning_joint;
        vcl::buffer<vcl::buffer<float>> rig_velocity_skinning_weights;

        {
            int const N_vertex = point_set_raw.size();
            rig_velocity_skinning_joint.resize_clear(N_vertex);
            rig_velocity_skinning_weights.resize_clear(N_vertex);

            for (int kv = 0; kv < N_vertex; ++kv)
            {
                auto const& influence = skinning.influence[kv];
                int const N_influence = influence.size();

                std::map<int, float> cumulative_weight_per_joint;

                for (int k_influence = 0; k_influence < N_influence; ++k_influence)
                {
                    int current_joint = influence[k_influence].joint;
                    float weight = influence[k_influence].weight;

                    while (current_joint != -1)
                    {
                        if (cumulative_weight_per_joint.find(current_joint) != cumulative_weight_per_joint.end())
                            cumulative_weight_per_joint[current_joint] += weight;
                        else
                            cumulative_weight_per_joint[current_joint] = weight;

                        current_joint = skeleton.connectivity[current_joint].parent;
                    }
                }

                for (auto const& it : cumulative_weight_per_joint)
                {
                    rig_velocity_skinning_joint[kv].push_back(it.first);
                    rig_velocity_skinning_weights[kv].push_back(it.second);
                }

            }

            //        for(int kv=0; kv<N_vertex; ++kv)
            //        {
            //            for(int k=0; k<rig_extended_to_ancestor_joint[kv].size(); ++k)
            //            {
            //                std::cout<<kv<<" - "<<rig_extended_to_ancestor_joint[kv][k]<<","<<rig_extended_to_ancestor_weight[kv][k]<<std::endl;
            //            }
            //            std::cout<<std::endl;
            //        }
        }

#ifdef CTHULHU
        // reduce to non fake bones
        {
            int const N_vertex = point_set_raw.size();
            vcl::buffer<vcl::buffer<int>> rig_velocity_skinning_joint2(N_vertex);
            vcl::buffer<vcl::buffer<float>> rig_velocity_skinning_weights2(N_vertex);

            for (int k = 0; k < N_vertex; ++k)
            {
                int const N_dep = rig_velocity_skinning_joint[k].size();
                for (int kj = 0; kj < N_dep; ++kj)
                {
                    int const j = rig_velocity_skinning_joint[k][kj];
                    std::string const name = joint_name[j];
                    if (name[0] != 'F')
                    {
                        rig_velocity_skinning_joint2[k].push_back(j);
                        rig_velocity_skinning_weights2[k].push_back(rig_velocity_skinning_weights[k][kj]);
                    }
                }
            }


            rig_velocity_skinning_joint = rig_velocity_skinning_joint2;
            rig_velocity_skinning_weights = rig_velocity_skinning_weights2;
        }
        

        std::cout<<"\tOK"<<std::endl;
#endif


        

        {
            // TBO for skeleton
            size_t const N_joint = skeleton.rest_pose.size();
            std::cout <<"Number of bones : "<< N_joint << std::endl;
            buffer_tbo_sk0.resize(N_joint);
            buffer_tbo_sk.resize(N_joint);
            buffer_tbo_angular_velocity.resize(N_joint);
            buffer_tbo_linear_velocity.resize(N_joint);

            /*
            std::vector<mat4> skeleton_rest_to_gpu(N_joint);
            for (int k = 0; k < N_joint; ++k) {
                mat4 T = mat4::from_mat3_vec3(skeleton.rest_pose[k].r.matrix(), skeleton.rest_pose[k].p);
                skeleton_rest_to_gpu[k] = T;
            }*/
            

            buffer_tbo_sk0 = skeleton_rest_to_gpu;

            glGenBuffers(1, &tbo_sk0);
            glGenBuffers(1, &tbo_sk);
            glGenBuffers(1, &tbo_angular_velocity);
            glGenBuffers(1, &tbo_linear_velocity);

            //std::cout << "---------------------------------------------" << std::endl;
            //std::cout << buffer_tbo_sk0 << std::endl;
            //std::cout << "---------------------------------------------" << std::endl;
            

            glBindBuffer(GL_TEXTURE_BUFFER, tbo_sk0);
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float) * buffer_tbo_sk0.size() * 16, &buffer_tbo_sk0[0][0], GL_STATIC_DRAW);
            glBindBuffer(GL_TEXTURE_BUFFER, tbo_sk);
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float) * buffer_tbo_sk.size() * 16, &buffer_tbo_sk[0][0], GL_STATIC_DRAW);
            glBindBuffer(GL_TEXTURE_BUFFER, tbo_angular_velocity);
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float) * buffer_tbo_angular_velocity.size() * 3, &buffer_tbo_angular_velocity[0].x, GL_STATIC_DRAW);
            glBindBuffer(GL_TEXTURE_BUFFER, tbo_linear_velocity);
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float)* buffer_tbo_linear_velocity.size() * 3, &buffer_tbo_linear_velocity[0].x, GL_STATIC_DRAW);
        }

        {
            glGenTextures(1, &texture_skinning_weights);
            glGenTextures(1, &texture_velocity_skinning_weights);

            glUseProgram(shaders["velocity_skinning_raw"]);
            glBindVertexArray(vao_raw); opengl_debug();



            glGenTextures(1, &tex_sk0);
            glGenTextures(1, &tex_sk);
            glGenTextures(1, &tex_angular_velocity);
            glGenTextures(1, &tex_linear_velocity);

            glActiveTexture(GL_TEXTURE2); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, tex_sk0); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, tbo_sk0); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "tbo_sk0", int(2)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glActiveTexture(GL_TEXTURE3); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, tex_sk); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, tbo_sk); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "tbo_sk", int(3)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glActiveTexture(GL_TEXTURE4); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, tex_angular_velocity); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, tbo_angular_velocity); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "tbo_angular_velocity", int(4)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glActiveTexture(GL_TEXTURE20); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, tex_linear_velocity); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, tbo_linear_velocity); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "tbo_linear_velocity", int(20)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);
        }

        {
            glGenBuffers(1, &buffer_floppy_weights_raw);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_floppy_weights_raw); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float)* floppy_weights_raw.size(), &floppy_weights_raw[0], GL_STATIC_DRAW); opengl_debug();


            size_data_send_gpu_LBS_only += sizeof(float) * floppy_weights_raw.size();

            glGenTextures(1, &texture_floppy_weights_raw);
            glActiveTexture(GL_TEXTURE5); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_floppy_weights_raw); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_floppy_weights_raw); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_floppy_weights_raw", int(5)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

        }

        {
            vcl::buffer<int> rig_joint_raw;
            vcl::buffer<float> rig_weight_raw;

            size_t const N_vertex = point_set_raw.size();
            vcl::buffer<int> rig_cumulative_index;
            vcl::buffer<int> rig_size;
            rig_cumulative_index.resize(N_vertex);
            rig_size.resize(N_vertex);
            rig_cumulative_index[0] = 0;
            for (int k = 0; k < N_vertex; ++k)
            {
                rig_size[k] = skinning.influence[k].size();
                if(k<N_vertex-1)
                    rig_cumulative_index[k + 1] = rig_cumulative_index[k] + rig_size[k];

                for (int kj = 0; kj < rig_joint[k].size(); ++kj)
                {
                    rig_joint_raw.push_back(rig_joint[k][kj]);
                    rig_weight_raw.push_back(rig_weights[k][kj]);
                }
            }

            //for (int k_v = 0; k_v < 9; ++k_v)
            //{
            //    std::cout << "Vertex : " << k_v << std::endl;
            //    int offset = rig_cumulative_index[k_v];
            //    int N_dep = rig_size[k_v];
            //    std::cout << "Offset:" << offset << " , N_dependency=" << N_dep << std::endl;
            //    for (int k_dep = 0; k_dep < N_dep; ++k_dep)
            //    {
            //        int const j = rig_joint_raw[offset + k_dep];
            //        float const w = rig_weight_raw[offset + k_dep];
            //        std::cout << "(" << j << "," << w << ") ";
            //    }
            //    std::cout << std::endl;
            //}

            
            glGenBuffers(1, &buffer_rig_joint);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_rig_joint); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(int)* rig_joint_raw.size(), &rig_joint_raw[0], GL_STATIC_DRAW); opengl_debug();
            size_data_send_gpu += sizeof(int) * rig_joint_raw.size();
            size_data_send_gpu_LBS_only += sizeof(int) * rig_joint_raw.size();
            glGenTextures(1, &texture_rig_joint);
            glActiveTexture(GL_TEXTURE6); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_rig_joint); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, buffer_rig_joint); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_rig_joint", int(6)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glGenBuffers(1, &buffer_rig_weight);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_rig_weight); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float)* rig_weight_raw.size(), &rig_weight_raw[0], GL_STATIC_DRAW); opengl_debug();
            size_data_send_gpu += sizeof(float) * rig_weight_raw.size();
            size_data_send_gpu_LBS_only += sizeof(float) * rig_weight_raw.size();
            glGenTextures(1, &texture_rig_weight);
            glActiveTexture(GL_TEXTURE7); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_rig_weight); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_rig_weight); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_rig_weight", int(7)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glGenBuffers(1, &buffer_rig_cumulative_index);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_rig_cumulative_index); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(int)* rig_cumulative_index.size(), &rig_cumulative_index[0], GL_STATIC_DRAW); opengl_debug();
            size_data_send_gpu += sizeof(int) * rig_cumulative_index.size();
            size_data_send_gpu_LBS_only += sizeof(int) * rig_cumulative_index.size();
            glGenTextures(1, &texture_rig_cumulative_index);
            glActiveTexture(GL_TEXTURE8); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_rig_cumulative_index); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, buffer_rig_cumulative_index); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_rig_cumulative_index", int(8)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glGenBuffers(1, &buffer_rig_size);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_rig_size); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(int)* rig_size.size(), &rig_size[0], GL_STATIC_DRAW); opengl_debug();
            size_data_send_gpu += sizeof(int) * rig_size.size();
            size_data_send_gpu_LBS_only += sizeof(int) * rig_size.size();
            glGenTextures(1, &texture_rig_size);
            glActiveTexture(GL_TEXTURE9); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_rig_size); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, buffer_rig_size); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_rig_size", int(9)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);
        }

        std::cout << "VS Rig Index" << std::endl;
        {
            vcl::buffer<int> vs_rig_joint_raw;
            vcl::buffer<float> vs_rig_weight_raw;

            size_t const N_vertex = point_set_raw.size();
            vcl::buffer<int> vs_rig_cumulative_index;
            vcl::buffer<int> vs_rig_size;
            vs_rig_cumulative_index.resize(N_vertex);
            vs_rig_size.resize(N_vertex);
            vs_rig_cumulative_index[0] = 0;
            for (int k = 0; k < N_vertex; ++k)
            {
                vs_rig_size[k] = rig_velocity_skinning_joint[k].size();
                if (k < N_vertex - 1)
                    vs_rig_cumulative_index[k + 1] = vs_rig_cumulative_index[k] + vs_rig_size[k];

                for (int kj = 0; kj < vs_rig_size[k]; ++kj)
                {
                    vs_rig_joint_raw.push_back(rig_velocity_skinning_joint[k][kj]);
                    vs_rig_weight_raw.push_back(rig_velocity_skinning_weights[k][kj]);
                }
            }

            /*
            for (int k_v = 0; k_v < 9; ++k_v)
            {
                std::cout << "Vertex : " << k_v << std::endl;
                int offset = vs_rig_cumulative_index[k_v];
                int N_dep = vs_rig_size[k_v];
                std::cout << "Offset:" << offset << " , N_dependency=" << N_dep << std::endl;
                for (int k_dep = 0; k_dep < N_dep; ++k_dep)
                {
                    int const j = vs_rig_joint_raw[offset + k_dep];
                    float const w = vs_rig_weight_raw[offset + k_dep];
                    std::cout << "(" << j << "," << w << ") ";
                }
                std::cout << std::endl;
            }
            */






            glGenBuffers(1, &buffer_vs_rig_joint);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_vs_rig_joint); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(int)* vs_rig_joint_raw.size(), &vs_rig_joint_raw[0], GL_STATIC_DRAW); opengl_debug();
            size_data_send_gpu += sizeof(int) * vs_rig_joint_raw.size();
            glGenTextures(1, &texture_vs_rig_joint);
            glActiveTexture(GL_TEXTURE10); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_vs_rig_joint); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, buffer_vs_rig_joint); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_vs_rig_joint", int(10)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glGenBuffers(1, &buffer_vs_rig_weight);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_vs_rig_weight); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float)* vs_rig_weight_raw.size(), &vs_rig_weight_raw[0], GL_STATIC_DRAW); opengl_debug();
            size_data_send_gpu += sizeof(float) * vs_rig_weight_raw.size();
            glGenTextures(1, &texture_vs_rig_weight);
            glActiveTexture(GL_TEXTURE11); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_vs_rig_weight); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_vs_rig_weight); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_vs_rig_weight", int(11)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glGenBuffers(1, &buffer_vs_rig_cumulative_index);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_vs_rig_cumulative_index); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(int)* vs_rig_cumulative_index.size(), &vs_rig_cumulative_index[0], GL_STATIC_DRAW); opengl_debug();
            size_data_send_gpu += sizeof(int) * vs_rig_cumulative_index.size();
            glGenTextures(1, &texture_vs_rig_cumulative_index);
            glActiveTexture(GL_TEXTURE12); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_vs_rig_cumulative_index); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, buffer_vs_rig_cumulative_index); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_vs_rig_cumulative_index", int(12)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

            glGenBuffers(1, &buffer_vs_rig_size);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_vs_rig_size); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(int)* vs_rig_size.size(), &vs_rig_size[0], GL_STATIC_DRAW); opengl_debug();
            size_data_send_gpu += sizeof(int) * vs_rig_size.size();
            glGenTextures(1, &texture_vs_rig_size);
            glActiveTexture(GL_TEXTURE13); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_vs_rig_size); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, buffer_vs_rig_size); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_vs_rig_size", int(13)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);
        }


        {
            glActiveTexture(GL_TEXTURE14); opengl_debug();
            glBindTexture(GL_TEXTURE_2D, character_visual.texture_id); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_character", int(14)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);
        }

#ifdef COW
        {
            N_cow = 500;
            float const scale = 40.0f;
            position_cow_raw.push_back({ 0,0,0.0f });
            angle_cow_raw.push_back(0.0f);
            for (int k = 0; k < N_cow; ++k)
            {
                float x = vcl::rand_interval(-scale, scale);
                float y = vcl::rand_interval(-scale, scale);
                vec2 const p = { x,y };
                bool to_add = true;
                for (int k2 = 0; to_add==true && k2 < position_cow_raw.size(); ++k2)
                {
                    vec2 const& p2 = { position_cow_raw[k2].x, position_cow_raw[k2].y };
                    if (norm(p - p2) < 3.0f)
                        to_add = false;
                }
                if (to_add == true)
                {
                    position_cow_raw.push_back({ x,y,0.0f });

                    float angle = vcl::rand_interval(0, 2 * 3.14f);
                    angle_cow_raw.push_back(angle);
                }
            }
            N_cow = angle_cow_raw.size();
            std::cout << "===============================================" << std::endl;
            std::cout << "N_cow = "<<N_cow << std::endl;
            std::cout << "N_vertex = " << N_cow*N_vertex << std::endl;
            std::cout << "N_triangle = " << N_cow * skinning.deformed.connectivity.size() << std::endl;

            glGenBuffers(1, &buffer_position_cow_raw);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_position_cow_raw); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float)* position_cow_raw.size() * 3, &position_cow_raw[0], GL_STATIC_DRAW); opengl_debug();
            glGenTextures(1, &texture_position_cow_raw);
            glActiveTexture(GL_TEXTURE15); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_position_cow_raw); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_position_cow_raw); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_position_cow", int(15)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);
            size_data_send_gpu += sizeof(float) * position_cow_raw.size() * 3;
            size_data_send_gpu_LBS_only += sizeof(float) * position_cow_raw.size() * 3;

            glGenBuffers(1, &buffer_angle_cow_raw);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_angle_cow_raw); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float)* angle_cow_raw.size(), &angle_cow_raw[0], GL_STATIC_DRAW); opengl_debug();
            glGenTextures(1, &texture_angle_cow_raw);
            glActiveTexture(GL_TEXTURE16); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_angle_cow_raw); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_angle_cow_raw); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_angle_cow", int(16)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);
            size_data_send_gpu += sizeof(float) * angle_cow_raw.size();
            size_data_send_gpu_LBS_only += sizeof(float) * angle_cow_raw.size();
        }
#endif
        // Center of mass
        {

            vcl::buffer<std::set<int> > triangle_set_depending_on_joint(N_joint);
            int const N_triangle = skinning.deformed.connectivity.size();
            for (int k_triangle = 0; k_triangle < N_triangle; ++k_triangle)
            {
                uint3 const& tri = skinning.deformed.connectivity[k_triangle];
                unsigned int a = tri[0];
                unsigned int b = tri[1];
                unsigned int c = tri[2];

                buffer<int> const& joints_a = rig_velocity_skinning_joint[a];
                buffer<int> const& joints_b = rig_velocity_skinning_joint[b];
                buffer<int> const& joints_c = rig_velocity_skinning_joint[c];

                for (int j : joints_a)
                    triangle_set_depending_on_joint[j].insert(k_triangle);
                for (int j : joints_b)
                    triangle_set_depending_on_joint[j].insert(k_triangle);
                for (int j : joints_c)
                    triangle_set_depending_on_joint[j].insert(k_triangle);
            }

            triangle_depending_on_joint.resize_clear(N_joint);
            for (int kj = 0; kj < N_joint; ++kj)
            {
                for (int t : triangle_set_depending_on_joint[kj])
                    triangle_depending_on_joint[kj].push_back(t);
            }


            triangle_area.resize_clear(N_triangle);
            triangle_center.resize_clear(N_triangle);


            center_of_mass_per_joint.resize_clear(N_joint);
            center_of_mass_per_joint_manual_offset.resize_clear(N_joint);

            buffer<float> barycenter_normalization;
            barycenter_normalization.resize_clear(N_joint);
            barycenter_normalization.fill(0.0);
            center_of_mass_per_joint.fill(vec3(0, 0, 0));
            

            for (size_t k_triangle = 0; k_triangle < N_triangle; ++k_triangle)
            {
                uint3 const& tri = skinning.deformed.connectivity[k_triangle];
                unsigned int const u0 = tri[0];
                unsigned int const u1 = tri[1];
                unsigned int const u2 = tri[2];

                vec3 const& p0 = point_set_raw[u0];
                vec3 const& p1 = point_set_raw[u1];
                vec3 const& p2 = point_set_raw[u2];

                triangle_area[k_triangle] = 0.5f * norm(cross(p1 - p0, p2 - p0));
                triangle_center[k_triangle] = (p0 + p1 + p2) / 3.0f;

                buffer<int> const& joints_a = rig_velocity_skinning_joint[u0];
                buffer<int> const& joints_b = rig_velocity_skinning_joint[u1];
                buffer<int> const& joints_c = rig_velocity_skinning_joint[u2];

                for (int kj = 0; kj < joints_a.size(); ++kj) {
                    float w = rig_velocity_skinning_weights[u0][kj];
                    center_of_mass_per_joint[joints_a[kj]] += w*triangle_area[k_triangle] * triangle_center[k_triangle];
                    barycenter_normalization[joints_a[kj]] += w*triangle_area[k_triangle];
                }

                for (int kj = 0; kj < joints_b.size(); ++kj) {
                    float w = rig_velocity_skinning_weights[u1][kj];
                    center_of_mass_per_joint[joints_b[kj]] += w * triangle_area[k_triangle] * triangle_center[k_triangle];
                    barycenter_normalization[joints_b[kj]] += w * triangle_area[k_triangle];
                }
                for (int kj = 0; kj < joints_c.size(); ++kj) {
                    float w = rig_velocity_skinning_weights[u2][kj];
                    center_of_mass_per_joint[joints_c[kj]] += w * triangle_area[k_triangle] * triangle_center[k_triangle];
                    barycenter_normalization[joints_c[kj]] += w * triangle_area[k_triangle];
                }

            }

            for (int k = 0; k < center_of_mass_per_joint.size(); ++k)
            {
                if( barycenter_normalization[k]>1e-4f )
                    center_of_mass_per_joint[k] = center_of_mass_per_joint[k] / barycenter_normalization[k];
            }



            glGenBuffers(1, &buffer_center_of_mass);
            glBindBuffer(GL_TEXTURE_BUFFER, buffer_center_of_mass); opengl_debug();
            glBufferData(GL_TEXTURE_BUFFER, sizeof(float) * center_of_mass_per_joint.size()*3, &center_of_mass_per_joint[0], GL_STATIC_DRAW); opengl_debug();
            size_data_send_gpu += sizeof(float) * center_of_mass_per_joint.size() * 3;
            glGenTextures(1, &texture_center_of_mass);
            glActiveTexture(GL_TEXTURE19); opengl_debug();
            glBindTexture(GL_TEXTURE_BUFFER, texture_center_of_mass); opengl_debug();
            glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_center_of_mass); opengl_debug();
            uniform(shaders["velocity_skinning_raw"], "texture_center_of_mass", int(19)); opengl_debug();
            glBindBuffer(GL_TEXTURE_BUFFER, 0);

        }


#ifdef COW
        {
            int N_time = Nt;
            buffer<mat4> T_to_gpu_anim(N_joint* N_time);
            buffer<vec3> angular_velocity_to_gpu_anim(N_joint* N_time);
            buffer<vec3> linear_velocity_to_gpu_anim(N_joint* N_time);



            buffer<buffer<vec3>> skeleton_position(N_joint);
            buffer<buffer<quaternion>> skeleton_rotation(N_joint);
            for (int k_time = 0; k_time < N_time; ++k_time)
            {
                
                float const t = skeleton.anim[0][k_time].time;
                
                interpolate_skeleton_at_time_with_constraints(skeleton_local_current,
                    t,
                    skeleton.anim,
                    gui_param.interpolate,
                    record_joint_fixed);
                skeleton_current = local_to_global(skeleton_local_current, skeleton.connectivity);
                for (int k = 0; k<int(skeleton_current.size()); ++k) {
                    skeleton_joint_speed[k].add(skeleton_local_current[k].p, t);
                    skeleton_joint_rotation_speed[k].add(skeleton_local_current[k].r, t);

                    mat3 R_parent = mat3::identity();
                    if (k > 0)
                        R_parent = skeleton_current[skeleton.connectivity[k].parent].r.matrix();

                    skeleton_speed_per_joint[k].center = skeleton_current[k].p;
                    skeleton_speed_per_joint[k].linear_speed = R_parent * skeleton_joint_speed[k].avg_speed;
                    skeleton_speed_per_joint[k].angular_speed = R_parent * skeleton_joint_rotation_speed[k].avg_rotation_speed;

                    angular_velocity_to_gpu_anim[k + N_joint * k_time] = skeleton_speed_per_joint[k].angular_speed;
                    linear_velocity_to_gpu_anim[k + N_joint * k_time] = skeleton_speed_per_joint[k].linear_speed;
                }



                for (size_t kj = 0; kj < N_joint; ++kj)
                {
                    skeleton_position[kj].push_back(skeleton_current[kj].p);
                    skeleton_rotation[kj].push_back(skeleton_current[kj].r);
                }


            }

            {
                for (int k_time = 0; k_time < N_time; ++k_time)
                {
                    for (size_t kj = 0; kj < N_joint; ++kj)
                    {
                        const mat3& R = skeleton_rotation[kj][k_time].matrix();
                        const vec3& p = skeleton_position[kj][k_time];
                        const mat4 T = mat4::from_mat3_vec3(R, p);

                        const mat3& R0 = skeleton_rest_pose[kj].r.matrix();
                        const vec3& p0 = skeleton_rest_pose[kj].p;
                        const mat4 T0_inv = mat4::from_mat3_vec3(transpose(R0), -transpose(R0) * p0);

                        const mat4 TT = T;// *T0_inv;
                        T_to_gpu_anim[kj + N_joint * k_time] = TT;
                    }
                }


                //std::cout << "@@---------------------------------------------" << std::endl;
                //for (int k = 0; k < N_joint; ++k)
                //{
                //    std::cout << "[" << k << "]" << std::endl;
                //    std::cout << T_to_gpu_anim[k] << std::endl << skeleton_rest_to_gpu[k] << std::endl << std::endl << std::endl;
                //}                
                //std::cout << "@@---------------------------------------------" << std::endl;



                glGenBuffers(1, &buffer_T_anim);
                glBindBuffer(GL_TEXTURE_BUFFER, buffer_T_anim); opengl_debug();
                glBufferData(GL_TEXTURE_BUFFER, sizeof(float)* T_to_gpu_anim.size()*16, &T_to_gpu_anim[0][0], GL_STATIC_DRAW); opengl_debug();
                size_data_send_gpu += sizeof(float) * T_to_gpu_anim.size() * 16;
                size_data_send_gpu_LBS_only += sizeof(float) * T_to_gpu_anim.size() * 16;
                glGenTextures(1, &texture_T_anim);
                glActiveTexture(GL_TEXTURE17); opengl_debug();
                glBindTexture(GL_TEXTURE_BUFFER, texture_T_anim); opengl_debug();
                glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_T_anim); opengl_debug();
                uniform(shaders["velocity_skinning_raw"], "texture_T_anim", int(17)); opengl_debug();
                glBindBuffer(GL_TEXTURE_BUFFER, 0);



                glGenBuffers(1, &buffer_angular_velocity_anim);
                glBindBuffer(GL_TEXTURE_BUFFER, buffer_angular_velocity_anim); opengl_debug();
                glBufferData(GL_TEXTURE_BUFFER, sizeof(float)* angular_velocity_to_gpu_anim.size() * 3, &angular_velocity_to_gpu_anim[0].x, GL_STATIC_DRAW); opengl_debug();
                size_data_send_gpu += sizeof(float) * angular_velocity_to_gpu_anim.size() * 3;
                glGenTextures(1, &texture_angular_velocity_anim);
                glActiveTexture(GL_TEXTURE18); opengl_debug();
                glBindTexture(GL_TEXTURE_BUFFER, texture_angular_velocity_anim); opengl_debug();
                glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_angular_velocity_anim); opengl_debug();
                uniform(shaders["velocity_skinning_raw"], "texture_angular_velocity_anim", int(18)); opengl_debug();
                glBindBuffer(GL_TEXTURE_BUFFER, 0);

                glGenBuffers(1, &buffer_linear_velocity_anim);
                glBindBuffer(GL_TEXTURE_BUFFER, buffer_linear_velocity_anim); opengl_debug();
                glBufferData(GL_TEXTURE_BUFFER, sizeof(float) * linear_velocity_to_gpu_anim.size() * 3, &linear_velocity_to_gpu_anim[0].x, GL_STATIC_DRAW); opengl_debug();
                size_data_send_gpu += sizeof(float) * linear_velocity_to_gpu_anim.size() * 3;
                glGenTextures(1, &texture_linear_velocity_anim);
                glActiveTexture(GL_TEXTURE21); opengl_debug();
                glBindTexture(GL_TEXTURE_BUFFER, texture_linear_velocity_anim); opengl_debug();
                glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, buffer_linear_velocity_anim); opengl_debug();
                uniform(shaders["velocity_skinning_raw"], "texture_linear_velocity_anim", int(21)); opengl_debug();
                glBindBuffer(GL_TEXTURE_BUFFER, 0);

            }
        }
#endif
        std::cout << "Data sent to GPU: " << size_data_send_gpu / 1024 / 1024.0 << " Mo" << std::endl;
        std::cout << "(LBS only): " << size_data_send_gpu_LBS_only / 1024 / 1024.0 << " Mo" << std::endl;



        {
            float scale = 200.0f;
            float z = -0.73f;
            ground = mesh_primitive_quad({ -scale,-scale,z }, { -scale,scale,z }, { scale,scale,z }, { scale,-scale,z });
            ground.shader = shaders["mesh"];
        }

        
    }


    timer_recording.stop();
    record_dt = 0.03f;

    arrow.init(shaders["mesh"]);
    arrow_line.init(shaders["mesh"]);

}


void scene_model::update_center_of_mass()
{

    size_t const N_triangle = skinning.deformed.connectivity.size();
    for(size_t k_triangle=0; k_triangle<N_triangle; ++k_triangle)
    {
        uint3 const& tri = skinning.deformed.connectivity[k_triangle];
        unsigned int const u0 = tri[0];
        unsigned int const u1 = tri[1];
        unsigned int const u2 = tri[2];

        vec3 const& p0 = point_set_raw[u0];
        vec3 const& p1 = point_set_raw[u1];
        vec3 const& p2 = point_set_raw[u2];

        triangle_area[k_triangle] = 0.5f * norm( cross(p1-p0, p2-p0) );
        triangle_center[k_triangle] = (p0+p1+p2)/3.0f;
    }

    size_t const N_joint = skinning_weights_per_joint_per_vertex.size();
    for(size_t k_joint=0; k_joint<N_joint; ++k_joint)
    {
        size_t N_triangle_dependency = triangle_depending_on_joint.data[k_joint].size();
        vec3 bar = {0,0,0};
        float counter = 0.0f;
        for(size_t k_triangle=0; k_triangle<N_triangle_dependency; ++k_triangle)
        {
            int k_tri = triangle_depending_on_joint.data[k_joint][k_triangle];
            uint3 const& tri = skinning.deformed.connectivity.data[k_tri];

            unsigned int const u0 = tri[0];
            unsigned int const u1 = tri[1];
            unsigned int const u2 = tri[2];

            float const w0 = skinning_weights_per_joint_per_vertex.data[k_joint].data[u0];
            float const w1 = skinning_weights_per_joint_per_vertex.data[k_joint].data[u1];
            float const w2 = skinning_weights_per_joint_per_vertex.data[k_joint].data[u2];

            bar += (w0+w1+w2) * triangle_area.data[k_tri] * triangle_center.data[k_tri];
            counter += (w0+w1+w2) * triangle_area.data[k_tri];
        }
        if(counter>1e-6f)
            bar /= counter;
        center_of_mass_per_joint.data[k_joint] = bar;
    }

}

void scene_model::generate_fake_speed()
{
    if(picking.is_selected)
    {

        bool is_translation = (gui_param.type_deformation==0 || picking.selected_joint==0);

        if(is_translation){
            int const joint = picking.selected_joint;
            vec3 const translation = picking.p_current - picking.p_clicked;
            skeleton_fake_speed_per_joint[joint].linear_speed = 2*translation;
        }
        else
        {
            int const joint = skeleton.connectivity[picking.selected_joint].parent;
            vec3 const& p_joint = skeleton_current[joint].p;
            vec3 const& p0 = picking.p_clicked;
            vec3 const& p1 = picking.p_current;

            if(picking.click_button==left) // rotate in plane
            {
                vec3 const u_ref = normalize(p0-p_joint);
                vec3 const u_objective = normalize(p1-p_joint);
                vec3 const axis = normalize(cross(u_ref, u_objective));
                float const angle = std::acos(dot(u_ref,u_objective));
                //quaternion const q = quaternion::axis_angle(axis,angle);
                skeleton_fake_speed_per_joint[joint].angular_speed = 2*axis*angle;
            }
            if(picking.click_button==right) // twist
            {
                vec3 const axis = normalize(p0 - p_joint);
                vec2 const sj = scene.camera.project_position(p_joint);
                vec2 const s0 = scene.camera.project_position(p0);
                vec2 const s1 = scene.camera.project_position(p1);

                vec2 const a = normalize(s0-sj);
                vec2 const b = s1-sj;
                float const angle = -2*3.14f*det(a,b);

                skeleton_fake_speed_per_joint[joint].angular_speed = 2*axis*angle;
            }

        }

    }
}

void scene_model::adapt_skeleton_interactive()
{

    if(picking.is_selected)
    {

        int const parent = skeleton.connectivity[picking.selected_joint].parent;

        quaternion q_parent;
        vec3 p_parent = skeleton_current[0].p;

        bool is_translation = (gui_param.type_deformation==0 || picking.selected_joint==0);

        // Displacement is applied to the current joint
        if(is_translation){
            if(parent>-1) {
                q_parent = skeleton_current[ parent ].r;
            }
            vec3 const translation = picking.p_current - picking.p_clicked;
            skeleton_local_interactive_deformation[picking.selected_joint].p = conjugate(q_parent).apply(translation);
        }

        // Rotation is applied to the parent joint
        if(!is_translation)
        {
            if(parent>-1) {

                p_parent = skeleton_current[ parent ].p;

                int const grand_parent = skeleton.connectivity[parent].parent;
                quaternion q_grand_parent = {0,0,0,1};
                if(grand_parent>-1)
                    q_grand_parent = skeleton_current[grand_parent].r;

                quaternion q;
                if(picking.click_button==left) // rotate in plane
                {
                    vec3 const u_ref = normalize(picking.p_clicked - p_parent);
                    vec3 const u_objective = normalize(picking.p_current - p_parent);

                    const float angle = std::acos(dot(u_ref,u_objective));
                    const vec3 axis = normalize(cross(u_ref, u_objective));
                    q = quaternion::axis_angle(axis,angle);
                }
                if(picking.click_button==right) // twist
                {
                    vec3 const axis = normalize(picking.p_clicked - p_parent);
                    vec2 const p0 = scene.camera.project_position(p_parent);
                    vec2 const p1 = scene.camera.project_position(picking.p_clicked);
                    vec2 const p  = scene.camera.project_position(picking.p_current);

                    vec2 const a = normalize(p1-p0);
                    vec2 const b = p-p0;
                    float const angle = -2*3.14f*det(a,b);
                    q = quaternion::axis_angle(axis,angle);
                }


                skeleton_local_interactive_deformation[parent].r =  conjugate(q_grand_parent) * q * q_grand_parent;;
            }
        }


    }

}



void scene_model::apply_deformation_on_skeleton()
{
    int const parent = skeleton.connectivity[picking.selected_joint].parent;




    // apply deformation on all time steps
    int const N_time = skeleton.anim[picking.selected_joint].size();
    for(int k_time=0; k_time<N_time; ++k_time)
    {
        skeleton.anim[picking.selected_joint][k_time].geometry.p += skeleton_local_interactive_deformation[picking.selected_joint].p;

        if(parent>-1){
            skeleton.anim[parent][k_time].geometry.r = skeleton_local_interactive_deformation[parent].r * skeleton.anim[parent][k_time].geometry.r;

            if(gui_param.symmetry)
            {
                if(symmetrical_joint.find(parent)!=symmetrical_joint.end())
                {
                    int const js = symmetrical_joint[parent];
                    skeleton.anim[js][k_time].geometry.r = skeleton_local_interactive_deformation[js].r * skeleton.anim[js][k_time].geometry.r;
                }
            }
        }
    }



    for(int k=0; k<int(skeleton_local_interactive_deformation.size()); ++k)
    {
        skeleton_local_interactive_deformation[k].p = {0,0,0};
        skeleton_local_interactive_deformation[k].r = {0,0,0,1};
    }

    //    skeleton_local_interactive_deformation[picking.selected_joint].p = {0,0,0};
    //    if(parent>-1)
    //        skeleton_local_interactive_deformation[parent].r = {0,0,0,1};
}

template <typename T, typename F>
int run_picking_selection_generic(scene_structure& scene, GLFWwindow* window, T const& buffer, F const& function_picker)
{
    int index_selected = -1;

    // Create the 3D ray passing by the selected point on the screen
    const vec2 cursor = glfw_cursor_coordinates_window(window);
    const ray r = picking_ray(scene.camera, cursor);

    bool is_selected = false;
    float distance_min = 0.0f;
    const int N = buffer.size();
    for(int k=0; k<N; ++k)
    {
        const vec3& c = function_picker(buffer, k);
        const picking_info info = ray_intersect_sphere(r, c, 0.04f);

        if( info.picking_valid ) // the ray intersects a sphere
        {
            const float distance = norm(info.intersection-r.p); // get the closest intersection
            if( is_selected==false || distance<distance_min ){
                is_selected = true;
                distance_min = distance;

                index_selected = k;
            }
        }
    }

    return index_selected;
}

int run_picking_selection_point_set(scene_structure& scene, GLFWwindow* window, buffer<vec3> const& point_set)
{

    return run_picking_selection_generic(scene, window, point_set, [](buffer<vec3> const& t,int k){return t[k];} );
}

int run_picking_selection_skeleton(scene_structure& scene, GLFWwindow* window, buffer<joint_geometry> const& skeleton_current)
{
    return run_picking_selection_generic(scene, window, skeleton_current, [](buffer<joint_geometry> const& t,int k){return t[k].p;} );
}


void convert_weight_to_color(buffer<float> const& value, buffer<vec4>& color)
{
    assert_vcl(value.size()==color.size(), "Incorrect size");
    size_t const N = value.size();
    for(size_t k=0; k<N; ++k)
        color[k] = { 1-std::max(value[k],0.0f), 1+std::min(value[k],0.0f), 1, 0 };
}

void scene_model::update_painted_color()
{
    switch(gui_param.painting.display_weights)
    {
    case 0:
        skinning.deformed.color.fill({1,1,1,0});
        break;
    case 1:
        convert_weight_to_color(weight_flappy, skinning.deformed.color);
        break;
    case 2:
        convert_weight_to_color(weight_squashy, skinning.deformed.color);
        break;
    }
    character_visual.update_color(skinning.deformed.color);
}

void scene_model::mouse_scroll(scene_structure& , GLFWwindow* window, float , float y_offset)
{
    // Increase/decrease falloff distance when scrolling the mouse
    if(gui_param.painting.activated)
    {
        const bool key_shift = glfw_key_shift_pressed(window);

        if(key_shift)
            gui_param.painting.radius = std::max(gui_param.painting.radius + y_offset*0.01f, 1e-6f);
        else
            gui_param.painting.threshold_percentage = std::min(std::max(gui_param.painting.threshold_percentage + y_offset*0.01f, 0.0f),1.0f);

    }
}

void scene_model::paint_vertices_around_picked_point()
{
    assert_vcl_no_msg(picking.painting_selected_vertex<int(skinning.deformed.position.size()));
    vec3 const& p_picked = skinning.deformed.position[picking.painting_selected_vertex];

    mesh& m = skinning.deformed;
    buffer<vec3>& vertices = m.position;
    size_t const N = vertices.size();
    float const threshold = gui_param.painting.radius * gui_param.painting.threshold_percentage;
    for(size_t k=0; k<N; ++k)
    {
        vec3 const& p = vertices[k];
        float const d = norm(p-p_picked);


        if( d < gui_param.painting.radius ) {
            float const current = weight_flappy[k];
            float const target = gui_param.painting.value;
            float const r      = gui_param.painting.radius;

            if(d<threshold)
            {
                weight_flappy[k] = target;
            }
            else
            {
                float alpha = (d-threshold)/(r-threshold);
                alpha = std::exp(-alpha*alpha*6);
                weight_flappy[k] = (1-alpha) * current + alpha * target;
            }

        }

    }

    update_painted_color();
}

void scene_model::mouse_move(scene_structure& scene_arg, GLFWwindow* window)
{
    const bool key_shift = glfw_key_shift_pressed(window);
    const bool mouse_click_left = glfw_mouse_pressed_left(window);


    if(gui_param.painting.activated && key_shift)
    {
        picking.painting_selected_vertex = run_picking_selection_point_set(scene_arg, window, skinning.deformed.position);
        if(picking.painting_selected_vertex>-1 && mouse_click_left)
            paint_vertices_around_picked_point();
    }


    if(!gui_param.painting.activated)
    {

        // Hovering
        if(!picking.is_selected && key_shift){
            picking.joint_hover = run_picking_selection_skeleton(scene_arg, window, skeleton_current);
        }


        // Displacement
        if(picking.is_selected || picking.picked_center_of_mass)
        {
            const vec2 cursor = glfw_cursor_coordinates_window(window);

            // Get vector orthogonal to camera orientation
            const mat4 M = scene.camera.camera_matrix();
            const vec3 n = {M(0,2),M(1,2),M(2,2)};

            const ray r = picking_ray(scene.camera, cursor);
            const picking_info info = ray_intersect_plane(r, n, picking.p_clicked);

            assert_vcl_no_msg(info.picking_valid);

            picking.p_current = info.intersection;

            if(picking.picked_center_of_mass)
            {
                int joint = picking.selected_joint_true;
                vec3 const& com = center_of_mass_per_joint[joint];
                quaternion q = skeleton_current[joint].r;

                center_of_mass_per_joint_manual_offset[joint] = conjugate(q).apply(picking.p_current-com);
            }
            else
            {
                if(!gui_param.fake_speed)
                    adapt_skeleton_interactive();
                else
                    generate_fake_speed();
            }


        }



    }



}

vcl::vec3 scene_model::position_center_of_mass(int joint)
{
    vec3 const& com_true = center_of_mass_per_joint[joint];
    vec3 const& offset = center_of_mass_per_joint_manual_offset[joint];
    quaternion const& q = skeleton_current[joint].r;

    return com_true + q.apply(offset);
}

void scene_model::mouse_click(scene_structure& scene_arg, GLFWwindow* window, int button, int action, int )
{
    // Check that the mouse is clicked (drag and drop)
    const bool mouse_click_left    = (button==GLFW_MOUSE_BUTTON_LEFT && action==GLFW_PRESS);
    const bool mouse_release_left  = (button==GLFW_MOUSE_BUTTON_LEFT && action==GLFW_RELEASE);
    const bool mouse_click_right    = (button==GLFW_MOUSE_BUTTON_RIGHT && action==GLFW_PRESS);
    const bool mouse_release_right  = (button==GLFW_MOUSE_BUTTON_RIGHT && action==GLFW_RELEASE);
    const bool key_shift = glfw_key_shift_pressed(window);
    const bool mouse_click   = mouse_click_left || mouse_click_right;
    const bool mouse_release = mouse_release_left || mouse_release_right;

    if(mouse_click_left) picking.click_button = left;
    if(mouse_click_right)picking.click_button = right;
    if(mouse_release)    picking.click_button = none;

    if(gui_param.painting.activated && key_shift)
    {
        picking.painting_selected_vertex = run_picking_selection_point_set(scene_arg, window, skinning.deformed.position);
        if(picking.painting_selected_vertex>-1 && mouse_click_left)
            paint_vertices_around_picked_point();
    }

    // Selection
    if(!gui_param.painting.activated){
        if(key_shift && mouse_click)
        {

            if(picking.selected_joint!=-1 && gui_param.display_center_of_mass)
            {
                int joint = picking.selected_joint_true;
                if(joint!=-1)
                {
                    vec3 const& com = position_center_of_mass(joint);
                    int picked_com = run_picking_selection_point_set(scene_arg, window, {com});

                    if(picked_com==0){
                        picking.picked_center_of_mass = true;
                        picking.p_clicked = com;
                        picking.p_current = com;
                    }

                }
            }


            int joint_selected = run_picking_selection_skeleton(scene_arg, window, skeleton_current);
            if(joint_selected!=-1 && picking.picked_center_of_mass==false)
            {
                vec3 const& picked_position = skeleton_current[joint_selected].p;
                picking.is_selected = true;
                picking.selected_joint = joint_selected;
                picking.p_clicked = picked_position;
                picking.p_current = picked_position;
                picking.selected_joint_true = picking.selected_joint;

                if( joint_selected!=0 && gui_param.type_deformation==1 ) // hack to get the actual selected joint which is the parent
                {
                    picking.selected_joint_true = skeleton.connectivity[joint_selected].parent;
                }

                std::cout<<"Picked joint: "<<joint_selected<<std::endl;
            }
        }

        if(mouse_release){
            picking.picked_center_of_mass = false;
            if(picking.is_selected){
                picking.is_selected = false;
                apply_deformation_on_skeleton();
            }
        }
    }

}

void scene_model::keyboard_input(scene_structure& , GLFWwindow* , int key, int , int action, int )
{

    if(key==GLFW_KEY_R && action==GLFW_PRESS)
    {
        if(picking.selected_joint!=-1)
        {

            gui_param.record_anim = !gui_param.record_anim;
            std::cout<<"Recording: "<<gui_param.record_anim<<std::endl;

            if(gui_param.record_anim==true)
            {
                record_position.clear();
                record_rotation.clear();

                timer_recording.start();
                timer_recording.update();
                timer_recording.periodic_event_time_step=record_dt;
                local_time_record = 0.0f;
                record_joint_fixed.clear();

                int joint = skeleton.connectivity[picking.selected_joint].parent;
                if( joint!=-1 )
                    recorded_joint = joint;
                else{
                    recorded_joint = 0;
                }



                int j = recorded_joint;
                record_joint_fixed.clear();
                while(j!=-1)
                {
                    record_joint_fixed.insert(j);
                    j = skeleton.connectivity[j].parent;
                }

            }
            else
            {
                timer_recording.stop();
                record_joint_fixed.clear();


                std::cout<<"Recorded joint: "<<recorded_joint<<std::endl;

                if(record_rotation.size()>0)
                {
                    int N_time_record = record_rotation.size();
                    int N_time_prev = skeleton.anim[recorded_joint].size();
                    int N_time_max = std::max(N_time_record, N_time_prev);

                    for(int kj=0; kj<int(skeleton.anim.size()); ++kj)
                    {
                        skeleton.anim[kj].resize(N_time_max);
                        for(int kt=N_time_prev; kt<N_time_record; ++kt)
                            skeleton.anim[kj][kt].geometry = skeleton.anim[kj][N_time_prev-1].geometry;
                    }
                    for(int kt=0; kt<N_time_record; ++kt){
                        skeleton.anim[recorded_joint][kt].geometry.r = record_rotation[kt];
                        skeleton.anim[recorded_joint][kt].geometry.p = record_position[kt];
                    }
                    for(int kt=N_time_record; kt<N_time_prev; ++kt)
                        skeleton.anim[recorded_joint][kt].geometry = skeleton.anim[recorded_joint][N_time_record-1].geometry;



                    for(int kj=0; kj<int(skeleton.anim.size()); ++kj)
                        for(int kt=0; kt<N_time_max; ++kt)
                            skeleton.anim[kj][kt].time = record_dt*kt;

                    timer_skeleton.t_max = record_dt*(N_time_max-1);

                    if(gui_param.symmetry)
                    {
                        if(symmetrical_joint.find(recorded_joint)!=symmetrical_joint.end())
                        {
                            int js = symmetrical_joint[recorded_joint];
                            quaternion const q = quaternion::axis_angle({1,0,0},3.14f);
                            for(int kt=0; kt<N_time_max; ++kt)
                                skeleton.anim[js][kt].geometry.r = q*skeleton.anim[recorded_joint][kt].geometry.r*conjugate(q);
                        }
                    }
                    //skeleton.anim[11] = skeleton.anim[10];

                    timer_skeleton.t = 0;
                }



            }
        }
    }


    if(key==GLFW_KEY_S && action==GLFW_PRESS)
    {
        std::cout<<"Export Model ..."<<std::endl;


        {
            std::ofstream fid("scene_web/mesh.json", std::ofstream::out);
            fid << "{\n";
            fid << "\"vertices\": [";
            size_t const N = skinning.rest_pose.size();
            for(size_t k=0; k<N; ++k)
            {
                vec3 const p = skinning.rest_pose[k];
                fid<<p.x<<", "<<p.y<<", "<<p.z;
                if(k<N-1)
                    fid<<", ";
            }
            fid << "],\n";

            fid << "\"uv\": [";
            for(size_t k=0; k<N; ++k)
            {
                vec2 const p = skinning.deformed.texture_uv[k];
                fid<<p.x<<", "<<p.y;
                if(k<N-1)
                    fid<<", ";
            }
            fid << "],\n";

            fid << "\"connectivity\": [";
            size_t const Nt = skinning.deformed.connectivity.size();
            for(size_t kt=0; kt<Nt; ++kt)
            {
                uint3 const f = skinning.deformed.connectivity[kt];
                fid<<f[0]<<", "<<f[1]<<", "<<f[2];
                if(kt<Nt-1)
                    fid<<", ";
            }
            fid << "]\n";

            fid << "}\n";
            fid.close();
        }


        {
            size_t const N = skeleton.connectivity.size();

            std::ofstream fid("scene_web/skeleton.json", std::ofstream::out);
            fid << "{\n";


            fid << "\"names\": [";
            for(size_t k=0; k<N; ++k)
            {
                fid<<"\""<<skeleton.connectivity[k].name<<"\"";
                if(k<N-1) fid<<", ";
            }
            fid << "],\n";


            fid << "\"parent_id\": [";
            for(size_t k=0; k<N; ++k)
            {
                fid<<skeleton.connectivity[k].parent;
                if(k<N-1) fid<<", ";
            }
            fid << "],\n";


            fid << "\"translation\": [";
            for(size_t k=0; k<N; ++k)
            {
                vec3 const p = skeleton.rest_pose[k].p;
                fid<<p.x<<", "<<p.y<<", "<<p.z;
                if(k<N-1) fid<<", ";
            }
            fid << "],\n";

            fid << "\"rotation\": [";
            for(size_t k=0; k<N; ++k)
            {
                quaternion const q = skeleton.rest_pose[k].r;
                fid<<q.x<<", "<<q.y<<", "<<q.z<<", "<<q.w;
                if(k<N-1) fid<<", ";
            }
            fid << "]\n";

            fid << "}\n";
            fid.close();
        }

        {

            std::ofstream fid("scene_web/rig.json", std::ofstream::out);
            fid << "{\n";
            int N_vertex = skinning.influence.size();
            fid << "\"joint\" : ";
            fid<<"[";
            for(int k_vertex=0; k_vertex<N_vertex; ++k_vertex)
            {
                int N_bones = skinning.influence[k_vertex].size();
                fid<<"[";
                for(int k_bone = 0; k_bone<N_bones; ++k_bone)
                {
                    int const j = skinning.influence[k_vertex][k_bone].joint;
                    fid<<j;
                    if(k_bone<N_bones-1) fid<<", ";
                }
                fid<<"]";
                if(k_vertex<N_vertex-1) fid<<", ";
            }
            fid<<"],\n";

            fid << "\"weight\" : ";
            fid<<"[";
            for(int k_vertex=0; k_vertex<N_vertex; ++k_vertex)
            {
                int N_bones = skinning.influence[k_vertex].size();
                fid<<"[";
                for(int k_bone = 0; k_bone<N_bones; ++k_bone)
                {
                    float const w = skinning.influence[k_vertex][k_bone].weight;
                    fid<<w;
                    if(k_bone<N_bones-1) fid<<", ";
                }
                fid<<"]";
                if(k_vertex<N_vertex-1) fid<<", ";
            }
            fid<<"] \n";

            fid << "}\n";
            fid.close();
        }


        // Export anim
        size_t const N_joint = skeleton.rest_pose.size();
        buffer<buffer<vec3>> center_of_mass(N_joint);
        {
            float const t_min = timer_skeleton.t_min;
            float const t_max = timer_skeleton.t_max;
            int N_time = int((t_max-t_min)/record_dt);
            assert_vcl_no_msg(N_time>1);





            buffer<mat4> T_to_gpu_anim(N_joint* N_time);
            buffer<vec3> angular_velocity_to_gpu_anim(N_joint* N_time);



            buffer<buffer<vec3>> skeleton_position(N_joint);
            buffer<buffer<quaternion>> skeleton_rotation(N_joint);
            for(int k_time=0; k_time<N_time; ++k_time)
            {
                float const dt = (t_max-t_min)/(N_time-1);
                float const t = t_min + k_time * dt;

                interpolate_skeleton_at_time_with_constraints(skeleton_local_current,
                                                              t,
                                                              skeleton.anim,
                                                              gui_param.interpolate,
                                                              record_joint_fixed);
                skeleton_current = local_to_global(skeleton_local_current, skeleton.connectivity);
                for(int k=0; k<int(skeleton_current.size()); ++k) {
                    skeleton_joint_speed[k].add( skeleton_local_current[k].p, t );
                    skeleton_joint_rotation_speed[k].add( skeleton_local_current[k].r, t );

                    mat3 R_parent = mat3::identity();
                    if(k>0)
                        R_parent = skeleton_current[ skeleton.connectivity[k].parent ].r.matrix();

                    skeleton_speed_per_joint[k].center = skeleton_current[k].p;
                    skeleton_speed_per_joint[k].linear_speed  = R_parent * skeleton_joint_speed[k].avg_speed;
                    skeleton_speed_per_joint[k].angular_speed = R_parent * skeleton_joint_rotation_speed[k].avg_rotation_speed;

                    angular_velocity_to_gpu_anim[k + N_joint * k_time] = skeleton_speed_per_joint[k].angular_speed;
                }
                //update_center_of_mass();



                for(size_t kj=0; kj<N_joint; ++kj)
                {
                    skeleton_position[kj].push_back(skeleton_local_current[kj].p);
                    skeleton_rotation[kj].push_back(skeleton_local_current[kj].r);
                    //center_of_mass[kj].push_back(center_of_mass_per_joint[kj]);
                }


            }

            {
                //buffer<mat4> T_to_gpu_anim(N_joint * N_time);
                //buffer<vec3> angular_velocity_to_gpu_anim(N_joint * N_time);

                for (int k_time = 0; k_time < N_time; ++k_time)
                {
                    for (size_t kj = 0; kj < N_joint; ++kj)
                    {
                        const mat3& R = skeleton_rotation[kj][k_time].matrix();
                        const vec3& p = skeleton_position[kj][k_time];
                        const mat4 T = mat4::from_mat3_vec3(R, p);

                        const mat3& R0 = skeleton_rest_pose[kj].r.matrix();
                        const vec3& p0 = skeleton_rest_pose[kj].p;
                        const mat4 T0_inv = mat4::from_mat3_vec3(transpose(R0), -transpose(R0) * p0);

                        const mat4 TT = T * T0_inv;
                        T_to_gpu_anim[kj + N_joint * k_time] = TT;

                    }
                }


                //GLuint buffer_angular_velocity_anim;
                //GLuint texture_angular_velocity_anim;
                //GLuint buffer_T_anim;
                //GLuint texture_T_anim;


            }

            {
                std::ofstream fid("scene_web/anim.txt", std::ofstream::out);
                for (int kt = 0; kt < N_time; ++kt)
                {
                    float const dt = (t_max - t_min) / (N_time - 1);
                    float const t = t_min + kt * dt;
                    fid << t;
                    if (kt < N_time - 1) fid << " ";
                }
                fid.close();
            }
            {
                std::ofstream fid("scene_web/rotation.txt", std::ofstream::out);
                for (int kt = 0; kt < N_time; ++kt)
                {
                    for (size_t kj = 0; kj < N_joint; ++kj)
                    {
                        quaternion const& q = skeleton_rotation[kj][kt];
                        fid << q.x << " " << q.y << " " << q.z << " " << q.w<<" ";
                    }
                }
                fid.close();
            }
            {
                std::ofstream fid("scene_web/position.txt", std::ofstream::out);
                for (int kt = 0; kt < N_time; ++kt)
                {
                    for (size_t kj = 0; kj < N_joint; ++kj)
                    {
                        vec3 const& p = skeleton_position[kj][kt];
                        fid << p.x << " " << p.y << " " << p.z<<" ";
                    }
                }
                fid.close();
            }


            std::ofstream fid("scene_web/anim.json", std::ofstream::out);

            fid << "{\n";

            fid <<"\"time\" : [";
            for(int kt=0; kt<N_time; ++kt)
            {
                float const dt = (t_max-t_min)/(N_time-1);
                float const t = t_min + kt * dt;
                fid << t;
                if(kt<N_time-1) fid << ", ";
            }
            fid<<"], \n";


            fid <<"\"position\" : [";
            for(int kt=0; kt<N_time; ++kt)
            {
                fid<<"[";
                for(size_t kj=0; kj<N_joint; ++kj)
                {
                    vec3 const& p = skeleton_position[kj][kt];
                    fid << p.x<<", "<<p.y<<", "<<p.z;
                    if(kj<N_joint-1) fid << ", ";

                }
                fid<<"]";
                if(kt<N_time-1) fid << ", ";
            }
            fid<<"], \n";

            fid <<"\"rotation\" : [";
            for(int kt=0; kt<N_time; ++kt)
            {
                fid<<"[";
                for(size_t kj=0; kj<N_joint; ++kj)
                {
                    quaternion const& q = skeleton_rotation[kj][kt];
                    fid << q.x<<", "<<q.y<<", "<<q.z<<", "<<q.w;
                    if(kj<N_joint-1) fid << ", ";

                }
                fid<<"]";
                if(kt<N_time-1) fid << ", ";
            }
            fid<<"] \n";

            fid << "}\n";
            fid.close();

        }

        {
            size_t const N_joint = skeleton.connectivity.size();
            size_t const N_vertex = skinning.rest_pose.size();

            std::ofstream fid("scene_web/velocity_skinning.json", std::ofstream::out);
            fid << "{\n";

            fid << "\"flappy_weights\" : [";
            for(size_t k_vertex=0; k_vertex<N_vertex; ++k_vertex)
            {
                float const w = weight_flappy[k_vertex];
                fid << w;
                if(k_vertex<N_vertex-1) fid<<" ,";
            }
            fid << "], \n";
/*
            fid << "\"center of mass\" : [";
            int const N_time = center_of_mass[0].size();
            for(int kt=0; kt<N_time; ++kt)
            {
                fid<<"[";
                for(size_t kj=0; kj<N_joint; ++kj)
                {
                    vec3 const& com = center_of_mass[kj][kt];
                    fid << com.x<<", "<<com.y<<", "<<com.z;
                    if(kj<N_joint-1) fid << ", ";

                }
                fid<<"]";
                if(kt<N_time-1) fid << ", ";
            }
            fid << "], \n";*/

            /*
            fid << "\"vertex_depending_on_joint\" : [";
            for(size_t k_joint=0; k_joint<N_joint; ++k_joint)
            {
                size_t const N = vertex_depending_on_joint[k_joint].size();
                fid<<" [";
                for(size_t k_v=0; k_v<N; ++k_v)
                {
                    fid << vertex_depending_on_joint[k_joint][k_v];
                    if(k_v<N-1) fid << ", ";
                }
                fid<<"]";
                if(k_joint<N_joint-1) fid << ", ";
            }
            fid << "],\n";

            fid << "\"vertex_weight_depending_on_joint\" : [";
            for(size_t k_joint=0; k_joint<N_joint; ++k_joint)
            {
                size_t const N = vertex_weight_depending_on_joint[k_joint].size();
                fid<<" [";
                for(size_t k_v=0; k_v<N; ++k_v)
                {
                    fid << vertex_weight_depending_on_joint[k_joint][k_v];
                    if(k_v<N-1) fid << ", ";
                }
                fid<<"]";
                if(k_joint<N_joint-1) fid << ", ";
            }
            ;
            fid << "]\n";

            fid << "}\n";
            */
            fid.close();

        }



        std::cout<<"Export Done"<<std::endl;

    }
}


#endif
