
#include "skinning.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "skinning_loader.hpp"
#include "helper_skeleton.hpp"

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
        vec3 const& linear_speed = (skeleton_speed_per_joint.data[joint].linear_speed / timer_skeleton.scale + skeleton_fake_speed_per_joint.data[joint].linear_speed);
        vec3 const& angular_speed = (skeleton_speed_per_joint.data[joint].angular_speed / timer_skeleton.scale + skeleton_fake_speed_per_joint.data[joint].angular_speed);
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


    timer_measurement["full"].tic();

    scene = _scene;
    timer.update();

    if(gui_param.animation)
        timer_skeleton.update();

    set_gui();

    float const t = timer.t;





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


    // apply deformation to skeleton
    for(size_t k=0; k<skeleton_local_current.size(); ++k) {
        skeleton_local_current[k].p += skeleton_local_interactive_deformation[k].p;
        skeleton_local_current[k].r = skeleton_local_interactive_deformation[k].r * skeleton_local_current[k].r;
    }




    skeleton_current = local_to_global(skeleton_local_current, skeleton.connectivity);
    if(is_rotating){
        skeleton_current[0].r = qq;
    }




    // Update per-join velocity
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


    timer_measurement["skinning"].tic();
    if(gui_param.dual_quaternion)
        compute_skinning_dual_quaternion(skinning, skeleton_current, skeleton_rest_pose);
    else
        compute_skinning(skinning, skeleton_current, skeleton_rest_pose);
    timer_measurement["skinning"].toc();



    character_visual.update_position(skinning.deformed.position);
    character_visual.update_normal(skinning.deformed.normal);

    save_skinning = skinning.deformed.position;

    //    // Update per-vertex velocity
    //    for(int k=0; k<int(skinning.deformed.position.size()); ++k)
    //        vertex_speed[k].add( skinning.deformed.position[k], t );

    /*
    character_visual.uniform.transform.translation = {0,0,-0.8f};
    if(gui_param.display_mesh) {
        glPolygonOffset( 1.0, 1.0 );
        GLuint const texture_id = (gui_param.display_texture? character_visual.texture_id : scene.texture_white);
        draw(character_visual, scene.camera, character_visual.shader, texture_id);
    }
    if(gui_param.display_wireframe) {
        glPolygonOffset( 1.0, 1.0 );
        draw(character_visual, scene.camera, shaders["wireframe_quads"]);
    }
    character_visual.uniform.transform.translation = {0,0,0};
    */


    timer_measurement["center of mass"].tic();
    update_center_of_mass();
    timer_measurement["center of mass"].toc();


    // Apply velocity skinning
    timer_measurement["velocity_skinning"].tic();
    velocity_skinning(1.0f);
    timer_measurement["velocity_skinning"].toc();


    timer_measurement["normals"].tic();
    normal(skinning.deformed.position, skinning.deformed.connectivity, skinning.deformed.normal);
    timer_measurement["normals"].toc();

    if(gui_param.display_deformed_surface)
    {
        character_visual.update_position(skinning.deformed.position);
        character_visual.update_normal(skinning.deformed.normal);
    }



    timer_measurement["drawing"].tic();



    scene.camera.apply_translation_orthogonal_to_screen_plane(-1.0f);
    scene.camera.apply_translation_in_screen_plane(0.5,0.5);
    mat4 lightCam = scene.camera.perspective.matrix()
            *
            scene.camera.view_matrix();
    scene.camera.apply_translation_in_screen_plane(-0.5,-0.5);
    scene.camera.apply_translation_orthogonal_to_screen_plane(1.0f);
    {

        glUseProgram(shaders["depth_map"]);
        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT); opengl_debug();
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO); opengl_debug();
        glClear(GL_DEPTH_BUFFER_BIT); opengl_debug();
        //glEnable(GL_DEPTH_TEST); opengl_debug();

        //glClearColor(0.1f, 0.1f, 0.1f, 1.0f); opengl_debug();
        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // we're not using the stencil buffer now
        //glEnable(GL_DEPTH_TEST); opengl_debug();

        opengl_debug();
        //GLint lightSpaceMatrixLocation = glGetUniformLocation(shaders["depth_map"], "lightSpaceMatrix");
        //glUniformMatrix4fv(lightSpaceMatrixLocation, 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));

        mat4 Id = mat4::identity();
        //scene.camera.apply_translation_in_screen_plane(0.2,0.2);
        uniform(shaders["depth_map"], "lightSpaceMatrix", lightCam);
        uniform(shaders["depth_map"], "model", Id);


        //glCullFace(GL_FRONT);
        draw(character_visual.data);
        //glCullFace(GL_BACK);

        //draw(character_visual, scene.camera, shaders["depth_map"]); opengl_debug();

        glBindFramebuffer(GL_FRAMEBUFFER, 0); opengl_debug(); // back to default
        glViewport(0, 0, scene.window_width, scene.window_height); opengl_debug();


    }




    //    {


    //        float near_plane = 1.0f, far_plane = 7.5f;
    //        glm::mat4 lightProjection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, near_plane, far_plane);

    //        glm::mat4 lightView = glm::lookAt(glm::vec3(-2.0f, 4.0f, -1.0f),
    //                                          glm::vec3( 0.0f, 0.0f,  0.0f),
    //                                          glm::vec3( 0.0f, 1.0f,  0.0f));


    //        glm::mat4 lightSpaceMatrix = lightProjection * lightView;


    //        glUseProgram(shaders["depth_map"]);
    //        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
    //        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    //        glClear(GL_DEPTH_BUFFER_BIT);
    //        //glActiveTexture(GL_TEXTURE0);


    //        mat4 Id = mat4::identity();

    //        GLint lightSpaceMatrixLocation = glGetUniformLocation(shaders["depth_map"], "lightSpaceMatrix");
    //        glUniformMatrix4fv(lightSpaceMatrixLocation, 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));
    //        uniform(shaders["depth_map"], "model", Id);

    //        draw(character_visual.data);
    //        //draw(character_visual, scene.camera, shaders["depth_map"]);




    //        glBindFramebuffer(GL_FRAMEBUFFER, 0);

    //        glViewport(0, 0, scene.window_width, scene.window_height);
    //        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //    }

    /*
    glUseProgram(shaders["mesh_shadow"]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    quad.uniform.color = {1.0,1.0,1.0};
    quad.texture_id = 0;
    //quad.uniform.shading = {1,0,0};
    //quad.texture_id = depthMap;//character_visual.texture_id;

    //uniform(shaders["mesh_shadow"], "texture_sampler", 0);
    //uniform(shaders["mesh_shadow"], "shadowMap", 1);
    glActiveTexture(GL_TEXTURE0+0);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
    glActiveTexture(GL_TEXTURE0+1);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    uniform(shaders["mesh_shadow"],"lightSpaceMatrix",lightCam);

    draw(quad, scene.camera, shaders["mesh_shadow"]);

    glUseProgram(0);
*/



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




//character_visual.texture_id=0;
    if(gui_param.display_mesh) {
        //glPolygonOffset( 1.0, 1.0 );

        glUseProgram(shaders["mesh_shadow"]);
        character_visual.shader = shaders["mesh_shadow"];
        //character_visual.texture_id = 0;


        //        if(character_visual.texture_id==0 || gui_param.display_texture==false)
        //            character_visual.texture_id = scene.texture_white;

        glActiveTexture(GL_TEXTURE0+0);
        glBindTexture(GL_TEXTURE_2D, character_visual.texture_id==0? scene.texture_white : character_visual.texture_id);

        glActiveTexture(GL_TEXTURE0+1);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        uniform(shaders["mesh_shadow"],"lightSpaceMatrix",lightCam);

        GLuint id_save = character_visual.texture_id;
        character_visual.texture_id = 0;
        //GLuint const texture_id = (gui_param.display_texture? character_visual.texture_id : scene.texture_white);
        draw(character_visual, scene.camera);//, character_visual.shader, texture_id);
        character_visual.texture_id = id_save;
    }

    glActiveTexture(GL_TEXTURE0+0);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
    glActiveTexture(GL_TEXTURE0+1);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);



    //glUseProgram(shaders["mesh"]);
    //uniform(shaders["mesh"], "texture_sampler", 0);

    if(gui_param.display_wireframe) {
        glPolygonOffset( 1.0, 1.0 );
        draw(character_visual, scene.camera, shaders["wireframe_quads"]);
    }

    glBindTexture(GL_TEXTURE_2D,scene.texture_white);





    // display displacement
    /*
    {
        arrow_line.scaling_width = 0.2f;
        sphere_visual.uniform.transform.scaling = 0.003f;

        for(int k=0; k<int(save_skinning.size()); k+=gui_param.display_deformation_arrow_vertex_offset)
        {
            vec3 const& p = save_skinning[k];
            vec3 const& d = deformation_per_vertex[k];

            if(gui_param.display_deformation_arrows)
                arrow_line.draw(p, p+d, {0.8f,0.8f,1.0f}, scene.camera);

            if(gui_param.display_deformation_target){
                if(norm(d)>1e-5f){
                    sphere_visual.uniform.transform.translation = p+d;
                    draw(sphere_visual, scene.camera);
                }
            }

        }
        arrow_line.scaling_width = 1.0f;
        sphere_visual.uniform.transform.scaling = 1.0f;
    }
    */

    // iterative trajectory
    {
        arrow_line.scaling_width = 0.2f;
        sphere_visual.uniform.transform.scaling = 0.003f;
        sphere_visual.uniform.color = {1,1,1};


        if(gui_param.curved_trajectory && gui_param.display_deformation_arrows)
        {
            buffer<vec3> previous_position = save_skinning;

            int N_alpha = 5; // sampling of the trajectory
            for(int k_alpha=0; k_alpha<N_alpha; ++k_alpha)
            {
                float const alpha = (k_alpha+1.0f)/N_alpha;
                velocity_skinning(alpha);

                for(int k=0; k<int(save_skinning.size()); k+=gui_param.display_deformation_arrow_vertex_offset)
                {
                    vec3 const& p0 = previous_position[k];
                    vec3 const& p1 = save_skinning[k] + deformation_per_vertex[k];

                    arrow_line.draw(p0, p1, {0.8f,0.8f,1.0f}, scene.camera);

                    if(k_alpha==N_alpha-1 && gui_param.display_deformation_target){
                        if(norm(deformation_per_vertex[k])>1e-3f){
                            sphere_visual.uniform.transform.translation = p1;
                            draw(sphere_visual, scene.camera);
                        }
                    }
                }

                previous_position = save_skinning + deformation_per_vertex;
            }
        }

        if(!gui_param.curved_trajectory && gui_param.display_deformation_target)
        {
            for(int k=0; k<int(save_skinning.size()); k+=gui_param.display_deformation_arrow_vertex_offset)
            {
                vec3 const& p = save_skinning[k];
                vec3 const& d = deformation_per_vertex[k];

                arrow_line.draw(p, p+d, {0.8f,0.8f,1.0f}, scene.camera);
            }
        }

        if(gui_param.display_deformation_target)
        {
            for(int k=0; k<int(save_skinning.size()); k+=gui_param.display_deformation_arrow_vertex_offset)
            {
                vec3 const& p = save_skinning[k];
                vec3 const& d = deformation_per_vertex[k];

                if(norm(d)>1e-5f){
                    sphere_visual.uniform.transform.translation = p+d;
                    draw(sphere_visual, scene.camera);
                }
            }
        }

        /*
        for(int k=0; k<int(save_skinning.size()); k+=gui_param.display_deformation_arrow_vertex_offset)
        {
            vec3 const& p = save_skinning[k];
            vec3 const& d = deformation_per_vertex[k];

            if(gui_param.display_deformation_arrows)
                arrow_line.draw(p, p+d, {0.8f,0.8f,1.0f}, scene.camera);

            if(gui_param.display_deformation_target){
                if(norm(d)>1e-5f){
                    sphere_visual.uniform.transform.translation = p+d;
                    draw(sphere_visual, scene.camera);
                }
            }
        }
        */
        arrow_line.scaling_width = 1.0f;
        sphere_visual.uniform.transform.scaling = 1.0f;
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
            int parent = skeleton.connectivity[k].parent;

            vec3 const& p0 = skeleton_current[parent].p;
            vec3 const& p1 = skeleton_current[k].p;

            vec3 const u = p1-p0;
            const float L = norm(p1-p0);
            const float Lr = std::min(L,0.2f);
            vec3 const un = u/L;

            if(picking.joint_hover==int(k))
                pyramid_skeleton_visual.uniform.color = {0.8f, 0.3f, 0.3f};
            else
                pyramid_skeleton_visual.uniform.color = {0.3f, 0.3f, 0.3f};
            pyramid_skeleton_visual.uniform.transform.translation = p0;
            pyramid_skeleton_visual.uniform.transform.scaling_axis = {Lr,Lr,L};
            pyramid_skeleton_visual.uniform.transform.rotation = rotation_between_vector_mat3({0,0,1}, un);

            draw(pyramid_skeleton_visual, scene.camera);
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

    //    segment_drawer.uniform_parameter.color = {0,0,0};
    //    if(gui_param.display_joint_speed){
    //        segment_drawer.uniform_parameter.color = {0,0,1};
    //        display_joint_speed(skeleton_current, skeleton_joint_speed, segment_drawer, shaders["segment_im"], scene.camera);
    //    }
    //    if(gui_param.display_joint_acceleration){
    //        segment_drawer.uniform_parameter.color = {0,1,1};
    //        display_joint_acceleration(skeleton_current, skeleton_joint_speed, segment_drawer, shaders["segment_im"], scene.camera);
    //    }


    /*
    {
        if(skeleton.connectivity.size()>9)
        {
            //int joint_picked = picking.selected_joint;
            int joint = 9;//skeleton.connectivity[joint_picked].parent;
            if(joint!=-1)
            {
                vec3 const& p_joint = skeleton_current[joint].p;
                vec3 const& bar = center_of_mass_per_joint[joint];
                vec3 const& angular_speed = skeleton_speed_per_joint[joint].angular_speed;

                sphere_visual.uniform.color = {1,1,0};
                sphere_visual.uniform.transform.scaling = 0.03f;
                sphere_visual.uniform.transform.translation = bar;
                draw(sphere_visual, scene.camera);

                vec3 const u_medial = bar - p_joint;
                vec3 const un_medial = normalize(u_medial);

                vec3 const un_angular_speed = normalize(angular_speed);
                vec3 const un_scaling = normalize(cross(un_medial, un_angular_speed));
                vec3 const un_squeeze = cross(un_medial, un_scaling);



                segment_drawer.uniform_parameter.color = {1,0,1};
                segment_drawer.uniform_parameter.p1 = p_joint;
                segment_drawer.uniform_parameter.p2 = p_joint+2*u_medial;
                segment_drawer.draw(shaders["segment_im"], scene.camera);

                segment_drawer.uniform_parameter.color = {0.8,0.8,1};
                segment_drawer.uniform_parameter.p1 = p_joint;
                segment_drawer.uniform_parameter.p2 = p_joint+2*un_scaling;
                segment_drawer.draw(shaders["segment_im"], scene.camera);

                segment_drawer.uniform_parameter.color = {0.2,0.2,0.2};
                segment_drawer.uniform_parameter.p1 = p_joint;
                segment_drawer.uniform_parameter.p2 = p_joint+2*un_squeeze;
                segment_drawer.draw(shaders["segment_im"], scene.camera);


                mat3 S = { 1+0.5f,0,0, 0,1/std::sqrt(1+0.5f),0, 0,0,1.0f };
                mat3 R = mat3(un_scaling, un_squeeze, un_medial);
                mat3 const T = R*S*transpose(R);

                for(int k=0; k<vertex_depending_on_joint[9].size(); ++k)
                {
                    int id = vertex_depending_on_joint[9][k];
                    vec3 const& p_vertex = save_skinning[id];
                    vec3 const p_medial = p_joint + dot(p_vertex-p_joint,un_medial)*un_medial;


                    std::cout<<id<<std::endl;

                    segment_drawer.uniform_parameter.color = {0.0,0.0,0.0};
                    segment_drawer.uniform_parameter.p1 = p_vertex;
                    segment_drawer.uniform_parameter.p2 = p_medial;
                    segment_drawer.draw(shaders["segment_im"], scene.camera);

                    sphere_visual.uniform.transform.translation = T * (p_vertex-p_medial) + p_medial;
                    sphere_visual.uniform.transform.scaling = 0.01f;
                    sphere_visual.uniform.color = {1,0.4,1};
                    draw(sphere_visual, scene.camera);
                }



            }
        }
    }
*/






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


    timer_measurement["drawing"].toc();

    timer_measurement["full"].toc();




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

        if(click_girafe || click_spot || click_dragon || click_snail)
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
                click_bar || click_character || click_girafe || click_spot ||  click_dragon || click_snail) {
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
                snprintf(buffer_timing, S, "[%d%] [%.1fms] %s - %.1fs  ", int(t/total*100), double(1000*avg), name.c_str(), double(t));
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
    skeleton_local_current_before_deformation  = interpolate_skeleton_at_time(0, skeleton.anim, gui_param.interpolate);
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

    if(weight_squashy.size()!=skinning.rest_pose.size()){
        weight_squashy.resize(skinning.rest_pose.size());
        weight_squashy.fill(1.0f);
    }
    if(weight_flappy.size()!=skinning.rest_pose.size()){
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
        for(size_t k_tri=0; k_tri<N_tri; ++k_tri)
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
        for(size_t kt=0; kt<N_triangle; ++kt)
        {
            uint3 const& tri = skinning.deformed.connectivity[kt];
            unsigned int a=tri[0], b=tri[1], c=tri[2];
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

        for(int kv=0; kv<N_vertex; ++kv)
        {
            auto const& influence = skinning.influence[kv];
            int const N_influence = influence.size();

            std::map<int,float> cumulative_weight_per_joint;

            for(int k_influence=0; k_influence<N_influence; ++k_influence)
            {
                int current_joint = influence[k_influence].joint;
                float weight = influence[k_influence].weight;

                while(current_joint != -1)
                {

                    if(cumulative_weight_per_joint.find(current_joint)!=cumulative_weight_per_joint.end())
                        cumulative_weight_per_joint[current_joint] += weight;
                    else
                        cumulative_weight_per_joint[current_joint]  = weight;

                    current_joint = skeleton.connectivity[current_joint].parent;
                }
            }

            for(auto const& it : cumulative_weight_per_joint)
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

    for(int k_vertex=0; k_vertex<N_vertex; ++k_vertex)
    {
        buffer<int> const& joint_dependencies = rig_extended_to_ancestor_joint[k_vertex];
        for(int k_joint=0; k_joint<int(joint_dependencies.size()); ++k_joint)
        {
            int joint = rig_extended_to_ancestor_joint[k_vertex][k_joint];
            float weight = rig_extended_to_ancestor_weight[k_vertex][k_joint];
            vertex_depending_on_joint[joint].push_back(k_vertex);
            vertex_weight_depending_on_joint[joint].push_back(weight);
        }
    }

    skinning_weights_per_joint_per_vertex.resize_clear(N_joint);
    for(int kj=0; kj<N_joint; ++kj)
    {
        skinning_weights_per_joint_per_vertex[kj].resize_clear(N_vertex);
        for(size_t k=0; k<vertex_depending_on_joint[kj].size(); ++k)
        {
            int vertex_id = vertex_depending_on_joint[kj][k];
            float w = vertex_weight_depending_on_joint[kj][k];
            skinning_weights_per_joint_per_vertex[kj][vertex_id] = w;
        }

    }



    vcl::buffer<std::set<int> > triangle_set_depending_on_joint(N_joint);
    int const N_triangle = skinning.deformed.connectivity.size();
    for(int k_triangle=0; k_triangle<N_triangle; ++k_triangle)
    {
        uint3 const& tri = skinning.deformed.connectivity[k_triangle];
        unsigned int a = tri[0];
        unsigned int b = tri[1];
        unsigned int c = tri[2];

        buffer<int> const& joints_a = rig_extended_to_ancestor_joint[a];
        buffer<int> const& joints_b = rig_extended_to_ancestor_joint[b];
        buffer<int> const& joints_c = rig_extended_to_ancestor_joint[c];

        for(int j : joints_a)
            triangle_set_depending_on_joint[j].insert(k_triangle);
        for(int j : joints_b)
            triangle_set_depending_on_joint[j].insert(k_triangle);
        for(int j : joints_c)
            triangle_set_depending_on_joint[j].insert(k_triangle);
    }

    triangle_depending_on_joint.resize_clear(N_joint);
    for(int kj=0; kj<N_joint; ++kj)
    {
        for(int t : triangle_set_depending_on_joint[kj])
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
    load_bending_cylinder_data(skeleton, skinning, weight_flappy, weight_squashy, character_visual, timer_skeleton, shader_mesh);
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

    resize_structure();


    {

        glGenFramebuffers(1, &depthMapFBO); opengl_debug();
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);  opengl_debug();
        glGenTextures(1, &depthMap); opengl_debug();
        glBindTexture(GL_TEXTURE_2D, depthMap); opengl_debug();
        //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL); opengl_debug();
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL); opengl_debug();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); opengl_debug();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);  opengl_debug();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);  opengl_debug();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);  opengl_debug();
        float borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);  opengl_debug();
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0); opengl_debug();
        //glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, depthMap, 0); opengl_debug();


        //        glGenRenderbuffers(1, &rbo); opengl_debug();
        //        glBindRenderbuffer(GL_RENDERBUFFER, rbo); opengl_debug();
        //        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, 1024, 1024); opengl_debug();
        //        glBindRenderbuffer(GL_RENDERBUFFER, 0); opengl_debug();
        //        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo); opengl_debug();
        //        if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) opengl_debug();
        //            std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
        //        glBindFramebuffer(GL_FRAMEBUFFER, 0); opengl_debug();

        //        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        //        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, depthMap, 0);
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }


    timer_recording.stop();
    record_dt = 0.1f;

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

        vec3 const& p0 = save_skinning[u0];
        vec3 const& p1 = save_skinning[u1];
        vec3 const& p2 = save_skinning[u2];

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
                }
                update_center_of_mass();



                for(size_t kj=0; kj<N_joint; ++kj)
                {
                    skeleton_position[kj].push_back(skeleton_local_current[kj].p);
                    skeleton_rotation[kj].push_back(skeleton_local_current[kj].r);
                    center_of_mass[kj].push_back(center_of_mass_per_joint[kj]);
                }


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
            fid.close();
        }



        std::cout<<"Export Done"<<std::endl;

    }
}


#endif
