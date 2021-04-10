#include "helper_velocity_skinning.hpp"
#ifdef SCENE_SQUASHY_SKINNING

using namespace vcl;


#ifdef _WIN32
#pragma warning(disable : 4267)
#endif


void display_joint_speed(vcl::buffer<joint_geometry> const& skeleton_current,
                         vcl::buffer<vcl::speed_tracker> const& skeleton_joint_speed,
                         vcl::segment_drawable_immediate_mode& segment_drawer,
                         GLuint shader, camera_scene const& camera)
{
    int N_joint = skeleton_current.size();
    for(int k_joint=0; k_joint<N_joint; ++k_joint)
    {
        vec3 const& p = skeleton_current[k_joint].p;
        vec3 const& v = skeleton_joint_speed[k_joint].avg_speed;

        segment_drawer.uniform_parameter.p1 = p;
        segment_drawer.uniform_parameter.p2 = p+0.2f*v;
        segment_drawer.draw(shader, camera);
    }
}

void display_joint_acceleration(vcl::buffer<joint_geometry> const& skeleton_current,
                                vcl::buffer<vcl::speed_tracker> const& skeleton_joint_speed,
                                vcl::segment_drawable_immediate_mode& segment_drawer,
                                GLuint shader, vcl::camera_scene const& camera)
{
    int N_joint = skeleton_current.size();
    for(int k_joint=0; k_joint<N_joint; ++k_joint)
    {
        vec3 const& p = skeleton_current[k_joint].p;
        vec3 const& v = skeleton_joint_speed[k_joint].avg_acceleration;

        segment_drawer.uniform_parameter.p1 = p;
        segment_drawer.uniform_parameter.p2 = p+0.2f*v;
        segment_drawer.draw(shader, camera);
    }
}

void display_sphere_hover(mesh_drawable& sphere_visual,
                          int joint_hover_id,
                          vcl::buffer<joint_geometry> const& skeleton_current,
                          vcl::camera_scene const& camera)
{
    sphere_visual.uniform.transform.scaling = 0.015f;

    sphere_visual.uniform.color = {0.5,1,1};
    assert_vcl_no_msg(joint_hover_id < int(skeleton_current.size()));
    sphere_visual.uniform.transform.translation = skeleton_current[joint_hover_id].p;
    draw(sphere_visual, camera);

    sphere_visual.uniform.transform.scaling = 1.0f;
    sphere_visual.uniform.color = {1,1,1};

}

void display_sphere_selected(mesh_drawable& sphere_visual, vec3 const& p_clicked, vec3 const& p_current, vcl::camera_scene const& camera)
{

    sphere_visual.uniform.transform.scaling = 0.01f;

    sphere_visual.uniform.color = {1,0,0};
    sphere_visual.uniform.transform.translation = p_clicked;
    draw(sphere_visual, camera);

    sphere_visual.uniform.color = {1,1,0};
    sphere_visual.uniform.transform.translation = p_current;
    draw(sphere_visual, camera);

    sphere_visual.uniform.transform.scaling = 1.0f;
    sphere_visual.uniform.color = {1,1,1};
}

void display_painting_cursor(int selected_vertex,
                        skinning_structure const& skinning,
                             float painting_radius, float threshold_percentage,
                        curve_drawable& painting_cursor,
                        mesh_drawable& sphere_visual,
                        vcl::camera_scene const& camera)
{
    int const id = selected_vertex;
    vec3 const& p = skinning.deformed.position[id];
    vec3 const& n = skinning.deformed.normal[id];

    painting_cursor.uniform.transform.translation = p;
    painting_cursor.uniform.transform.scaling = painting_radius;
    painting_cursor.uniform.transform.rotation = rotation_between_vector_mat3({0,0,1},n);
    draw(painting_cursor, camera);

    painting_cursor.uniform.transform.translation = p;
    painting_cursor.uniform.transform.scaling = painting_radius*threshold_percentage;
    painting_cursor.uniform.transform.rotation = rotation_between_vector_mat3({0,0,1},n);
    draw(painting_cursor, camera);

    sphere_visual.uniform.transform.translation = p;
    sphere_visual.uniform.transform.scaling = 0.01f;
    sphere_visual.uniform.color = {1,1,0};
    draw(sphere_visual, camera);
    sphere_visual.uniform.color = {1,1,1};
}

void display_vertex_to_bone_correspondance(vcl::buffer<vec3> const& position,
                                           vcl::buffer<joint_geometry> const& skeleton_current,
                                           vcl::buffer<bone_correspondance> const& vertex_to_bone_correspondance,
                                           vcl::segment_drawable_immediate_mode & segment_drawer,
                                           GLuint shader, vcl::camera_scene const& camera)
{
    for(int k=0; k<int(position.size()); k=k+15)
    {
        vec3 p = position[k];
        auto bone_connect = vertex_to_bone_correspondance[k];

        vec3 pa = skeleton_current[bone_connect.joint_origin].p;
        vec3 pb = skeleton_current[bone_connect.joint_end].p;
        vec3 pi = pa + bone_connect.alpha*(pb-pa);

        segment_drawer.uniform_parameter.p1 = pi;
        segment_drawer.uniform_parameter.p2 = p;
        segment_drawer.draw(shader, camera);
    }
}



buffer<float> diffuse_laplacien_weights(buffer<float> const& weights,
                                        buffer<std::set<unsigned int> > const& one_ring,
                                        float lambda, int number_of_steps)
{

    buffer<float> w = weights;

    size_t const N = one_ring.size();
    for(int k_step=0; k_step<number_of_steps; ++k_step)
    {
        buffer<float> w_prev = w;
        for(size_t k=0; k<N; ++k)
        {
            size_t const N_neigh = one_ring[k].size();
            assert_vcl_no_msg(N_neigh>0);

            float s = 0.0f;
            for(int n : one_ring[k])
                s += w_prev[n];
            s /= float(N_neigh);

            w[k] = (1-lambda) * w_prev[k] + lambda*s;
        }
    }

    return w;
}

struct arrow_structure {

    void init(GLuint shader);
    vcl::mesh_drawable cylinder;
    vcl::mesh_drawable cone;

    void draw(vcl::vec3 const& p1, vcl::vec3 const& p2, vcl::camera_scene const& camera);
};




void line_drawable::init(unsigned int shader)
{
    cylinder = mesh_primitive_cylinder(radius_cylinder, {0,0,0}, {1,0,0}, 10,10,false);
    cylinder.shader = shader;
}


void line_drawable::draw(vec3 const& p0, vec3 const& p1, vec3 const& color, vcl::camera_scene const& camera)
{
    vec3 const u = p1-p0;
    float const L = norm(p1-p0);

    vec3 const un = u/L;
    mat3 const R = rotation_between_vector_mat3({1,0,0}, un);

    cylinder.uniform.color = color;
    cylinder.uniform.transform.scaling_axis = {L, scaling_width*1.0f, scaling_width*1.0f};
    cylinder.uniform.transform.rotation = R;
    cylinder.uniform.transform.translation = p0;

    vcl::draw(cylinder, camera);
}



void arrow_drawable::init(unsigned int shader)
{
    cylinder = mesh_primitive_cylinder(radius_cylinder, {0,0,0}, {1,0,0}, 10,10,false);
    cone = mesh_primitive_cone(radius_cone, {0.0f,0,0}, {length_cone,0,0}, 20,10);

    cylinder.shader = shader;
    cone.shader= shader;


}


void arrow_drawable::draw(vec3 const& p0, vec3 const& p1, vec3 const& color, vcl::camera_scene const& camera)
{
    vec3 const u = p1-p0;
    float const L = norm(p1-p0);
    if(L<1.5*length_cone*scaling_width)
        return;

    vec3 const un = u/L;
    mat3 const R = rotation_between_vector_mat3({1,0,0}, un);

    cylinder.uniform.color = color;
    cylinder.uniform.transform.scaling_axis = {L-length_cone*scaling_width, scaling_width*1.0f, scaling_width*1.0f};
    cylinder.uniform.transform.rotation = R;
    cylinder.uniform.transform.translation = p0;

    cone.uniform.color = color;
    cone.uniform.transform.translation = p1 - length_cone*scaling_width*un;
    cone.uniform.transform.scaling = scaling_width;
    cone.uniform.transform.rotation = R;

    vcl::draw(cylinder, camera);
    vcl::draw(cone, camera);
}

#endif
