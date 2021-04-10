#pragma once

#include "scenes/base/base.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "quaternion.hpp"
#include "helper_skinning.hpp"
#include "velocity_tracker.hpp"

#include <set>

struct joint_speed {
    vcl::vec3 center;
    vcl::vec3 linear_speed;
    vcl::vec3 angular_speed;
};

struct arrow_drawable {

    void init(unsigned int shader);
    vcl::mesh_drawable cylinder;
    vcl::mesh_drawable cone;

    float length_cone = 0.03f;
    float radius_cone = 0.015f;
    float radius_cylinder = 0.006f;
    float scaling_width = 1.0f;

    void draw(vcl::vec3 const& p1, vcl::vec3 const& p2, vcl::vec3 const& color, vcl::camera_scene const& camera);
};

struct line_drawable {

    void init(unsigned int shader);
    vcl::mesh_drawable cylinder;

    float radius_cylinder = 0.006f;
    float scaling_width = 1.0f;

    void draw(vcl::vec3 const& p1, vcl::vec3 const& p2, vcl::vec3 const& color, vcl::camera_scene const& camera);
};




void display_joint_speed(vcl::buffer<joint_geometry> const& skeleton_current,
                         vcl::buffer<vcl::speed_tracker> const& skeleton_joint_speed,
                         vcl::segment_drawable_immediate_mode& segment_drawer,
                         GLuint shader, vcl::camera_scene const& camera);

void display_joint_acceleration(vcl::buffer<joint_geometry> const& skeleton_current,
                                vcl::buffer<vcl::speed_tracker> const& skeleton_joint_speed,
                                vcl::segment_drawable_immediate_mode& segment_drawer,
                                GLuint shader, vcl::camera_scene const& camera);

void display_arrow(vcl::vec3 const& p0, vcl::vec3 const& p1, vcl::camera_scene const& camera);

void display_sphere_hover(vcl::mesh_drawable& sphere_visual,
                          int joint_hover_id,
                          vcl::buffer<joint_geometry> const& skeleton_current,
                          vcl::camera_scene const& camera);

void display_sphere_selected(vcl::mesh_drawable& sphere_visual,
                             vcl::vec3 const& p_clicked,
                             vcl::vec3 const& p_current,
                             vcl::camera_scene const& camera);

void display_painting_cursor(int selected_vertex,
                        skinning_structure const& skinning,
                        float painting_radius, float threshold_percentage,
                        vcl::curve_drawable& painting_cursor,
                        vcl::mesh_drawable& sphere_visual,
                        vcl::camera_scene const& camera);

void display_vertex_to_bone_correspondance(vcl::buffer<vcl::vec3> const& position,
                                           vcl::buffer<joint_geometry> const& skeleton_current,
                                           vcl::buffer<bone_correspondance> const& vertex_to_bone_correspondance,
                                           vcl::segment_drawable_immediate_mode & segment_drawer,
                                           GLuint shader, vcl::camera_scene const& camera);


vcl::buffer<float> diffuse_laplacien_weights(vcl::buffer<float> const& weights,
                                        vcl::buffer<std::set<unsigned int> > const& one_ring,
                                        float lambda, int number_of_steps);

#endif
