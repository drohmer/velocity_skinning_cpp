#pragma once

#include "skinning.hpp"
#ifdef SCENE_SQUASHY_SKINNING

struct data_loaded
{
    skeleton_structure skeleton;
    vcl::mesh shape;
    vcl::buffer< vcl::buffer<skinning_influence> > skinning_rig;
    vcl::buffer<float> weight_flappy;
    std::map<int,int> symmetry;
    GLuint shader;
    GLuint texture_id;
    float anim_time_max;
};

bool check_integrity(data_loaded const& data);

// Helper function reading skinning data from file
vcl::buffer<joint_geometry> read_skeleton_geometry(const std::string& filename, float scaling);
vcl::buffer<vcl::buffer<joint_geometry_time> > read_skeleton_animation(const std::string& filename, float scaling);
vcl::buffer<joint_connectivity> read_skeleton_connectivity(const std::string& filename);
vcl::buffer<vcl::buffer<skinning_influence> > read_skinning_influence(const std::string& filename);
vcl::buffer<vcl::buffer<skinning_influence> > read_skinning_influence_variable(const std::string& filename);

// Initiate shape and skinning weights
void load_sphere_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader);

void load_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader);
void load_rectangle_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader);


void load_diagonal_translate_cylinder_data (skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader);
void load_bending_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader);

void load_bending_twisting_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader);


void load_rondinella_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader);

void load_character_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader);
void load_girafe_data   (skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader);
data_loaded load_girafe2_data(GLuint shader);
data_loaded load_spot_data(GLuint shader);
data_loaded load_dragon_data(GLuint shader);
data_loaded load_snail_data(GLuint shader);

//Map correspondance between skinning weights and vertices (that have been duplicated to load the texture coordinates)
template <typename T>
vcl::buffer<T> map_correspondance(vcl::buffer<T> value, vcl::buffer<vcl::buffer<int> > const& correspondance)
{
    // find the maximal index used for destination
    int max_index = 0;
    for(size_t k1=0; k1<correspondance.size(); ++k1)
        for(size_t k2=0; k2<correspondance[k1].size(); ++k2)
            if(max_index<correspondance[k1][k2])
                max_index = correspondance[k1][k2];

    vcl::buffer<T> new_value;
    new_value.resize(max_index+1);

    // Apply correspondance (copy value[k] in its destination)
    for(size_t k=0; k<correspondance.size(); ++k)
        for(size_t k2=0; k2<correspondance[k].size(); ++k2)
            new_value[correspondance[k][k2]] = value[k];
    return new_value;
}

#endif
