#pragma once

#include "scenes/base/base.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "skinning_loader.hpp"





void interpolate_skeleton_at_time_with_constraints(vcl::buffer<joint_geometry>& skeleton, float time, const vcl::buffer< vcl::buffer<joint_geometry_time> >& animation, bool interpolate, std::set<int> const& fixed_joints);
vcl::buffer<joint_geometry> interpolate_skeleton_at_time(float time, const vcl::buffer< vcl::buffer<joint_geometry_time> >& animation, bool interpolate);
vcl::buffer<joint_geometry> local_to_global(const vcl::buffer<joint_geometry>& local, const vcl::buffer<joint_connectivity>& connectivity);

// compute sons of joints in the skeleton
vcl::buffer<vcl::buffer<int> > compute_joint_sons(vcl::buffer<joint_connectivity> const& skeleton_connectivity);

// get all the possible sons segments (pa,pb) at a given joint
vcl::buffer< std::array<vcl::vec3,2> > get_all_sons_bones_segment(int joint, vcl::buffer<joint_geometry> const& geometry, vcl::buffer<vcl::buffer<int> > const& sons);

vcl::buffer<bone_correspondance> compute_bone_correspondance(vcl::buffer<vcl::vec3> const& position,
                                                             vcl::buffer< vcl::buffer<skinning_influence> > const& influence,
                                                             vcl::buffer<vcl::buffer<int>> skeleton_son_connectivity,
                                                             vcl::buffer<joint_geometry> const& skeleton_geometry);

void display_frames(const vcl::buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    vcl::mesh_drawable& frame);

void display_joints(const vcl::buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    vcl::mesh_drawable& sphere);

void display_skeleton(const vcl::buffer<joint_geometry>& skeleton_geometry,
                      const vcl::buffer<joint_connectivity>& skeleton_connectivity,
                      const std::map<std::string,GLuint>& shaders,
                      const scene_structure& scene,
                      vcl::segment_drawable_immediate_mode& segment_drawer);



#endif
