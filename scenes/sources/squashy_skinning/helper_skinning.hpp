#pragma once

#include "scenes/base/base.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "quaternion.hpp"


// Connectivity information of a joint in the hierarchy
//  Store parent index, and the current name
struct joint_connectivity
{
    int parent;
    std::string name;
};

// 3D Geometry of a joint (position p, and rotation r)
struct joint_geometry
{
    vcl::vec3 p;
    vcl::quaternion r;
};

// Key pose of a joint (Key-time, and geometry at this time)
struct joint_geometry_time
{
    float time;
    joint_geometry geometry;
};

// Storage of the influence of a joint for a given vertex
struct skinning_influence
{
    int joint;    // index of the corresponding joint
    float weight; // skinning weight of this joint
};

// Structure storing skeleton data to perform skinning afterward
struct skeleton_structure
{
    vcl::buffer<joint_connectivity> connectivity;           // Connectivity of the skeleton
    vcl::buffer<joint_geometry>     rest_pose;              // Skeleton of the rest pose expressed in local coordinates
    vcl::buffer<vcl::buffer<joint_geometry_time> > anim;    // Skeleton animation expressed in local coordinates (N_joint x N_time)
};

// Storage structure to perform skinning deformation of a surface
struct skinning_structure
{
    vcl::buffer< vcl::buffer<skinning_influence> > influence; // Skinning weights: for each vertex, store all influence values (bone+weight)
    vcl::buffer<vcl::vec3> rest_pose;                         // 3D position of the mesh in rest pose
    vcl::buffer<vcl::vec3> rest_pose_normal;                  // 3D normals of the mesh in rest pose
    vcl::mesh deformed;                                       // Deformed mesh
};

struct bone_correspondance
{
    bone_correspondance();
    bone_correspondance(int joint_origin, int joint_end, float alpha);

    int joint_origin;
    int joint_end;
    float alpha;  // relative position with respect to the segment
};

void compute_skinning(skinning_structure& skinning,
                      const vcl::buffer<joint_geometry>& skeleton_current,
                      const vcl::buffer<joint_geometry>& skeleton_rest_pose);

void compute_skinning_dual_quaternion(skinning_structure& skinning,
                                      const vcl::buffer<joint_geometry>& skeleton_current,
                                      const vcl::buffer<joint_geometry>& skeleton_rest_pose);






#endif
