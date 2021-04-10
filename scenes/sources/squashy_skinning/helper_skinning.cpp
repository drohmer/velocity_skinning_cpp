#include "helper_skinning.hpp"
#ifdef SCENE_SQUASHY_SKINNING

using namespace vcl;

#ifdef _WIN32
#pragma warning(disable : 4267)
#endif

bone_correspondance::bone_correspondance()
    :joint_origin(-1), joint_end(-1), alpha(-1.0f)
{}

bone_correspondance::bone_correspondance(int joint_origin_arg, int joint_end_arg, float alpha_arg)
    :joint_origin(joint_origin_arg), joint_end(joint_end_arg), alpha(alpha_arg)
{}

void compute_skinning(skinning_structure& skinning,
                      const vcl::buffer<joint_geometry>& skeleton_current,
                      const vcl::buffer<joint_geometry>& skeleton_rest_pose)
{
    const size_t N_vertex = skinning.rest_pose.size();
    for(size_t k=0; k<N_vertex; ++k)
    {
        const buffer<skinning_influence>& influence = skinning.influence.data[k];

        // Transformation matrix for skinning
        mat3 R = mat3::zero();
        vec3 tr = {0,0,0};
        for(size_t kb=0; kb<influence.size(); ++kb) // Loop over influencing joints
        {
            const int idx = influence.data[kb].joint;
            const float w = influence.data[kb].weight;

            const quaternion& r = skeleton_current.data[idx].r;
            const vec3& p = skeleton_current.data[idx].p;

            const quaternion& r0 = skeleton_rest_pose.data[idx].r;
            const quaternion r0_inv = conjugate(r0);
            const vec3& p0 = skeleton_rest_pose.data[idx].p;


            // Convert rotation/translation to matrix
            //mat4 T = mat4::from_mat3_vec3(r.matrix(), p);
            //mat4 T0_inv = mat4::from_mat3_vec3(conjugate(r0).matrix(), conjugate(r0).apply(-p0)); // inverse

            // Skinning
            //M += w*T*T0_inv;

            R  += w * (r * r0_inv).matrix();
            tr += w * (p - (r*r0_inv).apply(p0));


        }


        // Apply skinning transform on vertex
        const vec3& p0 = skinning.rest_pose.data[k];
        skinning.deformed.position.data[k] = R*p0+tr;

        const vec3& n0 = skinning.rest_pose_normal.data[k];
        skinning.deformed.normal.data[k] = R*n0;
    }

}


void compute_skinning_dual_quaternion(skinning_structure& skinning,
                                      const buffer<joint_geometry>& skeleton_current,
                                      const buffer<joint_geometry>& skeleton_rest_pose)
{
    const size_t N_vertex = skinning.rest_pose.size();
    for(size_t k=0; k<N_vertex; ++k)
    {
        const buffer<skinning_influence>& influence = skinning.influence[k];

        quaternion_dual d = { {0,0,0,0}, {0,0,0,0} };
        for(size_t kb=0; kb<influence.size(); ++kb) // Loop over influencing joints
        {
            const int idx = influence[kb].joint;
            float w = influence[kb].weight;

            quaternion const& r = skeleton_current[idx].r;
            vec3 const& p = skeleton_current[idx].p;
            quaternion const& r0 = skeleton_rest_pose[idx].r;
            vec3 const& p0 = skeleton_rest_pose[idx].p;

            quaternion const q_deform = r * conjugate(r0);
            vec3 const p_deform = (r*conjugate(r0)).apply(-p0) + p;

            d += w * quaternion_dual(q_deform, p_deform);

        }

        float const n = norm(d.q);
        if(std::abs(n)>1e-5f)
            d = d/n;

        vec3 const& p0 = skinning.rest_pose[k];
        skinning.deformed.position[k] = (d.q).apply(p0) + d.translation();

        vec3 const& n0 = skinning.rest_pose_normal[k];
        skinning.deformed.normal[k] = d.q.apply(n0);

    }
}




#endif
