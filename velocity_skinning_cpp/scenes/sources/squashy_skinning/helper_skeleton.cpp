#include "helper_skeleton.hpp"
#ifdef SCENE_SQUASHY_SKINNING

using namespace vcl;

#ifdef _WIN32
#pragma warning(disable : 4267)
#endif

void interpolate_skeleton_at_time_with_constraints(buffer<joint_geometry>& skeleton, float time, const buffer< buffer<joint_geometry_time> >& animation, bool interpolate, std::set<int> const& fixed_joints)
{
    size_t N_joint = animation.size();
    assert_vcl_no_msg(N_joint == skeleton.size());


    for(size_t k_joint=0; k_joint<N_joint; ++k_joint)
    {
        if( fixed_joints.find( int(k_joint) )==fixed_joints.end() )
        {
            const buffer<joint_geometry_time>& joint_anim = animation[k_joint];

            // Find the index corresponding to the current time
            size_t k_current = 0;
            assert_vcl_no_msg(joint_anim.size()>k_current+1);
            while( time>joint_anim[k_current+1].time ) {
                ++k_current;
                assert_vcl_no_msg(joint_anim.size()>k_current+1);
            }


            if(interpolate){
                const joint_geometry& g1 = joint_anim[k_current].geometry;
                const joint_geometry& g2 = joint_anim[k_current+1].geometry;
                const float t1 = joint_anim[k_current].time;
                const float t2 = joint_anim[k_current+1].time;



                const float alpha = (time-t1)/(t2-t1);
                const vec3 p = (1.0f-alpha)*g1.p+alpha*g2.p;

                joint_geometry g;
                g.p = p;
                g.r = slerp(g1.r,g2.r,alpha);


                skeleton[k_joint] = g;
            }
            else {
                const joint_geometry current_geometry = joint_anim[k_current].geometry;
                skeleton[k_joint] = current_geometry;
            }
        }

    }
}

buffer<joint_geometry> interpolate_skeleton_at_time(float time, const buffer< buffer<joint_geometry_time> >& animation, bool interpolate)
{
    // Compute skeleton corresponding to a given time from key poses
    // - animation[k] corresponds to the kth animated joint (vector of joint geometry at given time)
    // - animation[k][i] corresponds to the ith pose of the kth joint
    //      - animation[k][i].time         -> corresponding time
    //      - animation[k][i].geometry.p/r -> corresponding position, rotation

    size_t N_joint = animation.size();
    buffer<joint_geometry> skeleton;
    skeleton.resize(N_joint);

    for(size_t k_joint=0; k_joint<N_joint; ++k_joint)
    {
        const buffer<joint_geometry_time>& joint_anim = animation[k_joint];

        // Find the index corresponding to the current time
        size_t k_current = 0;
        assert_vcl_no_msg(joint_anim.size()>k_current+1);
        while( time>joint_anim[k_current+1].time ) {
            ++k_current;
            assert_vcl_no_msg(joint_anim.size()>k_current+1);
        }


        if(interpolate){
            const joint_geometry& g1 = joint_anim[k_current].geometry;
            const joint_geometry& g2 = joint_anim[k_current+1].geometry;
            const float t1 = joint_anim[k_current].time;
            const float t2 = joint_anim[k_current+1].time;



            const float alpha = (time-t1)/(t2-t1);
            const vec3 p = (1.0f-alpha)*g1.p+alpha*g2.p;

            joint_geometry g;
            g.p = p;
            g.r = slerp(g1.r,g2.r,alpha);


            skeleton[k_joint] = g;
        }
        else {
            const joint_geometry current_geometry = joint_anim[k_current].geometry;
            skeleton[k_joint] = current_geometry;
        }

    }

    return skeleton;
}


// Convert skeleton from local to global coordinates
buffer<joint_geometry> local_to_global(const buffer<joint_geometry>& local, const buffer<joint_connectivity>& connectivity)
{


    const size_t N = connectivity.size();
    assert_vcl_no_msg(local.size()==connectivity.size());
    std::vector<joint_geometry> global;
    global.resize(N);
    global[0] = local[0];

    // T_global = T_global^parent * T_local (T: 4x4 transformation matrix)
    //   => R_global = R_global^parent * R_local
    //   => P_global = R_global^parent * P_local + P_global^parent
    for(size_t k=1; k<N; ++k)
    {
        const int parent = connectivity[k].parent;
        global[k].r = global[parent].r * local[k].r;
        global[k].p = global[parent].r.apply(local[k].p) + global[parent].p;
    }

    return global;
}


void display_skeleton(const buffer<joint_geometry>& skeleton_geometry,
                      const buffer<joint_connectivity>& skeleton_connectivity,
                      const std::map<std::string,GLuint>& shaders,
                      const scene_structure& scene,
                      segment_drawable_immediate_mode& segment_drawer)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=1; k<N; ++k)
    {
        int parent = skeleton_connectivity[k].parent;
        const vec3& p1 = skeleton_geometry[parent].p;
        const vec3& p2 = skeleton_geometry[k].p;

        segment_drawer.uniform_parameter.p1 = p1;
        segment_drawer.uniform_parameter.p2 = p2;
        segment_drawer.draw(shaders.at("segment_im"),scene.camera);
    }
}

void display_joints(const buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    mesh_drawable& sphere)
{
    const size_t N = skeleton_geometry.size();
    sphere.uniform.transform.scaling = 0.005f;
    for(size_t k=0; k<N; ++k)
    {
        sphere.uniform.transform.translation = skeleton_geometry[k].p;
        draw(sphere, scene.camera);
    }
    sphere.uniform.transform.scaling = 1.0f;

}

void display_frames(const buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    mesh_drawable& frame)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=0; k<N; ++k)
    {
        frame.uniform.transform.rotation = skeleton_geometry[k].r.matrix();
        frame.uniform.transform.translation = skeleton_geometry[k].p;
        draw(frame, scene.camera);
    }
}

vcl::buffer<vcl::buffer<int> > compute_joint_sons(vcl::buffer<joint_connectivity> const& skeleton_connectivity)
{
    int N_joint = int(skeleton_connectivity.size());
    vcl::buffer<vcl::buffer<int> > sons(N_joint);
    for(int k=1; k<N_joint; ++k)
    {
        int parent = skeleton_connectivity[k].parent;
        sons[parent].push_back(k);
    }

    return sons;
}

vcl::buffer< std::array<vcl::vec3,2> > get_all_sons_bones_segment(int joint, vcl::buffer<joint_geometry> const& geometry, vcl::buffer<vcl::buffer<int> > const& sons)
{
    int N_sons = sons[joint].size();
    vec3 const& pa = geometry[joint].p; // initial point
    vcl::buffer< std::array<vcl::vec3,2> > segments;
    segments.resize(N_sons);

    for(int k=0; k<N_sons; ++k)
    {
        segments[k][0] = pa;
        segments[k][1] = geometry[ sons[joint][k] ].p;
    }

    return segments;
}


vec3 closest_point_on_segment(vec3 const& p, vec3 const& pa, vec3 const& pb)
{
    vec3 const pab = pb-pa;
    float const L = norm(pab);
    if( is_equal(L,0.0f) )
        return pa;

    vec3 const u = pab/L;

    float const d = dot(p-pa,u);

    // orthogonal projection is inside the segment ab
    if(d>=0 && d<L) {
        vec3 const pi = pa + d*u;
        return pi;
    }

    // otherwise return the closest extremity
    float const d_pa = norm(p-pa);
    float const d_pb = norm(p-pb);

    if(d_pa<d_pb)
        return pa;
    else
        return pb;
}


vcl::buffer<bone_correspondance> compute_bone_correspondance(vcl::buffer<vcl::vec3> const& position,
                                                             vcl::buffer< vcl::buffer<skinning_influence> > const& influence,
                                                             vcl::buffer<vcl::buffer<int>> skeleton_son_connectivity,
                                                             vcl::buffer<joint_geometry> const& skeleton_geometry)
{
    int const N_vertex = position.size();
    buffer<bone_correspondance> correspondance(N_vertex);

    assert_vcl_no_msg(influence.size()==position.size());


    for(int k_vertex=0; k_vertex<N_vertex; ++k_vertex)
    {
        vec3 const& p = position[k_vertex];

        auto const& vertex_influence = influence[k_vertex];
        int const N_influence = vertex_influence.size();

        float min_dist = 0.0f;
        bone_correspondance best_correspondance;
        for(int k_influence=0; k_influence<N_influence; ++k_influence)
        {
            int const joint = vertex_influence[k_influence].joint;

            float const d = norm(p-skeleton_geometry[joint].p);
            if( k_influence==0 || d<min_dist) {
                min_dist = d;
                best_correspondance = {joint,joint,0.0f};
            }

            vcl::buffer< std::array<vcl::vec3,2> > all_sons_bones = get_all_sons_bones_segment(joint, skeleton_geometry, skeleton_son_connectivity);
            int N_possible_segment = all_sons_bones.size();
            for(int ks=0; ks<N_possible_segment; ++ks)
            {
                vec3 const& pa = all_sons_bones[ks][0];
                vec3 const& pb = all_sons_bones[ks][1];

                //std::cout<<"Son of "<<joint<<" is "<<pa<<" "<<pb<<std::endl;

                vec3 const pi = closest_point_on_segment(p, pa,pb);
                float const ds = norm(p-pi);
                if( (k_influence==0 && ks==0) || ds<min_dist)
                {
                    min_dist = ds;

                    best_correspondance.joint_origin = joint;
                    best_correspondance.joint_end = skeleton_son_connectivity[joint][ks];
                    float const L = norm(pb-pa);
                    if( is_equal(L,0.0f) )
                        best_correspondance.alpha = 0.0f;
                    else
                        best_correspondance.alpha = norm(pi-pa)/L;
                }
            }


        }


        correspondance[k_vertex] = best_correspondance;

    }


    return correspondance;
}

#endif
