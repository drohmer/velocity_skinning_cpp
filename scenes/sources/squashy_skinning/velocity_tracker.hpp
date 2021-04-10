#pragma once
#include "scenes/base/base.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "vcl/math/vec/vec3/vec3.hpp"
#include "quaternion.hpp"

#include <list>

namespace vcl
{

struct speed_tracker
{
    speed_tracker();
    void add(const vec3& new_position, float time);

    vec3 last_position;
    vec3 last_speed;
    float last_time;

    int counter_initialization;

    vec3 avg_speed;
    vec3 avg_acceleration;

    // temporal filter new_speed = alpha*old_speed + (1-alpha)*current_speed
    float alpha_speed = 0.75f;
    float alpha_acceleration = 0.9f;
};

struct rotation_tracker
{
    rotation_tracker();
    void add(const quaternion& new_rotation, float time);

    quaternion last_rotation;
    //quaternion last_rotation_speed;
    float last_time;

    int counter_initialization;

    vec3 avg_rotation_speed;
    //quaternion avg_rotation_acceleration;

    // temporal filter new_speed = alpha*old_speed + (1-alpha)*current_speed
    float alpha_speed = 0.75f;//0.75f;
    //float alpha_acceleration = 0.9f;
};

template< typename T >
struct velocity_tracker_structure
{
    void clear();
    void add(const T& position_arg, float time_arg);
    T speed_avg() const;
    T acceleration_avg() const;

    size_t N_position_max = 8;
    std::list<T> position = {};
    std::list<float> time    = {};
};

struct quaternion_tracker_structure
{
    void clear();
    void add(const quaternion& position_arg, float time_arg);
    quaternion speed_avg() const;
    //quaternion acceleration_avg() const;

    size_t N_position_max = 8;
    std::list<quaternion> position = {};
    std::list<float> time    = {};
};


}



// Implementation

namespace vcl
{

template< typename T >
void velocity_tracker_structure<T>::clear()
{
    position.clear();
    time.clear();
}

template< typename T >
void velocity_tracker_structure<T>::add(const T& position_arg, float time_arg)
{
    position.push_back(position_arg);
    time.push_back(time_arg);

    if(position.size()>N_position_max)
    {
        position.pop_front();
        time.pop_front();
    }
}

template< typename T >
T velocity_tracker_structure<T>::speed_avg() const
{
    if( position.size()<2 )
        return T();

    const T& p1 = *position.begin();
    const T& p2 = *position.rbegin();

    const float t1 = *time.begin();
    const float t2 = *time.rbegin();

//    if( ! (t2>t1+1e-5f) )
//        return T();


    const T s = (p2-p1)/(t2-t1);
    return s;
}

template< typename T >
T velocity_tracker_structure<T>::acceleration_avg() const
{
    if( position.size()<5 )
        return {0,0,0};

    const int N = position.size();
    auto it_p = position.begin();
    auto it_t = time.begin();
    for(int k=0; k<N/2; ++k){
        ++it_p; ++it_t;
    }


    const T& p1 = *position.begin();
    const T& p2 = *position.rbegin();
    const T& p = *it_p; // middle position

    const float t1 = *time.begin();
    const float t2 = *time.rbegin();
    const float t  = *it_t; // middle position

//    if( ! (t2>t1+1e-5f) )
//        return {0,0,0};


    const T a = ((p2-p)/(t2-t)+(p-p1)/(t-t1))/(t2-t1);
    return a;
}



}

#endif

