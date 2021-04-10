
#include "velocity_tracker.hpp"

#ifdef SCENE_SQUASHY_SKINNING

namespace vcl
{
speed_tracker::speed_tracker()
    :last_position(),
      last_time(0),
      counter_initialization(0),
      avg_speed(),
      avg_acceleration()
{}

void speed_tracker::add(const vec3& new_position, float time)
{
    if( is_equal(time,last_time) )
        return ;

    // Update speed
    vec3 new_speed = {0,0,0};
    if(counter_initialization>=1) {
        new_speed = (new_position-last_position)/(time - last_time);
        avg_speed = alpha_speed*avg_speed + (1-alpha_speed)*new_speed;
    }
    // update acceleration
    if(counter_initialization>=2) {
        vec3 const new_acceleration = (avg_speed-last_speed)/(time - last_time);
        avg_acceleration = alpha_acceleration*avg_acceleration + (1-alpha_acceleration)*new_acceleration;
    }

    // beginning - do not measure speed & acceleration
    if(counter_initialization==1) counter_initialization = 2;
    if(counter_initialization==0) counter_initialization = 1;


    last_position = new_position;
    last_speed = avg_speed;
    last_time = time;

}


rotation_tracker::rotation_tracker()
    :last_rotation(),
      last_time(0),
      counter_initialization(0),
      avg_rotation_speed()
      //avg_rotation_acceleration()
{}

void quaternion_to_axis_angle(quaternion const& q, vec3& axis, float& angle)
{
    vec3 n_sin = vec3(q.x,q.y,q.z);
    float s = norm(n_sin);
    if(s<1e-6)
    {
        axis={1,0,0};
        angle = 0.0f;
        return ;
    }

    angle = 2 * std::atan2(s, q.w);
    axis = n_sin/s;

}

void rotation_tracker::add(const quaternion& new_rotation, float time)
{
    if( is_equal(time,last_time) )
        return ;

    // Update speed
    if(counter_initialization>=1) {
        quaternion const q = new_rotation*conjugate(last_rotation);

        vec3 axis;
        float angle;
        quaternion_to_axis_angle(q, axis, angle);

        vec3 const new_angular_speed = axis * angle / (time-last_time);


        avg_rotation_speed = alpha_speed*avg_rotation_speed + (1-alpha_speed)*new_angular_speed;
    }

    // beginning - do not measure speed & acceleration
    if(counter_initialization==1) counter_initialization = 2;
    if(counter_initialization==0) counter_initialization = 1;


    last_rotation = new_rotation;
    //last_speed = avg_speed;
    last_time = time;

}




//{
//    void add(const vec3& new_position, float time);
//    vec3 last_position;
//    float last_time;

//    int counter_initialization;

//    vec3 avg_speed;
//    vec3 avg_acc;
//};



void quaternion_tracker_structure::clear()
{
    position.clear();
    time.clear();
}


void quaternion_tracker_structure::add(const quaternion& position_arg, float time_arg)
{
    position.push_back(position_arg);
    time.push_back(time_arg);

    if(position.size()>N_position_max)
    {
        position.pop_front();
        time.pop_front();
    }
}


quaternion quaternion_tracker_structure::speed_avg() const
{
    if( position.size()<2 )
        return quaternion(0,0,0,1);





    const quaternion& p1 = *position.begin();
    const quaternion& p2 = *position.rbegin();



    const float t1 = *time.begin();
    const float t2 = *time.rbegin();

    if( ! (t2>t1+1e-5f) )
        return quaternion(0,0,0,1);


    const quaternion s = (p2*conjugate(p1))/(t2-t1);
    return s;
}


//quaternion quaternion_tracker_structure::acceleration_avg() const
//{
//    if( position.size()<5 )
//        return {0,0,0};

//    const int N = position.size();
//    auto it_p = position.begin();
//    auto it_t = time.begin();
//    for(int k=0; k<N/2; ++k){
//        ++it_p; ++it_t;
//    }


//    const T& p1 = *position.begin();
//    const T& p2 = *position.rbegin();
//    const T& p = *it_p; // middle position

//    const float t1 = *time.begin();
//    const float t2 = *time.rbegin();
//    const float t  = *it_t; // middle position

//    if( ! (t2>t1+1e-5f) )
//        return {0,0,0};


//    const T a = ((p2-p)/(t2-t)+(p-p1)/(t-t1))/(t2-t1);
//    return a;
//}


}

#endif
