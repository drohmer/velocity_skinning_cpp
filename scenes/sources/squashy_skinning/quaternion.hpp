#pragma once
#include "scenes/base/base.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "vcl_advanced/vcl_advanced.hpp"

namespace vcl
{

// Helper quaternion structure with associated functions
struct quaternion : vcl::vec4 {
    quaternion();
    quaternion(const vcl::vec4& v);
    quaternion(float x_arg,float y_arg,float z_arg,float w_arg);

    // Build quaternion from axis/angle values
    static quaternion axis_angle(const vcl::vec3& axis, float angle);
    vcl::mat3 matrix() const;

    // Apply quaternion to
    vcl::vec3 apply(const vcl::vec3& p) const;
};


std::istream& read_from_stream(std::istream& stream, quaternion& data);

quaternion inverse(quaternion const& q);
quaternion operator*(float s, const quaternion& q);
quaternion operator*(const quaternion& q, const float s);
quaternion operator+(const quaternion& q1, const quaternion& q2);
quaternion operator*(const quaternion& q1,const quaternion& q2);
quaternion operator/(const quaternion& q, float s);
quaternion conjugate(const quaternion& q);
quaternion slerp(quaternion q1, const quaternion& q2, float t);

quaternion normalize(quaternion const& q);

//namespace vcl{
//template <>
//vcl::buffer<quaternion> file_reader_as_buffer<quaternion>::read(const std::string& filename);
//}


#ifdef SOLUTION
struct quaternion_dual
{
    quaternion q;
    quaternion qe;

    quaternion_dual();
    quaternion_dual(quaternion const& q, quaternion const qe);
    quaternion_dual(quaternion const& q, vcl::vec3 const& tr);

    vcl::vec3 translation() const;
};

quaternion_dual& operator+=(quaternion_dual& a, quaternion_dual const& b);
quaternion_dual operator+(quaternion_dual const& a, quaternion_dual const& b);
quaternion_dual operator*(float s, quaternion_dual const& d);
quaternion_dual operator/(quaternion_dual const& d, float s);
#endif

}

#endif
