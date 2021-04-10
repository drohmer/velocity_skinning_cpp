#include "unit_test.hpp"
#include "vcl/vcl.hpp"

namespace vcl
{

void unit_test()
{
    {
        buffer<float> const a = {1,2,3,4};
        buffer<float> const b = {4,5,3,1};


        assert_vcl_no_msg( is_equal(-a, {-1,-2,-3,-4}) );
        assert_vcl_no_msg( is_equal(a+b, {5,7,6,5}) );
        assert_vcl_no_msg( is_equal(a-b, {-3,-3,0,3}) );
        assert_vcl_no_msg( is_equal(a*b, {4,10,9,4}) );
        assert_vcl_no_msg( is_equal(a/b, {0.25f,0.4f,1.0f,4.0f}) );

        assert_vcl_no_msg( is_equal(a+1.0f, {2,3,4,5}) );
        assert_vcl_no_msg( is_equal(1.0f+a, {2,3,4,5}) );
        assert_vcl_no_msg( is_equal(2.0f*a, {2,4,6,8}) );
        assert_vcl_no_msg( is_equal(a*2.0f, {2,4,6,8}) );
        assert_vcl_no_msg( is_equal(a/2.0f, {0.5,1,1.5,2}) );
    }

    {
        buffer< vec3 > const a = { {1,2,3}, {2,1,2} , {3,1,-2} };
        assert_vcl_no_msg( is_equal(2*a, {{2,4,6},{4,2,4},{6,2,-4}}) );
        assert_vcl_no_msg( is_equal(a*2, {{2,4,6},{4,2,4},{6,2,-4}}) );
        assert_vcl_no_msg( is_equal(2.0f*a, {{2,4,6},{4,2,4},{6,2,-4}}) );
        assert_vcl_no_msg( is_equal(a*2.0f, {{2,4,6},{4,2,4},{6,2,-4}}) );

        assert_vcl_no_msg( is_equal(vec3(1,2,3)+2*a, {{3,6,9},{5,4,7},{7,4,-1}}) );
        assert_vcl_no_msg( is_equal(2*a+vec3(1,2,3), {{3,6,9},{5,4,7},{7,4,-1}}) );
    }

    {
        vec2 a = {1.0f, 2.0f};
        vec3 b = {1.0f, 2.0f, 3.0f};
        vec4 c = {1.0f, 2.0f, 3.0f, 4.0f};

        assert_vcl_no_msg( is_equal(str(a), str(1.0f)+" "+str(2.0f)) );
        assert_vcl_no_msg( is_equal(str(b), str(1.0f)+" "+str(2.0f)+" "+str(3.0f)) );
        assert_vcl_no_msg( is_equal(str(c), str(1.0f)+" "+str(2.0f)+" "+str(3.0f)+" "+str(4.0f)) );
    }

    {
        vec3 a = {1.0f, 2.0f, 3.0f};
        int counter=0;
        for(float v : a) {
            if(counter==0)
                assert_vcl_no_msg( is_equal(v,1.0f) );
            if(counter==1)
                assert_vcl_no_msg( is_equal(v,2.0f) );
            if(counter==2)
                assert_vcl_no_msg( is_equal(v,3.0f) );
            counter++;
        }
        assert_vcl_no_msg(counter==3);
    }




    {
#ifdef __linux__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif
        buffer2D<float> a(2,2);
        a(0,0)=1; a(0,1)=2;
        a(1,0)=3; a(1,1)=4;
        buffer2D<float> b(2,2);
        b(0,0)=4; b(0,1)=5;
        b(1,0)=3; b(1,1)=1;

        buffer2D<float> const c1 = a+b;
        buffer2D<float> const c2 = a-b;
        buffer2D<float> const c3 = a*b;
        buffer2D<float> const c4 = a/b;
        assert_vcl_no_msg( c1(0,0)==5 && c1(1,0)==6 && c1(0,1)==7 && c1(1,1)==5);
        assert_vcl_no_msg( c2(0,0)==-3 && c2(1,0)==0 && c2(0,1)==-3 && c2(1,1)==3);
        assert_vcl_no_msg( c3(0,0)==4 && c3(1,0)==9 && c3(0,1)==10 && c3(1,1)==4);
        assert_vcl_no_msg( c4(0,0)==0.25f && c4(1,0)==1.0f && c4(0,1)==0.4f && c4(1,1)==4.0f);

        buffer2D<float> const c5 = a+1.0f;
        buffer2D<float> const c6 = 2.0f*a;
        buffer2D<float> const c7 = a/2.0f;
        assert_vcl_no_msg( c5(0,0)==2 && c5(1,0)==4 && c5(0,1)==3 && c5(1,1)==5);
        assert_vcl_no_msg( c6(0,0)==2 && c6(1,0)==6 && c6(0,1)==4 && c6(1,1)==8);
        assert_vcl_no_msg( c7(0,0)==0.5f && c7(1,0)==1.5f && c7(0,1)==1.0f && c7(1,1)==2.0f);

#ifdef __linux__
#pragma GCC diagnostic pop
#endif
    }



    {
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({1,0,0},{1,0,0})*vec3(1,0,0),{1,0,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({1,0,0},{0,1,0})*vec3(1,0,0),{0,1,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({1,0,0},{0,0,1})*vec3(1,0,0),{0,0,1}) );

        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({1,0,0},{-1,0,0})*vec3(1,0,0),{-1,0,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({1,0,0},{0,-1,0})*vec3(1,0,0),{0,-1,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({1,0,0},{0,0,-1})*vec3(1,0,0),{0,0,-1}) );

        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,1,0},{1,0,0})*vec3(0,1,0),{1,0,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,1,0},{0,1,0})*vec3(0,1,0),{0,1,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,1,0},{0,0,1})*vec3(0,1,0),{0,0,1}) );

        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,1,0},{-1,0,0})*vec3(0,1,0),{-1,0,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,1,0},{0,-1,0})*vec3(0,1,0),{0,-1,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,1,0},{0,0,-1})*vec3(0,1,0),{0,0,-1}) );

        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,0,1},{1,0,0})*vec3(0,0,1),{1,0,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,0,1},{0,1,0})*vec3(0,0,1),{0,1,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,0,1},{0,0,1})*vec3(0,0,1),{0,0,1}) );

        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,0,1},{-1,0,0})*vec3(0,0,1),{-1,0,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,0,1},{0,-1,0})*vec3(0,0,1),{0,-1,0}) );
        assert_vcl_no_msg( is_equal(rotation_between_vector_mat3({0,0,1},{0,0,-1})*vec3(0,0,1),{0,0,-1}) );
    }

    {
        assert_vcl_no_msg( is_equal(linspace(-2,2,5), {-2,-1,0,1,2}) );
        assert_vcl_no_msg( is_equal(linspace(0,2,5), {0,0.5,1,1.5,2}) );

        {
            buffer2D<vec2> const b = linspace({0,0},{2,2},{3,3});
            assert_vcl_no_msg( is_equal(b(0,0),{0,0}) );
            assert_vcl_no_msg( is_equal(b(1,0),{1,0}) );
            assert_vcl_no_msg( is_equal(b(2,0),{2,0}) );

            assert_vcl_no_msg( is_equal(b(0,1),{0,1}) );
            assert_vcl_no_msg( is_equal(b(1,1),{1,1}) );
            assert_vcl_no_msg( is_equal(b(2,1),{2,1}) );

            assert_vcl_no_msg( is_equal(b(0,2),{0,2}) );
            assert_vcl_no_msg( is_equal(b(1,2),{1,2}) );
            assert_vcl_no_msg( is_equal(b(2,2),{2,2}) );
        }

        {
            buffer3D<vec3> const b = linspace({0,0,0},{2,2,2},{3,3,3});
            for(int kx=0; kx<3; ++kx)
                for(int ky=0; ky<3; ++ky)
                    for(int kz=0; kz<3; ++kz)
                        assert_vcl_no_msg( is_equal(b(kx,ky,kz),vec3(float(kx),float(ky),float(kz))) );
        }


    }

}

}

