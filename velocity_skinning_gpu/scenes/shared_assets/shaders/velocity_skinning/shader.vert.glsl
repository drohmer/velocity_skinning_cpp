#version 330 core

layout (location = 0) in vec4 position;
layout (location = 1) in vec4 normal;
layout (location = 2) in vec4 color;
layout (location = 3) in vec2 texture_uv;

out struct fragment_data
{
    vec4 position;
    vec4 normal;
    vec4 color;
    vec2 texture_uv;
} fragment;


// model transformation
uniform vec3 translation = vec3(0.0, 0.0, 0.0);                      // user defined translation
uniform mat3 rotation = mat3(1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0); // user defined rotation
uniform float scaling = 1.0;                                         // user defined scaling
uniform vec3 scaling_axis = vec3(1.0,1.0,1.0);                       // user defined scaling
uniform int N_vertex;
uniform int N_joint;

// view transform
uniform mat4 view;
// perspective matrix
uniform mat4 perspective;

//uniform mat4 skeleton[35];
//uniform mat4 skeleton_rest[35];
//uniform vec3 angular_velocity[35];

uniform samplerBuffer tbo_weights;
uniform samplerBuffer tbo_velocity_weights;

uniform samplerBuffer tbo_sk0;
uniform samplerBuffer tbo_sk;
uniform samplerBuffer tbo_angular_velocity;

mat3 rotation_from_axis_angle(vec3 u, float angle)
{
    float x = u.x;
    float y = u.y;
    float z = u.z;
    float c = cos(angle);
    float s = sin(angle);

    mat3 R = mat3(vec3(c+x*x*(1-c)  , y*x*(1-c)+z*s, z*x*(1-c)-y*s), vec3(x*y*(1-c)-z*s, c+y*y*(1-c), z*y*(1-c)+x*s), vec3(x*z*(1-c)+y*s, y*z*(1-c)-x*s, c+z*z*(1-c)) );
    //return mat3 {c+x*x*(1-c)  , x*y*(1-c)-z*s, x*z*(1-c)+y*s,
    //                y*x*(1-c)+z*s, c+y*y*(1-c)  , y*z*(1-c)-x*s,
    //                z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)   };
    return R;
}

mat4 mat4_from_texture(samplerBuffer s, int k)
{
        float xx = texelFetch(s, 16*k+0).r; float xy = texelFetch(s, 16*k+1).r; float xz = texelFetch(s, 16*k+2).r; float xw = texelFetch(s, 16*k+3).r;
        float yx = texelFetch(s, 16*k+4).r; float yy = texelFetch(s, 16*k+5).r; float yz = texelFetch(s, 16*k+6).r; float yw = texelFetch(s, 16*k+7).r;
        float zx = texelFetch(s, 16*k+8).r; float zy = texelFetch(s, 16*k+9).r; float zz = texelFetch(s, 16*k+10).r; float zw = texelFetch(s, 16*k+11).r;
        float wx = texelFetch(s, 16*k+12).r; float wy = texelFetch(s, 16*k+13).r; float wz = texelFetch(s, 16*k+14).r; float ww = texelFetch(s, 16*k+15).r;

        return mat4(vec4(xx,yx,zx,wx), vec4(xy,yy,zy,wy), vec4(xz,yz,zz,wz), vec4(xw,yw,zw,ww));
}
vec3 translation_from_mat4_texture(samplerBuffer s, int k)
{
        float xx = texelFetch(s, 16*k+0).r; float xy = texelFetch(s, 16*k+1).r; float xz = texelFetch(s, 16*k+2).r; float xw = texelFetch(s, 16*k+3).r;
        float yx = texelFetch(s, 16*k+4).r; float yy = texelFetch(s, 16*k+5).r; float yz = texelFetch(s, 16*k+6).r; float yw = texelFetch(s, 16*k+7).r;
        float zx = texelFetch(s, 16*k+8).r; float zy = texelFetch(s, 16*k+9).r; float zz = texelFetch(s, 16*k+10).r; float zw = texelFetch(s, 16*k+11).r;
        float wx = texelFetch(s, 16*k+12).r; float wy = texelFetch(s, 16*k+13).r; float wz = texelFetch(s, 16*k+14).r; float ww = texelFetch(s, 16*k+15).r;

        return vec3(texelFetch(s, 16*k+3).r, texelFetch(s, 16*k+7).r, texelFetch(s, 16*k+11).r);
}


void main()
{
    // scaling matrix
    mat4 Sm = mat4(scaling*scaling_axis.x,0.0,0.0,0.0, 0.0,scaling*scaling_axis.y,0.0,0.0, 0.0,0.0,scaling*scaling_axis.z,0.0, 0.0,0.0,0.0,1.0);
    // 4x4 rotation matrix
    mat4 Rm = mat4(rotation);
    // 4D translation
    vec4 Tm = vec4(translation,0.0);


    fragment.color = color;
    fragment.texture_uv = texture_uv;


    vec4 position_transformed = position;// + vec4(skeleton[0][0][3],skeleton[0][1][3],skeleton[0][2][3],0.0);
    vec4 normal_transformed = normal;

    vec4 p_skinning = vec4(0.0, 0.0, 0.0, 0.0);
    vec4 p_skinning_normal = vec4(0.0, 0.0, 0.0, 0.0);
    for(int k_joint=0; k_joint<N_joint; k_joint++)
    {

        mat4 T = mat4_from_texture(tbo_sk, k_joint);
        mat4 T0 = mat4_from_texture(tbo_sk0, k_joint);

        //float w = texture(texture_weights, vec2( (gl_VertexID+0.5)/(N_vertex-0.0), (k_joint+0.5)/(N_joint-0.0))).r;
        //float w = texelFetch(texture_weights,  ivec2(gl_VertexID,k_joint));
        float w = texelFetch(tbo_weights, gl_VertexID+k_joint*N_vertex).r;
        p_skinning = p_skinning + w * T * inverse(T0) * position_transformed;
        p_skinning_normal = p_skinning_normal + w * T * inverse(T0) * vec4(normal_transformed.xyz,0.0);
    }

    vec3 p_skinning3d = p_skinning.xyz;
    p_skinning_normal = normalize(p_skinning_normal);



    mat3 Id = mat3(vec3(1,0,0),vec3(0,1,0),vec3(0,0,1));
    for(int k_joint=0; k_joint<N_joint; k_joint++)
    {

        vec3 omega = vec3(texelFetch(tbo_angular_velocity, 3*k_joint+0).r,texelFetch(tbo_angular_velocity, 3*k_joint+1).r,texelFetch(tbo_angular_velocity, 3*k_joint+2).r);//+0.01*angular_velocity[k_joint];
        if( length(omega)>0.0001 ){
            vec3 un_angular_velocity = normalize(omega);
            float w = texelFetch(tbo_velocity_weights, gl_VertexID+k_joint*N_vertex).r;
            vec3 p_joint = translation_from_mat4_texture(tbo_sk, k_joint);//vec3(skeleton[k_joint][0][3],skeleton[k_joint][1][3],skeleton[k_joint][2][3]);
            vec3 u_joint = p_skinning3d - p_joint;
            vec3 rotation_center = p_joint + dot(u_joint, un_angular_velocity)*un_angular_velocity;
            float rotation_angle = 0.1*length(omega) * length(u_joint);
            mat3 R = rotation_from_axis_angle(un_angular_velocity, -rotation_angle);

            p_skinning3d = p_skinning3d + w * (R-Id)*(p_skinning3d-rotation_center);
        }
    }

    position_transformed = Rm*Sm*vec4(p_skinning3d,1.0)+Tm;
    
    fragment.normal = Rm*p_skinning_normal;
    fragment.position = position_transformed;
    gl_Position = perspective * view * position_transformed;
}
