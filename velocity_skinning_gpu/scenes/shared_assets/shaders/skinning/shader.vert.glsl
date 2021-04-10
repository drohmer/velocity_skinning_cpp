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

uniform mat4 skeleton[60];
uniform mat4 skeleton_rest[60];

uniform samplerBuffer tbo_weights;

void main()
{
    // scaling matrix
    mat4 S = mat4(scaling*scaling_axis.x,0.0,0.0,0.0, 0.0,scaling*scaling_axis.y,0.0,0.0, 0.0,0.0,scaling*scaling_axis.z,0.0, 0.0,0.0,0.0,1.0);
    // 4x4 rotation matrix
    mat4 R = mat4(rotation);
    // 4D translation
    vec4 T = vec4(translation,0.0);


    fragment.color = color;
    fragment.texture_uv = texture_uv;

    fragment.normal = R*normal;
    vec4 position_transformed = R*S*position + T;// + vec4(skeleton[0][0][3],skeleton[0][1][3],skeleton[0][2][3],0.0);

    vec4 p_skinning = vec4(0.0, 0.0, 0.0, 0.0);
    for(int k_joint=0; k_joint<N_joint; k_joint++)
    {
        mat4 T = transpose(skeleton[k_joint]);
        mat4 T0 = transpose(skeleton_rest[k_joint]);

        //float w = texture(texture_weights, vec2( (gl_VertexID+0.5)/(N_vertex-0.0), (k_joint+0.5)/(N_joint-0.0))).r;
        //float w = texelFetch(texture_weights,  ivec2(gl_VertexID,k_joint));
        float w = texelFetch(tbo_weights, gl_VertexID+k_joint*N_vertex).r;
        p_skinning = p_skinning + w * T * inverse(T0) * position_transformed;
    }
    position_transformed = p_skinning;

    fragment.position = position_transformed;
    gl_Position = perspective * view * position_transformed;
}
