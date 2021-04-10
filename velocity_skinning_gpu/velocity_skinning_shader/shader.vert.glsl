#version 330 core


layout (location = 0) in vec4 position;
layout (location = 1) in vec4 normal;
layout (location = 2) in vec2 texture_uv;

out struct fragment_data
{
    vec4 position;
    vec4 normal;
    vec4 color;
    vec4 position_cam_space;
    float floppy_weight;
    vec2 texture_uv;
} fragment;


// model transformation
uniform vec3 translation = vec3(0.0, 0.0, 0.0);                      // user defined translation
uniform mat3 rotation = mat3(1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0); // user defined rotation
uniform float scaling = 1.0;                                         // user defined scaling
uniform vec3 scaling_axis = vec3(1.0,1.0,1.0);                       // user defined scaling
uniform int N_vertex;
uniform int N_joint;
uniform float floppy_power = 1.0;
uniform float squashy_power = 1.0;

// view transform
uniform mat4 view;
// perspective matrix
uniform mat4 perspective;




// Rest pose Skeleton
uniform samplerBuffer tbo_sk0; // Rest-pose skeleton bones positions


// Data used for interactive mode (updated at each frame)
uniform samplerBuffer tbo_sk;  // Current skeleton bones positions
uniform samplerBuffer tbo_angular_velocity; // Current angular velocity of each bone
uniform samplerBuffer tbo_linear_velocity;  // Current linear velocity of each bone


// Skinning weights
uniform isamplerBuffer texture_rig_joint; // Skinning rig - joints dependence
uniform samplerBuffer texture_rig_weight; // Skinning rig - skinning weights
uniform isamplerBuffer texture_rig_cumulative_index; // Skinning rig - index in lookup-table
uniform isamplerBuffer texture_rig_size;             // Skinning rig - number of dependance for a given vertex

// Velocity weights
uniform isamplerBuffer texture_vs_rig_joint; // Velocity Skinning rig - joints dependence
uniform samplerBuffer texture_vs_rig_weight; // Velocity Skinning rig - weights
uniform isamplerBuffer texture_vs_rig_cumulative_index; // Velocity Skinning rig - index in lookup-table
uniform isamplerBuffer texture_vs_rig_size;             // Skinning rig - number of dependance for a given vertex


uniform samplerBuffer texture_floppy_weights_raw; // per-vertex floppy weights
uniform samplerBuffer texture_center_of_mass;  // Center of mass (in rest pose)

// Only used for instancing (cow meadow)
uniform samplerBuffer texture_position_cow; // Instancing - position of the shape
uniform samplerBuffer texture_angle_cow;    // Instancing - orientation of the shape

// Only used for recorded animation
uniform samplerBuffer texture_T_anim;      // Animation - deformation matrices
uniform samplerBuffer texture_angular_velocity_anim; // Animation - angular velocities
uniform samplerBuffer texture_linear_velocity_anim; // Animation - linear velocities

// switch to recorded animation
uniform bool interactive_mode = true; 

uniform bool floppy_weight_active = false;
uniform bool floppy_limit = false;
uniform bool gpu_velocity_skinning = true;

uniform float t;
uniform int N_time;


int shuffle_time = 0;


mat3 rotation_from_axis_angle(vec3 u, float angle)
{
    float x = u.x;
    float y = u.y;
    float z = u.z;
    float c = cos(angle);
    float s = sin(angle);

    mat3 R = mat3(vec3(c+x*x*(1-c)  , y*x*(1-c)+z*s, z*x*(1-c)-y*s), vec3(x*y*(1-c)-z*s, c+y*y*(1-c), z*y*(1-c)+x*s), vec3(x*z*(1-c)+y*s, y*z*(1-c)-x*s, c+z*z*(1-c)) );
    return R;
}


mat3 rotation_between_vector(vec3 a, vec3 b)
{
    vec3 u0 = normalize(a);
    vec3 u1 = normalize(b);

    if( length(u0-u1)<1e-4f || length(u0+u1)<1e-4f )
        return mat3( vec3(1,0,0), vec3(0,1,0), vec3(0,0,1) );

    float d = dot(u0,u1);
    float angle = acos( d );

    vec3 axis = normalize(cross(u0,u1));

    return rotation_from_axis_angle(axis,angle);
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



void global_transform(out mat4 Sm, out mat4 Rm, out vec4 Tm)
{
    // Global transform on the model
    Sm = mat4(scaling*scaling_axis.x,0.0,0.0,0.0, 0.0,scaling*scaling_axis.y,0.0,0.0, 0.0,0.0,scaling*scaling_axis.z,0.0, 0.0,0.0,0.0,1.0);
    Rm = mat4(rotation);
    Tm = vec4(translation,0.0);
}

void global_transform_instances(inout mat4 Sm, inout mat4 Rm, inout vec4 Tm)
{
    vec3 translate_instance = vec3(texelFetch(texture_position_cow, 3*gl_InstanceID).r, texelFetch(texture_position_cow, 3*gl_InstanceID+1).r, texelFetch(texture_position_cow, 3*gl_InstanceID+2).r);
    Tm += vec4(translate_instance,0.0);
    float angle_instance = texelFetch(texture_angle_cow,gl_InstanceID).r;
    mat4 R_instance = mat4(vec4(cos(angle_instance),sin(angle_instance),0.0,0.0),vec4(-sin(angle_instance),cos(angle_instance),0.0,0.0),vec4(0.0,0.0,1.0,0.0),vec4(0.0,0.0,0.0,1.0));
    Rm = R_instance * Rm;
}


//**********************************************************************//
//*                 Linear Blend Skinning                             **//
//**********************************************************************//

void LBS(in vec4 p0, in vec4 n0, inout vec4 p_skinning, inout vec4 p_skinning_normal)
{
    p_skinning = vec4(0.0);
    p_skinning_normal = vec4(0.0);

    int offset = texelFetch(texture_rig_cumulative_index, gl_VertexID).r;
    int N_dependency = texelFetch(texture_rig_size, gl_VertexID).r;
    for(int k_dep=0; k_dep<N_dependency; ++k_dep)
    {
        int joint = texelFetch(texture_rig_joint, offset+k_dep).r;
        float w = texelFetch(texture_rig_weight, offset+k_dep).r;

        mat4 T;
        if(interactive_mode){
            T = mat4_from_texture(tbo_sk, joint);
        }
        else {
            T = mat4_from_texture(texture_T_anim, joint + (N_joint * int(shuffle_time)) );
        }
        mat4 T0 = mat4_from_texture(tbo_sk0, joint);
        
        p_skinning += w * T * T0 * p0;
        p_skinning_normal += w * T * T0 * n0;
    }
    p_skinning_normal = normalize(p_skinning_normal);
    
}

//**********************************************************************//
//*                 Velocity Skinning Deformers                       **//
//**********************************************************************//
vec3 velocity_skinning_floppy_rotation(vec3 omega,vec3 p_joint,vec3 p_vertex, float floppy_magnitude)
{
    vec3 un_angular_velocity = normalize(omega);
    vec3 u_joint = p_vertex - p_joint;
    vec3 rotation_center = p_joint + dot(u_joint, un_angular_velocity)*un_angular_velocity;
    float vertex_speed_norm = length(omega) * length(u_joint);
    float rotation_angle = floppy_magnitude * vertex_speed_norm;
            
    float m = 3.14/12.0;
    if(rotation_angle>m && floppy_limit==true)
        rotation_angle = m + (rotation_angle-m)/vertex_speed_norm;

    mat3 R = rotation_from_axis_angle(un_angular_velocity, -rotation_angle);
    mat3 Id = mat3(1.0);

    return (R-Id)*(p_vertex-rotation_center);
}

vec3 velocity_skinning_squashy_rotation(vec3 omega, vec3 p_joint, vec3 p_vertex, float squashy_magnitude, vec3 center_of_mass)
{
    vec3 u_joint = p_vertex - p_joint;
    float vertex_speed_norm = length(omega) * length(u_joint);

    float squash_factor = squashy_magnitude * vertex_speed_norm;
    float elongation = 1+squash_factor;
    float squeeze = 1/(1+squash_factor);
    mat3  S = mat3( vec3(elongation,0,0), vec3(0,squeeze,0), vec3(0,0,squeeze) );

    // Rotate to correct frame
    vec3 un_angular_velocity = normalize(omega);
    vec3 un_medial = normalize(center_of_mass - p_joint);
    vec3 u_elongation = cross(un_medial, un_angular_velocity);
    if( length(u_elongation)< 1e-2f) {
        return vec3(0.0);
    }

    vec3 un_elongation = normalize(u_elongation);
    vec3 un_squeeze = cross(un_medial, un_elongation);
    mat3 R = mat3(un_elongation, un_squeeze, un_medial);

    // Deformation matrix
    mat3 T = R*S*transpose(R);
    mat3 Id = mat3(1.0);

    // Center of scaling
    vec3 p_medial = p_joint + dot(p_vertex-p_joint,un_medial)*un_medial;

    return (T-Id) * (p_vertex-p_medial);
}

vec3 velocity_skinning_floppy_translation(float floppy_magnitude, vec3 linear_speed)
{
    return -floppy_magnitude * linear_speed;
}

vec3 velocity_skinning_squashy_translation(float squashy_magnitude, vec3 linear_speed, vec3 center_of_mass, vec3 p_vertex)
{
    float squash_factor = squashy_magnitude * length(linear_speed);
    float elongation_scaling = 1+squash_factor;
    float squeeze_scaling = 1/sqrt(1+squash_factor);

    mat3 Id = mat3(1.0);
    mat3 S = mat3(vec3(elongation_scaling,0,0), vec3(0,squeeze_scaling,0), vec3(0,0,squeeze_scaling)); // Scaling
    mat3 R = rotation_between_vector(vec3(1,0,0), normalize(linear_speed) ); // Rotation to the correct frame
    mat3 T = R * S * transpose(R); // Deformation matrix

    return (T-Id)*(p_vertex-center_of_mass);
}


void retrieve_joint_info(in int k_joint, out mat4 T, out vec3 angular_velocity, out vec3 linear_velocity)
{
    if(interactive_mode) {
        angular_velocity = vec3(texelFetch(tbo_angular_velocity, 3*k_joint+0).r,texelFetch(tbo_angular_velocity, 3*k_joint+1).r,texelFetch(tbo_angular_velocity, 3*k_joint+2).r);//+0.01*angular_velocity[k_joint];
        linear_velocity = vec3(texelFetch(tbo_linear_velocity, 3*k_joint+0).r,texelFetch(tbo_linear_velocity, 3*k_joint+1).r,texelFetch(tbo_linear_velocity, 3*k_joint+2).r);//+0.01*angular_velocity[k_joint];
        T = mat4_from_texture(tbo_sk, k_joint);
    }
    else {
        angular_velocity = vec3(texelFetch(texture_angular_velocity_anim, (3*k_joint+0)+3*N_joint*int(shuffle_time)).r,texelFetch(texture_angular_velocity_anim, 3*k_joint+1+3*N_joint*int(shuffle_time)).r,texelFetch(texture_angular_velocity_anim, 3*k_joint+2+3*N_joint*int(shuffle_time)).r);
        linear_velocity = vec3(texelFetch(texture_linear_velocity_anim, (3*k_joint+0)+3*N_joint*int(shuffle_time)).r,texelFetch(texture_linear_velocity_anim, 3*k_joint+1+3*N_joint*int(shuffle_time)).r,texelFetch(texture_linear_velocity_anim, 3*k_joint+2+3*N_joint*int(shuffle_time)).r);
        T = mat4_from_texture(texture_T_anim, k_joint + (N_joint * int(shuffle_time)) );
    }
}


void velocity_skinning(in float floppy_weight_vertex, in vec3 p_skinning3d, inout vec3 p_deformed3d)
{
    int vs_offset = texelFetch(texture_vs_rig_cumulative_index, gl_VertexID).r;
    int vs_N_dependency = texelFetch(texture_vs_rig_size, gl_VertexID).r;

    for(int k_dep=0; k_dep<vs_N_dependency; ++k_dep)
    {
        int k_joint = texelFetch(texture_vs_rig_joint, vs_offset+k_dep).r; // current joint
        float w = texelFetch(texture_vs_rig_weight, vs_offset+k_dep).r;    // velocity-skinning weight
        vec3 center_of_mass = vec3(texelFetch(texture_center_of_mass, 3*k_joint+0).r,texelFetch(texture_center_of_mass, 3*k_joint+1).r,texelFetch(texture_center_of_mass, 3*k_joint+2).r);

        mat4 T;vec3 angular_velocity, linear_velocity; // position and velocity of joint
        retrieve_joint_info(k_joint, T, angular_velocity, linear_velocity);

        // compute current position of center of mass
        mat4 T0 = mat4_from_texture(tbo_sk0, k_joint);
        center_of_mass = vec3(T*T0*vec4(center_of_mass,1.0));

        float floppy_magnitude = 0.2 * floppy_weight_vertex * floppy_power;
        float squashy_magnitude = 0.05 * squashy_power;

        // If there is some rotation
        if( length(angular_velocity)>0.001 ){
            
            vec3 p_joint = translation_from_mat4_texture(tbo_sk, k_joint);

            // Compute Floppy and squashy deformation for rotation motion
            vec3 deformation_floppy  = velocity_skinning_floppy_rotation(angular_velocity, p_joint, p_skinning3d, floppy_magnitude);
            vec3 deformation_squashy  = velocity_skinning_squashy_rotation(angular_velocity, p_joint, p_skinning3d, squashy_magnitude, center_of_mass);
              
            p_deformed3d += w * (deformation_floppy + deformation_squashy);
        }

        // If there is some translation
        if( length(linear_velocity)>1e-4 )
        {
            vec3 deformation_floppy = velocity_skinning_floppy_translation(floppy_magnitude, linear_velocity);
            vec3 deformation_squashy = velocity_skinning_squashy_translation(squashy_magnitude, linear_velocity, center_of_mass, p_skinning3d);

            p_deformed3d += w * (deformation_floppy + deformation_squashy);
        }
    }
}

void main()
{

    mat4 Sm = mat4(1.0); 
    mat4 Rm = mat4(1.0); 
    vec4 Tm = vec4(0.0);
    global_transform(Sm, Rm, Tm);
    global_transform_instances(Sm, Rm, Tm);
    




    // Shuffle time for instanciated anim
    shuffle_time = int( mod(t+gl_InstanceID,N_time-1));// int(mod(int(t)+23154*gl_InstanceID+gl_InstanceID*15,N_time-1));

    // Compute Linear Blend Skinning
    vec4 p_skinning = position;
    vec4 p_skinning_normal = vec4(normal.xyz,0.0);
    LBS(position, vec4(normal.xyz,0.0), p_skinning, p_skinning_normal);
    vec3 p_skinning3d = p_skinning.xyz;
    vec3 p_deformed3d = p_skinning3d;

    
    // Compute Velocity Skinning
    float floppy_weight_vertex = 1.0f;
    
    if(gpu_velocity_skinning==true)
    {
        if(floppy_weight_active) {
            floppy_weight_vertex = texelFetch(texture_floppy_weights_raw,gl_VertexID).r;
        }

        velocity_skinning(floppy_weight_vertex,p_skinning3d,p_deformed3d);
    }
    
    
    

    p_skinning_normal = normalize(p_skinning_normal);
    vec4 position_deformed = Rm*Sm*vec4(p_deformed3d,1.0)+Tm;
    fragment.normal = Rm*p_skinning_normal;
    fragment.position = position_deformed;
    fragment.color = vec4(floppy_weight_vertex,0,0,0);
    fragment.position_cam_space = view * position_deformed;
    fragment.texture_uv = texture_uv;
    fragment.floppy_weight = floppy_weight_vertex;
    gl_Position = perspective * fragment.position_cam_space;
}
