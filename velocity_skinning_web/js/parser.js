"use strict";

function setUniqueModel(name)
{
    for(let n in sceneElements.models){
        sceneElements.models[n]["visible"] = false;
    }
    for(let o in sceneElements.props){
        sceneElements.props[o]["visible"] = false;
    }
    sceneElements.models[name]["visible"] = true;

    //let y = sceneElements.camera.position.y;
    //sceneElements.camera.position.set(1,y,3);
    //sceneElements.camera.target.set(0, 0, 3);
    const t = sceneElements.models[name].param["translation"];
    sceneElements.control.target.set(t[0],t[1],t[2]);

    //sceneElements.camera.updateProjectionMatrix();

}
function viewAllScene()
{
    for(let o in sceneElements.props){
        sceneElements.props[o]["visible"] = true;
    }
    for(name in sceneElements.models) {
        sceneElements.models[name]["visible"] = true;
    }
}

function loadAsset(name, translation, scaling, rotation, inverse_normals)
{
    const dir = "assets/"+name+"/";

    sceneElements.models[name] = {};

    fetch(dir+"/mesh.json")
    .then(response => response.json())
    .then( json => 
        {
            const mesh_elements = meshFromJson(json, dir+"/texture.png");
            sceneElements.models[name]["mesh"]=mesh_elements["mesh"];
            sceneElements.models[name]["vertex_rest_pose"] = mesh_elements["vertices"];
            
            sceneElements.models[name]["vertex_skinning"] = [];
            for(let k=0; k<mesh_elements["vertices"].length; ++k) {
                sceneElements.models[name]["vertex_skinning"].push(new THREE.Vector3().copy(mesh_elements["vertices"][k]));
            }
            
            sceneElements.models[name]["vertex_skinning_inverse"] = [];
            for(let k=0; k<mesh_elements["vertices"].length; ++k) {
                sceneElements.models[name]["vertex_skinning_inverse"].push(new THREE.Vector3().copy(mesh_elements["vertices"][k]));
            }

            sceneElements.models[name]["vertex_velocity_skinning"] = [];
            for(let k=0; k<mesh_elements["vertices"].length; ++k) {
                sceneElements.models[name]["vertex_velocity_skinning"].push(new THREE.Vector3().copy(mesh_elements["vertices"][k]));
            }

            sceneElements.models[name]["velocity_skinning_deformation"] = [];
            for(let k=0; k<mesh_elements["vertices"].length; ++k) {
                sceneElements.models[name]["velocity_skinning_deformation"].push(new THREE.Vector3());
            }


            sceneElements.sceneGraph.add( sceneElements.models[name]["mesh"] );

            sceneElements.models[name]["mesh"].translateX(translation[0]);
            sceneElements.models[name]["mesh"].translateY(translation[1]);
            sceneElements.models[name]["mesh"].translateZ(translation[2]);

            sceneElements.models[name]["mesh"].setRotationFromQuaternion(rotation);

            sceneElements.models[name]["mesh"].scale.copy(new THREE.Vector3(scaling, scaling, scaling));
         
            sceneElements.models[name]["material"] = sceneElements.models[name]["mesh"].material;

            sceneElements.models[name]["texture"] = sceneElements.models[name]["mesh"].material.map;




        fetch(dir+"skeleton.json")
        .then(response => response.json())
        .then( json => {
            sceneElements.models[name]["skeleton"]=skeletonFromJson(json);

            const N_joint = sceneElements.models[name]["skeleton"]["position_global"].length;



            sceneElements.models[name]["skeleton_current"] = {};
            sceneElements.models[name]["skeleton_current"]["position_local"] = [];
            sceneElements.models[name]["skeleton_current"]["rotation_local"] = [];
            sceneElements.models[name]["skeleton_current"]["position_global"] = [];
            sceneElements.models[name]["skeleton_current"]["rotation_global"] = [];

            for(let k=0; k<N_joint; ++k)
            {
                sceneElements.models[name]["skeleton_current"]["position_local"][k] = new THREE.Vector3();
                sceneElements.models[name]["skeleton_current"]["position_global"][k] = new THREE.Vector3();

                sceneElements.models[name]["skeleton_current"]["rotation_local"][k] = new THREE.Quaternion();
                sceneElements.models[name]["skeleton_current"]["rotation_global"][k] = new THREE.Quaternion();
            }
            

            
            const sphereMaterial = new THREE.MeshPhongMaterial( {color:'rgb(255, 0, 255)'} ); 
            sceneElements.models[name]["skeleton"]["visual_sphere"] = [];

            for(let k=0; k<N_joint; ++k)
            {
                const p_joint = sceneElements.models[name]["skeleton"]["position_global"][k];
                let sphereGeometry = new THREE.SphereBufferGeometry(0.03, 32,32);
                let sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
                
                sceneElements.models[name]["mesh"].add( sphere );
                sphere.position.copy(p_joint);

                sceneElements.models[name]["skeleton"]["visual_sphere"][k] = sphere;
                sphere.visible = false;

            }

            sceneElements.models[name]["skeletonSegments"] = [];
            for(let k=1; k<N_joint; ++k)
            {
                const parent = sceneElements.models[name]["skeleton"]["parent_id"][k];
                const p1 = sceneElements.models[name]["skeleton"]["position_global"][k];
                const p0 = sceneElements.models[name]["skeleton"]["position_global"][parent];


                let geometry = new THREE.Geometry().setFromPoints( [p0,p1] );

                let material = new THREE.LineBasicMaterial( { color: 0x0000ff } );
                let line = new THREE.Line( geometry, material );
                sceneElements.models[name]["mesh"].add(line);
                sceneElements.models[name]["skeletonSegments"][k] = line;
                line.visible = false;
            }


        } );



        fetch(dir+"/velocity_skinning.json")
        .then(response => response.json())
        .then( json => {
            sceneElements.models[name]["velocity_skinning"]=velocitySkinningFromJson(json);
    
            const N_joint = sceneElements.models[name]["velocity_skinning"]["vertex_depending_on_joint"].length;
    
            sceneElements.models[name]["velocity_skinning"]["speed_tracker"] = [];
            sceneElements.models[name]["velocity_skinning"]["rotation_tracker"] = [];
            for(let k=0; k<N_joint; k++)
            {
                sceneElements.models[name]["velocity_skinning"]["speed_tracker"][k] = {
                    last_position : new THREE.Vector3(),
                    avg_speed : new THREE.Vector3(),
                    last_time : 0.0,
                    current_speed : new THREE.Vector3(),
                };
    
                sceneElements.models[name]["velocity_skinning"]["rotation_tracker"][k] = {
                    last_position : new THREE.Quaternion(),
                    avg_speed : new THREE.Vector3(),
                    last_time : 0.0,
                    current_speed : new THREE.Vector3(),
                };
            }


            //Vertex color
            const flappy_weight = sceneElements.models[name]["velocity_skinning"].flappy_weight;
            const vertex_color = sceneElements.models[name]["mesh"].geometry.colors;
            const faces = sceneElements.models[name]["mesh"].geometry.faces

            for(let k=0; k<faces.length; ++k){
                const a = faces[k].a;
                const b = faces[k].b;
                const c = faces[k].c;

                const fa = flappy_weight[a];
                const ra = 1-Math.max(fa,0);
                const ga = 1+Math.min(fa,0);

                const fb = flappy_weight[b];
                const rb = 1-Math.max(fb,0);
                const gb = 1+Math.min(fb,0);

                const fc = flappy_weight[c];
                const rc = 1-Math.max(fc,0);
                const gc = 1+Math.min(fc,0);

                //console.log(faces);
                faces[k].vertexColors[0] = new THREE.Color(ra,ga,1.0);
                faces[k].vertexColors[1] = new THREE.Color(rb,gb,1.0);
                faces[k].vertexColors[2] = new THREE.Color(rc,gc,1.0);
                //flappy_weight
            }


            sceneElements.models[name]["mesh"].geometry.elementsNeedUpdate = true;
            //sceneElements.models[name]["mesh"].material.vertexColors = 
            
    
        } );

    }   
    );

    fetch(dir+"rig.json")
    .then(response => response.json())
    .then( json => {
        sceneElements.models[name]["rig"]=rigFromJson(json); } );

    fetch(dir+"anim.json")
    .then(response => response.json())
    .then( json => {
        sceneElements.models[name]["animation"]=animFromJson(json);

        // const N_time = sceneElements.models[name]["animation"]["time"].length;
        // const tmax = sceneElements.models[name]["animation"]["time"][N_time-1];
        // const tmin = sceneElements.models[name]["animation"]["time"][0];
        // sceneElements.timer.interval[1] = Math.max(sceneElements.timer.interval[1], tmax);
        // sceneElements.timer.interval[0] = Math.min(sceneElements.timer.interval[0], tmin);
    } );





    


    sceneElements.models[name]["select"] = true;

    sceneElements.models[name]["param"] = {'flappy':1.0, 'squashy':1.0};

    let folder = gui.addFolder(name);
    let obj = { Select:function(){setUniqueModel(name)} };
    folder.add(obj, "Select").name("Select "+name);


    folder.add(sceneElements.models[name]["param"], 'flappy', 0.0, 3.0) . listen();
    folder.add(sceneElements.models[name]["param"], 'squashy', 0.0, 3.0) . listen();

    
    

}

function skeletonFromJson(data)
{
    const skeleton = {};

    const position_local_json = data["translation"]
    const position_local = []
    for(let k=0; k<position_local_json.length/3; k++)
    {
        const p = new THREE.Vector3(position_local_json[3*k+0],position_local_json[3*k+1],position_local_json[3*k+2]);
        position_local.push(p);
    }

    const rotation_local_json = data["rotation"]
    const rotation_local = []
    for(let k=0; k<rotation_local_json.length/4; k++)
    {
        const p = new THREE.Quaternion(rotation_local_json[4*k+0],rotation_local_json[4*k+1],rotation_local_json[4*k+2],rotation_local_json[4*k+3]);
        rotation_local.push(p);
    }

    skeleton["position_local"] = position_local;
    skeleton["rotation_local"] = rotation_local;
    skeleton["parent_id"] = data["parent_id"];
    skeleton["names"] = data["names"];

    const skeleton_global = local_to_global(skeleton["position_local"], skeleton["rotation_local"], skeleton["parent_id"]);
    //console.log(skeleton_global);

    skeleton["position_global"] = skeleton_global["position"];
    skeleton["rotation_global"] = skeleton_global["rotation"];
    
    return skeleton;
}

function meshFromJson(data, texturePath)
{
    const geometry = new THREE.Geometry();
    const vertices = []

    const v = data["vertices"];
    const c = data["connectivity"];
    const uv = data["uv"];
    for(let k=0; k<v.length/3; k++)
    {
        const p = new THREE.Vector3(v[3*k+0], v[3*k+1], v[3*k+2])
        geometry.vertices.push(p);
        vertices.push(p.clone());
    }


    for(let k=0; k<c.length/3; k++)
    {
        const f = new THREE.Face3(c[3*k+0], c[3*k+1], c[3*k+2])
        geometry.faces.push(f);
        // f.vertexColors[0] = new THREE.Color('rgb(255,0,0)');
        // f.vertexColors[1] = new THREE.Color('rgb(255,0,0)');
        // f.vertexColors[2] = new THREE.Color('rgb(255,0,0)');
    }


    var geom = new THREE.PlaneGeometry( 5, 20, 32 );


    for(let k=0; k<c.length/3; k++)
    {
        const k0 = c[3*k+0];
        const k1 = c[3*k+1];
        const k2 = c[3*k+2];

        const t0 = new THREE.Vector2(uv[2*k0+0],1-uv[2*k0+1]);
        const t1 = new THREE.Vector2(uv[2*k1+0],1-uv[2*k1+1]);
        const t2 = new THREE.Vector2(uv[2*k2+0],1-uv[2*k2+1]);

        geometry.faceVertexUvs[0].push([t0,t1,t2]);
    }

    

    geometry.computeVertexNormals();

    const loader = new THREE.TextureLoader();
    const texture = loader.load(texturePath);
    const material = new THREE.MeshPhongMaterial( {color:'rgb(255, 255, 255)',  side:THREE.DoubleSide, map:texture, vertexColors: false} );  

     

    const mesh = new THREE.Mesh( geometry, material );

    
    //mesh.receiveShadow = true;
    mesh.castShadow = true;



    return {"mesh":mesh, "vertices":vertices};
}

function rigFromJson(data)
{ 
    const rig = {};
    rig["joint"] = data["joint"];
    rig["weight"] = data["weight"];

    return rig;
}

function array_of_value_to_array_of_vector3(data)
{
    const a = [];
    const N = data.length/3;
    for(let k=0; k<N; k++)
    {
        a.push(new THREE.Vector3(data[3*k+0], data[3*k+1], data[3*k+2]));
    }
    return a;
} 
function array_of_value_to_array_of_quaternion(data)
{
    const a = [];
    const N = data.length/4;
    for(let k=0; k<N; k++)
    {
        a.push(new THREE.Quaternion(data[4*k+0], data[4*k+1], data[4*k+2], data[4*k+3]));
    }
    return a;
}    

function animFromJson(data)
{
    const position = [];
    const rotation = [];
    const N_time = data["time"].length;
    for(let kt=0; kt<N_time; kt++)
    {
        position.push(array_of_value_to_array_of_vector3(data["position"][kt]));
        rotation.push(array_of_value_to_array_of_quaternion(data["rotation"][kt]));
    }


    return {"time":data["time"], "position":position, "rotation":rotation};

}

function velocitySkinningFromJson(data)
{
    
    const velocitySkinning = {};
    velocitySkinning["vertex_depending_on_joint"] = data["vertex_depending_on_joint"];
    velocitySkinning["vertex_weight_depending_on_joint"] = data["vertex_weight_depending_on_joint"];

    velocitySkinning["flappy_weight"] = data["flappy_weights"];

    velocitySkinning["center_of_mass"] = [];
    const N_joint = data["vertex_depending_on_joint"].length;
    for(let k_joint=0; k_joint<N_joint; ++k_joint)  {
        velocitySkinning["center_of_mass"][k_joint] = new THREE.Vector3();
    }
    // velocitySkinning["center_of_mass"] = [];
    // const N_time = data["center_of_mass"];
    // for(let k_time=0; k_time<N_time; ++k_time)
    // {
    //     const com
    // }
    // const N_joint = data["vertex_depending_on_joint"].length;
    // for(let k_joint=0; k_joint<N_joint; ++k_joint)

    return velocitySkinning;
}