
"use strict";


let K0 = 1;


initEmptyScene(sceneElements);
load3DObjects(sceneElements.sceneGraph);
requestAnimationFrame( computeFrame );

// Set up resize window
window.addEventListener('resize', resizeWindow);


function changeWireframe(value)
{
    if(value===true) {
        for(let name in sceneElements.models)
        {
            sceneElements.models[name]["mesh"].material = sceneElements.param.materialWireframe;
        }
    }
    else{
        for(let name in sceneElements.models)
        {
            sceneElements.models[name]["mesh"].material = sceneElements.models[name]["material"];
        }
    }
}

function changeDisplaySkeleton(value)
{
    for(let name in sceneElements.models)
    {
        for(let k in sceneElements.models[name]["mesh"]["children"]){
            sceneElements.models[name]["mesh"]["children"][k].visible = value;
            
        }
    }

}

function changeTexture(value)
{
    for(let name in sceneElements.models)
    {
        if(value==false) {
            sceneElements.models[name]["mesh"].material.map = sceneElements.param.textureWhite;
            sceneElements.models[name]["mesh"].material.color = new THREE.Color("rgb(220, 220, 220)");
        }
        else{
            sceneElements.models[name]["mesh"].material.map = sceneElements.models[name].texture;
            sceneElements.models[name]["mesh"].material.color = new THREE.Color("rgb(255, 255, 255)");
        }

    }


}

function changeProps(value)
{
    for(let element in sceneElements.props)
    {
        const model = sceneElements.props[element];
        if(value==false) {
            model.visible = false;
        }
        else{
            model.visible = true;
        }

    }

    if(value==false) {
        sceneElements.renderer.setClearColor('rgb(255, 255, 255)', 1.0);
    }
    else{
        sceneElements.renderer.setClearColor('rgb(200, 200, 255)', 1.0);
    }
}

function changeWeight(value)
{

    for(let name in sceneElements.models)
    {
        const model = sceneElements.models[name]["mesh"];
        model.material.vertexColors = false;
        if(value==true) {
            model.material.vertexColors = true;
        }
        else{
            model.material.vertexColors = false;
        }
        model.material.needsUpdate = true
    }
}

function quaternion_to_axis_angle(q, axis, angle)
{
    axis.set(q.x, q.y, q.z);
    const n = axis.length();
    if(n<1e-6)
    {
        axis.set(1,0,0);
        angle = 0.0;
    }
    else{
        angle = 2 * Math.atan2(n, q.w);
        axis.divideScalar(n);
    }
}

function quaternion_to_angular_speed(q, W)
{
    W.set(q.x, q.y, q.z);
    const n = W.length();
    if(n<1e-6)
    {
        W.set(0,0,0);
    }
    else{
        const angle = 2 * Math.atan2(n, q.w);
        W.divideScalar(n);
        W.multiplyScalar(angle);
    }
}

function build_hierarchy_dependency(model)
 {
//     model["rig"]["vertex_depending_on_joint"] = [];
//     model["rig"]["vertex_weight_depending_on_joint"] = [];

//     const N_vertex = model["vertex_rest_pose"].length;
//     const N_joint = model["skeleton"]["position_local"].length;

//     const joint_ancestor = [];

    // for(let k_joint=0; k_joint<N_joint; ++k_joint)
    // {
    //     model["rig"]["vertex_depending_on_joint"][k] = [];
    //     model["rig"]["vertex_weight_depending_on_joint"][k] = [];

    //     joint_ancestor[k_joint] = [];
    //     let joint = k_joint;
    //     while(joint != -1)
    //     {
    //         joint_ancestor
    //         joint = model["skeleton"]["parent_id"];
    //     }
    // }

    // for(let k_vertex=0; k_vertex<N_vertex; ++k_vertex)
    // {
    //     const N_dependency = model["rig"][k_vertex].length;
    //     for(let k_joint=0; k_joint<N_dependency; ++k_joint)
    //     {
    //         const joint0 = model["rig"][k_vertex][k_joint];
    //         let current_joint = joint0;
    //         while(current_joint != -1)
    //         {

    //         }
    //     }
    // }
}

let avg_dt = 0.0;
let previous_time = 0;
function computeFrame( time ) {



    const dt = time-previous_time;
    previous_time = time;
    //stats.begin();

    if(time>2000)
    {
        avg_dt = 0.9*avg_dt + 0.1*dt;
    }
    else{
        avg_dt = 1/60;
    }

    // sceneElements.timer['interval'][1]=12;

    sceneElements.timer.current = sceneElements.timer.current + 1/50;//avg_dt/1000;//+1/50;
    if(sceneElements.timer.current>sceneElements.timer.interval[1]){
        sceneElements.timer.current = sceneElements.timer.interval[0];
    }
    //console.log(sceneElements.timer.current, sceneElements.timer.interval);






    for(let name in sceneElements.models)
    {


        const model = sceneElements.models[name];

        

        let ready = true; 
        if(!model["mesh"] || !model["animation"] || !model["skeleton"] || !model["skeleton_current"] || !model["velocity_skinning"]) { ready=false; }

        //if(model["rig"] && model["mesh"] && !model["rig"]["vertex_depending_on_joint"]) {build_hierarchy_dependency(model);}

        if(ready)
        {
            

            if(model["visible"]===false){
                model["mesh"].visible = false;
                continue;
            }
            else{
                model["mesh"].visible = true;
            }



            interpolate_skeleton_at_time(sceneElements.timer.current, model["animation"], model["skeleton_current"], model["skeleton"]["parent_id"]);


            const N_joint = model["skeleton_current"]["position_local"].length;
            
            const R_parent = new THREE.Quaternion(0,0,0,1);
            for(let k=0; k<N_joint; ++k)
            {

                const p1 = model["skeleton_current"]["position_local"][k];
                const p0 = model["velocity_skinning"]["speed_tracker"][k].last_position;
                const new_speed = new THREE.Vector3();
                new_speed.copy(p1);
                new_speed.sub(p0);
                new_speed.divideScalar(1/60.0);
                model["velocity_skinning"]["speed_tracker"][k].avg_speed.multiplyScalar(0.75);
                new_speed.multiplyScalar(1-0.75);
                model["velocity_skinning"]["speed_tracker"][k].avg_speed.add(new_speed);
                model["velocity_skinning"]["speed_tracker"][k].last_position.copy(p1);


                

                const q1 = model["skeleton_current"]["rotation_local"][k];
                const q0 = model["velocity_skinning"]["rotation_tracker"][k].last_position;
                const new_rotation_speed = new THREE.Quaternion();
                new_rotation_speed.copy(q0);
                new_rotation_speed.conjugate();
                new_rotation_speed.premultiply(q1);

                const new_rotation_speed_vec = new THREE.Vector3();
                quaternion_to_angular_speed(new_rotation_speed, new_rotation_speed_vec);
                new_rotation_speed_vec.divideScalar(1/60.0);


                new_rotation_speed_vec.multiplyScalar(1-0.75);

                model["velocity_skinning"]["rotation_tracker"][k].avg_speed.multiplyScalar(0.75);
                model["velocity_skinning"]["rotation_tracker"][k].avg_speed.add(new_rotation_speed_vec);
                model["velocity_skinning"]["rotation_tracker"][k].last_position.copy(q1);




                if(k>0)
                {
                    const parent = model["skeleton"]["parent_id"][k];
                    R_parent.copy( model["skeleton_current"]["rotation_global"][parent] );
                }
                model["velocity_skinning"]["speed_tracker"][k].current_speed.copy(model["velocity_skinning"]["speed_tracker"][k].avg_speed);
                model["velocity_skinning"]["speed_tracker"][k].current_speed.applyQuaternion(R_parent);

                model["velocity_skinning"]["rotation_tracker"][k].current_speed.copy(model["velocity_skinning"]["rotation_tracker"][k].avg_speed);
                model["velocity_skinning"]["rotation_tracker"][k].current_speed.applyQuaternion(R_parent);

            }

            // if(K0<150 && name==="cylinder"){
            // console.log(model["rotation_tracker"][1].current_speed);
            // }

            
            computeSkinning(model["vertex_skinning"], model["rig"], model["vertex_rest_pose"], model["skeleton"], model["skeleton_current"]);

            if(sceneElements.param['DeformOnly'])
            {
                computeSkinningInverse(model["vertex_skinning_inverse"], model["rig"], model["vertex_skinning"], model["skeleton"], model["skeleton_current"]);

                for(let k=0, N=model["vertex_skinning"].length; k<N; ++k) {
                    model["vertex_skinning"][k].copy(model["vertex_skinning_inverse"][k]);
                }
            }

            //computeSkinning(model["vertex_skinning_inverse"], model["rig"], model["vertex_skinning"], model["skeleton_current"], model["skeleton"]);

            // computeSkinningInverse(model["vertex_skinning_inverse"], model["rig"], model["vertex_skinning"], model["skeleton"], model["skeleton_current"]);




            for(let k=0; k<N_joint; ++k)
            {
                const p_joint = model["skeleton_current"]["position_global"][k];
                model["skeleton"]["visual_sphere"][k].position.copy(p_joint);
            }
            for(let k=1; k<N_joint; ++k)
            {
                const parent = model["skeleton"]["parent_id"][k];
                const p1 = model["skeleton_current"]["position_global"][k];
                const p0 = model["skeleton_current"]["position_global"][parent];

                model["skeletonSegments"][k].geometry.vertices[0].copy(p0);
                model["skeletonSegments"][k].geometry.vertices[1].copy(p1);
            }

            const N_vertex = model["vertex_skinning"].length;
            for(let k=0; k<N_vertex; ++k){
                model["mesh"].geometry.vertices[k].copy( model["vertex_skinning"][k] );
            }


            velocitySkinning(model);


            for(let k=0, N=model["vertex_skinning"].length; k<N; ++k) {
                model["mesh"].geometry.vertices[k].copy(model["vertex_velocity_skinning"][k]);
            }


            model["mesh"].geometry.verticesNeedUpdate = true;
            model["mesh"].geometry.computeVertexNormals()
            model["mesh"].geometry.normalsNeedUpdate = true;
            
            const N_children = model["mesh"]["children"].length;
            for(let k=0; k<N_children; ++k) {
                const child = model["mesh"]["children"][k];
                if(child.type==="Line"){
                    child.geometry.verticesNeedUpdate = true;
                }

            }

        }
    }


    K0 = K0+1;

    composer 

    render( sceneElements );


    sceneElements.control.update();

    //stats.end();

    requestAnimationFrame( computeFrame );
}


// Update render image size and camera aspect when the window is resized
function resizeWindow(eventParam) {
    const width = window.innerWidth;
    const height = window.innerHeight;

    sceneElements.camera.aspect = width / height;
    sceneElements.camera.updateProjectionMatrix();

    sceneElements.renderer.setSize(width, height);
}





function render(sceneElements) {

    if(sceneElements.param.ssao===true){
        composer.render();
    }
    else{
        sceneElements.renderer.render(sceneElements.sceneGraph, sceneElements.camera);
    }
    


}


 






// Create and insert in the scene graph the shapes of the 3D scene
function load3DObjects(sceneGraph) {


    const loader = new THREE.TextureLoader();
    sceneElements.param.textureWhite = loader.load("assets/texture_white.png");
    




    gui.add(sceneElements.param, 'Activated').name("VelocitySkinning");
    gui.add(sceneElements.param, 'DeformOnly').name("RestShape");

    let obj = { Select:function(){viewAllScene()} };
    gui.add(obj,'Select').name( 'Select All');

    gui.add(sceneElements.param, 'Flappy', 0, 3.0);
    gui.add(sceneElements.param, 'Squashy', 0, 3.0);

    const Id = new THREE.Quaternion();
    const R = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-3.14/2.0);
    
    const R_cow = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-5*3.14/6.0) . multiply(R);
    loadAsset("cow", [-2,0.75,2], 1.0, R_cow, false);
    sceneElements.models["cow"].param["translation"] = [-2,0.75,2];
    sceneElements.models["cow"].param["squashy"] = 0.2;

    //const R_cow_up = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-7*3.14/6.0);
    //loadAsset("cow_up", [3,1.3,2], 1.0, R_cow_up);

    const R_dragon = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-3.14/4.0) . multiply(R);
    loadAsset("dragon", [2.0,1.05,-0.5], 0.8, R_dragon, false);
    sceneElements.models["dragon"].param["translation"] = [2.0,0.5,-0.5];
    sceneElements.models["dragon"].param["squashy"] = 0;

    //const R_dragon2 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-3.14/4.0) . multiply(R);
    //loadAsset("dragon2", [2.5,0.9,-0.5], 0.7, R_dragon);
    

    
    // loadAsset("cylinder", [0,0,0], R);

    loadAsset("giraffe", [1,0,3], 1.3, Id, true);
    sceneElements.models["giraffe"]["param"]["flappy"] = 0.7;
    sceneElements.models["giraffe"].param["translation"] = [1,1.3,3];
    sceneElements.models["giraffe"].param["squashy"] = 0.1;


    const R_bird = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-5*3.14/7.0).multiply(R);
    loadAsset("bird", [1.5,2.5,0], 1.0, R_bird);
    sceneElements.models["bird"]["param"]["squashy"] = 0.5;
    sceneElements.models["bird"]["param"]["flappy"] = 0.5;
    sceneElements.models["bird"].param["translation"] = [1.5,2.5,0];

    


    const R_sphere = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),3.14/2.0);
    loadAsset("sphere", [-0.5,0.35,0], 1.6, R_sphere, false);
    sceneElements.models["sphere"]["param"]["flappy"] = 0.0;
    sceneElements.models["sphere"]["param"]["squashy"] = 1.25;
    sceneElements.models["sphere"].param["translation"] = [-0.5,0.35,0];





    //loadAsset("snail", [2,0.75,2], Id);



    let fbx_loader = new THREE.FBXLoader();
    fbx_loader.load( 'assets/flower/sunflower.fbx', function ( object ) {

        object.scale.copy(new THREE.Vector3(0.3,0.3,0.3));
        object.translateY(-1.0).translateZ(-2.5).translateX(-2.25);
        sceneGraph.add( object );
        sceneElements.props["flower"]=object;
    
    } );

    fbx_loader.load( 'assets/tree/Boom_Final.fbx', function ( object ) {

        object.scale.copy(new THREE.Vector3(0.004,0.004,0.004));
        object.translateY(-1.0).translateZ(-3.5).translateX(1.5);
        sceneGraph.add( object );

        const loader = new THREE.TextureLoader();
        const texture = loader.load('assets/tree/Boom_Final_UV.png');

        const R = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),3.14/2.0+3.14/8.0);

        object.children[0].material.map = texture;
        object.children[0].castShadow = true;
        object.setRotationFromQuaternion(R);
           
        sceneElements.props["tree"]=object;
    } );


    gui.add(sceneElements.timer, "current", 0, sceneElements.timer["interval"][1]).listen().name("Time Cycle");

    const folderDisplay = gui.addFolder(`Display`)


    const materialWireframe = new THREE.MeshBasicMaterial( {color:'rgb(255, 255, 255)',  side:THREE.DoubleSide, wireframe:true} );
    sceneElements.param["materialWireframe"] = materialWireframe;
    folderDisplay.add(sceneElements.param, "Wireframe").listen().onChange(changeWireframe);

    folderDisplay.add(sceneElements.param, "displaySkeleton").name("Skeleton").listen().onChange(changeDisplaySkeleton);
    

    folderDisplay.add(sceneElements.param, "texture").listen().onChange(changeTexture);
    folderDisplay.add(sceneElements.param, "background").listen().onChange(changeProps);
    folderDisplay.add(sceneElements.param, "weights").name("Floppy weights").listen().onChange(changeWeight);
    folderDisplay.add(sceneElements.param, "ssao").name("SSAO");
    


    
    
    console.log(sceneElements);
    sceneElements.timer.current = sceneElements.timer.interval[0];

    loadVisualElementHelper(sceneGraph);
    createBasicElements(sceneGraph);


}






