"use strict";


function interpolate_skeleton_at_time(t, anim, skeleton_destination, parent_id)
{

    const N_joint = anim["position"][0].length;

    console.assert(N_joint>=1);
    console.assert(skeleton_destination["position_local"]);
    console.assert(skeleton_destination["position_local"].length==N_joint);


    // if(skeleton_destination["position_local"].length != N_joint)
    // {
    //     skeleton_destination["position_local"] = [];
    //     skeleton_destination["rotation_local"] = [];
    
    //     for(let k=0; k<N_joint; k++){
    //         skeleton_destination["position_local"].push(new THREE.Vector3());
    //         skeleton_destination["rotation_local"].push(new THREE.Quaternion());
    //     }
    // }

    

    const times = anim["time"];
    const N_time = times.length;
    if(t<times[0])
    {
        for(let k=0; k<N_joint; k++)
        {
            skeleton_destination["position_local"][k].copy(anim["position"][0][k]);
            skeleton_destination["rotation_local"][k].copy(anim["rotation"][0][k]);
        }
    }
    else if(t>=times[N_time-1])
    {
        for(let k=0; k<N_joint; k++)
        {
            skeleton_destination["position_local"][k].copy(anim["position"][N_time-1][k]);
            skeleton_destination["rotation_local"][k].copy(anim["rotation"][N_time-1][k]);
        }
    }
    else{

        // find time
        let kt=0;
        while( t>times[kt+1] ){
            kt++;
            console.assert(times.length>kt+1);
        }

        const p_temp = new THREE.Vector3();
        const q_temp = new THREE.Quaternion();

        for(let kj=0; kj<N_joint; ++kj)
        {
            const p1 = anim["position"][kt][kj];
            const r1 = anim["rotation"][kt][kj];

            const p2 = anim["position"][kt+1][kj];
            const r2 = anim["rotation"][kt+1][kj];

            const alpha = (t-times[kt])/(times[kt+1]-times[kt]);
            p_temp.copy(p1);
            p_temp.multiplyScalar(1-alpha);
            skeleton_destination["position_local"][kj].copy(p_temp);
            p_temp.copy(p2);
            p_temp.multiplyScalar(alpha);
            skeleton_destination["position_local"][kj].add(p_temp);


            q_temp.copy(r1);
            q_temp.slerp(r2, alpha);
            skeleton_destination["rotation_local"][kj].copy(q_temp);


        }



    }

    

    const sk_global = local_to_global(skeleton_destination["position_local"], skeleton_destination["rotation_local"], parent_id);

    skeleton_destination["position_global"] = sk_global["position"];
    skeleton_destination["rotation_global"] = sk_global["rotation"];

}

function local_to_global(position_local, rotation_local, parent_index)
{
    const position_global = []
    const rotation_global = []
    const N = position_local.length;
    position_global[0] = position_local[0]
    rotation_global[0] = rotation_local[0].clone()

    for(let k=1; k<N; k++)
    {
        const parent = parent_index[k];       
        
        rotation_global[k] = rotation_global[parent].clone().multiply(rotation_local[k]) . clone();
        position_global[k] = position_local[k].clone().applyQuaternion(rotation_global[parent]).clone().add(position_global[parent]) . clone();
    }

    return {"position":position_global,"rotation":rotation_global};
}