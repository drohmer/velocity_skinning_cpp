"use strict";

function computeSkinningInverse(vertex_deformed, rig, vertex_rest_pose, skeleton_rest_pose, skeleton_current)
{
    let p_temp = new THREE.Vector3();
    let q_temp = new THREE.Quaternion();
    let qj0_inv = new THREE.Quaternion();
    let M = new THREE.Matrix4();
    let Ms = new THREE.Matrix4();
    let Ms_inv = new THREE.Matrix4();
    let tr = new THREE.Vector3();
    let T = new THREE.Matrix4();

    if(vertex_deformed && rig && vertex_rest_pose && skeleton_rest_pose && skeleton_current)
    {
        const N_vertex = vertex_rest_pose.length;
        
        for(let k_vertex=0; k_vertex<N_vertex; k_vertex++)
        {

            vertex_deformed[k_vertex].set(0,0,0);
            const N_dependency = rig["joint"][k_vertex].length;
           
            T.set(0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,1);
            Ms.set(0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0);
            
            for(let k_dep=0; k_dep<N_dependency; k_dep++)
            {

                const idx = rig["joint"][k_vertex][k_dep];
                const w = rig["weight"][k_vertex][k_dep];

                const qj = skeleton_current["rotation_global"][idx];
                const pj = skeleton_current["position_global"][idx];

                const qj0 = skeleton_rest_pose["rotation_global"][idx];
                const pj0 = skeleton_rest_pose["position_global"][idx];

                qj0_inv.copy(qj0);
                qj0_inv.conjugate();

                

                
                q_temp.copy(qj);
                q_temp.multiply(qj0_inv);
                M.makeRotationFromQuaternion(q_temp);

    

                tr.copy(pj0);
                tr.applyMatrix4(M);
                tr.multiplyScalar(-1.0);
                tr.add(pj);


                M.setPosition(tr);

                for(let k=0; k<16; ++k)
                {
                    Ms.elements[k] += w * M.elements[k];
                }

            }

            p_temp.copy(vertex_rest_pose[k_vertex]);
            Ms_inv.getInverse(Ms);
            p_temp.applyMatrix4(Ms_inv);
            vertex_deformed[k_vertex].copy(p_temp);

            //model["mesh"].geometry.vertices[k_vertex].add(new THREE.Vector3(sceneElements.timer.current,0,0));
            // if(K0<10 && k_vertex==5){
            // console.log(new THREE.Vector3(sceneElements.timer.current,0,0));
            // }
            //model["mesh"].geometry.vertices[k_vertex].copy(sceneElements.models[name]["vertex_rest_pose"]);
            //model["mesh"].geometry.vertices[k_vertex].applyMatrix4(T);

        }
        //mesh.geometry.verticesNeedUpdate = true;
    }
}

function computeSkinning(vertex_deformed, rig, vertex_rest_pose, skeleton_rest_pose, skeleton_current)
{

    let p_temp = new THREE.Vector3();
    let q_temp = new THREE.Quaternion();
    let qj0_inv = new THREE.Quaternion();
    let M = new THREE.Matrix4();
    let Ms = new THREE.Matrix4();
    let tr = new THREE.Vector3();
    let T = new THREE.Matrix4();

    if(vertex_deformed && rig && vertex_rest_pose && skeleton_rest_pose && skeleton_current)
    {
        const N_vertex = vertex_rest_pose.length;
        
        for(let k_vertex=0; k_vertex<N_vertex; k_vertex++)
        {

            vertex_deformed[k_vertex].set(0,0,0);
            const N_dependency = rig["joint"][k_vertex].length;
           
            T.set(0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,1);
            Ms.set(0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0);
            
            for(let k_dep=0; k_dep<N_dependency; k_dep++)
            {

                const idx = rig["joint"][k_vertex][k_dep];
                const w = rig["weight"][k_vertex][k_dep];

                const qj = skeleton_current["rotation_global"][idx];
                const pj = skeleton_current["position_global"][idx];

                const qj0 = skeleton_rest_pose["rotation_global"][idx];
                const pj0 = skeleton_rest_pose["position_global"][idx];

                qj0_inv.copy(qj0);
                qj0_inv.conjugate();

                

                
                q_temp.copy(qj);
                q_temp.multiply(qj0_inv);
                M.makeRotationFromQuaternion(q_temp);

    

                tr.copy(pj0);
                tr.applyMatrix4(M);
                tr.multiplyScalar(-1.0);
                tr.add(pj);


                M.setPosition(tr);

                for(let k=0; k<16; ++k)
                {
                    Ms.elements[k] += w * M.elements[k];
                }

            }

            p_temp.copy(vertex_rest_pose[k_vertex]);
            p_temp.applyMatrix4(Ms);
            vertex_deformed[k_vertex].copy(p_temp);

            //model["mesh"].geometry.vertices[k_vertex].add(new THREE.Vector3(sceneElements.timer.current,0,0));
            // if(K0<10 && k_vertex==5){
            // console.log(new THREE.Vector3(sceneElements.timer.current,0,0));
            // }
            //model["mesh"].geometry.vertices[k_vertex].copy(sceneElements.models[name]["vertex_rest_pose"]);
            //model["mesh"].geometry.vertices[k_vertex].applyMatrix4(T);

        }
        //mesh.geometry.verticesNeedUpdate = true;
    }

    

    


}
