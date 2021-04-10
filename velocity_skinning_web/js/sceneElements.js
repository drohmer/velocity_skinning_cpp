"use strict";



// General variable storing the scene graph, and elements usefull to render the scene (camera, renderer, etc)
const sceneElements = {
    sceneGraph : null,
    camera     : null,  
    control    : null,
    renderer   : null,
    models     : [],
    props      : {},
    timer : {'interval':[0,12],current:0},
    param : {'Activated':true, 'DeformOnly':false, 'Flappy':1.0, 'Squashy':1.0, 'Wireframe':false, 'materialWireframe':null, displaySkeleton:false, ssao:false, weights:false, texture:true, background:true, textureWhite:null,}
};



let composer, renderPass, saoPass;

let gui = new dat.GUI({
    
});



// let stats = new Stats();
// stats.showPanel( 1 ); // 0: fps, 1: ms, 2: mb, 3+: custom
// document.body.appendChild( stats.dom );
