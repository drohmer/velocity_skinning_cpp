"use strict";

// Create an empty scene with a camera, spotlight, and a renderer
function initEmptyScene(sceneElements) {

    // ************************** //
    // Create the 3D scene
    // ************************** //
    sceneElements.sceneGraph = new THREE.Scene();

    // ************************** //
    // Add camera
    // ************************** //
    const width = window.innerWidth;
    const height = window.innerHeight;
    const camera = new THREE.PerspectiveCamera(45, width/height, 0.1, 500);
    sceneElements.camera = camera;
    camera.position.set(-1, 3, 10);
    camera.lookAt(0, 0, 0);

    

    // ************************** //
    // Add an ambient light
    // ************************** //
    const ambientLight = new THREE.AmbientLight( 'rgb(255, 255, 255)', 0.2 );
    sceneElements.sceneGraph.add(ambientLight);
    
    // ************************** //
    // Add spotlight (with shadow)
    // ************************** //
    const spotLight = new THREE.SpotLight('rgb(255, 255, 255)', 0.5);
    spotLight.position.set(6, 15, 9);

    const spotLight2 = new THREE.SpotLight('rgb(255, 255, 255)', 0.2);
    spotLight2.position.set(+9, 15, -9);

    const ambiantLight = new THREE.AmbientLight('rgb(255, 255, 255)', 0.3);

    //const shadowLight = new THREE.LightShadow('rgb(255, 255, 255)', 0.6);
    //shadowLight.position.set(-9, 15, 9);
    
    sceneElements.sceneGraph.add(spotLight);
    sceneElements.sceneGraph.add(spotLight2);
    sceneElements.sceneGraph.add(ambiantLight);
    //sceneElements.sceneGraph.add(shadowLight);

    // Setup shadow properties for the spotlight
    spotLight.castShadow = true;
    spotLight.shadow.mapSize.width = 2048;
    spotLight.shadow.mapSize.height = 2048;
    spotLight.shadow.camera.near = 5;
    spotLight.shadow.camera.far = 30;


    

    // ************************** //
    // Create renderer (with shadow map)
    // ************************** //
    const renderer = new THREE.WebGLRenderer( {antialias:true} );
    sceneElements.renderer = renderer;
    renderer.setPixelRatio( window.devicePixelRatio );
    renderer.setClearColor('rgb(200, 200, 255)', 1.0);
    renderer.setSize( width, height );
    //renderer.fog(new THREE.Fog('rgb(200,200,200)', 0.1, 20.0));
    
    // Setup shadowMap property
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    

    composer = new THREE.EffectComposer( renderer );
    renderPass = new THREE.RenderPass( sceneElements.sceneGraph, sceneElements.camera );
    composer.addPass( renderPass );
    saoPass = new THREE.SAOPass( sceneElements.sceneGraph, sceneElements.camera, false, true );
    composer.addPass( saoPass );

    //saoPass.params.output = THREE.SAOPass.OUTPUT.SAO;

    saoPass.params.saoBias = 50;
    saoPass.params.saoIntensity = 0.005;//0.01;
    saoPass.params.saoScale = 1;
    saoPass.params.saoKernelRadius = 10;

    saoPass.params.saoBlur = true;
    saoPass.params.saoBlurRadius = 3;
    saoPass.params.saoBlurStdDev = 2;
    saoPass.params.saoBlurDepthCutoff = 0.02;

    // saoPass.params.saoBias = 1.0;
    // saoPass.params.saoIntensity = 0.01;//0.01;
    // saoPass.params.saoScale = 8;
    // saoPass.params.saoKernelRadius = 10;

    // saoPass.params.saoBlur = true;
    // saoPass.params.saoBlurRadius = 5;
    // saoPass.params.saoBlurStdDev = 5;
    // saoPass.params.saoBlurDepthCutoff = 0.02;
    
    
    // ************************** //
    // Add the render image in the HTML DOM
    // ************************** //
    const htmlElement = document.querySelector("#Tag3DScene");
    htmlElement.appendChild(renderer.domElement);


    // ************************** //
    // Control for the camera
    // ************************** //
    sceneElements.control = new THREE.OrbitControls( camera, sceneElements.renderer.domElement );
    sceneElements.control.screenSpacePanning = true;
    sceneElements.control.enableKeys = false;

    
}