"use strict";


function createBasicElements(sceneGraph)
{

    // var imagePrefix = "assets/textures/cartoon/";
	// var directions  = ["px", "nx", "py", "ny", "pz", "nz"];
	// var imageSuffix = ".png";
	// var skyGeometry = new THREE.CubeGeometry( 50, 50, 50 );	
	
	// var materialArray = [];
	// for (var i = 0; i < 6; i++)
	// 	materialArray.push( new THREE.MeshBasicMaterial({
	// 		map: THREE.ImageUtils.loadTexture( imagePrefix + directions[i] + imageSuffix ),
	// 		side: THREE.BackSide
	// 	}));
	// var skyMaterial = new THREE.MeshFaceMaterial( materialArray );
	// var skyBox = new THREE.Mesh( skyGeometry, skyMaterial );
	// sceneGraph.add( skyBox );

    // var loader = new THREE.CubeTextureLoader();
    // loader.setPath( 'assets/textures/pisa/' );

    // var textureCube = loader.load( [
    //     'px.png', 'nx.png',
    //     'py.png', 'ny.png',
    //     'pz.png', 'nz.png'
    // ] );

    // var cubeMapMaterial = new THREE.MeshBasicMaterial( { color: 0xffffff, map: textureCube, side:THREE.DoubleSide } );

    // const cubeMapGeometry = new THREE.BoxGeometry( 20, 20, 20 );
    // const cubeMap = new THREE.Mesh( cubeMapGeometry, cubeMapMaterial );  
    // sceneGraph.add(cubeMap);





    // ************************** //
    // Create a ground plane
    // ************************** //
    const planeGeometry = new THREE.PlaneGeometry( 20, 20 );
    const planeMaterial = new THREE.MeshPhongMaterial( {color:'rgb(150, 210, 150)',  side:THREE.FrontSide} );  
    const plane = new THREE.Mesh( planeGeometry, planeMaterial );
    sceneGraph.add( plane );

    // Change orientation of the plane using rotation
    plane.rotateOnAxis(new THREE.Vector3(1,0,0), -Math.PI/2 );
    //plane.translateZ(-1.0);
    // Set shadow property
    plane.receiveShadow = true;


    sceneElements.props["plane"] = plane;

   

    // // ************************** //
    // // Sphere
    // // ************************** //    
    // const sphereGeometry = new THREE.SphereGeometry(0.2, 32, 32);
    // const sphereMaterial = new THREE.MeshPhongMaterial( {color:'rgb(180,180,255)'} );
    // const sphere = new THREE.Mesh( sphereGeometry, sphereMaterial );
    // sceneGraph.add(sphere);

    // // Set position of the sphere
    // sphere.translateX(1.0).translateY(1.0).translateZ(1.0);
    // // Set shadow property
    // sphere.castShadow = true;



    // // ************************** //
    // // Create cylinder
    // // ************************** //
    // const cylinderGeometry = new THREE.CylinderGeometry(0.1, 0.1, 2.0, 25, 1);
    // const cylinderMaterial = new THREE.MeshPhongMaterial( {color:'rgb(150,255,150)'} );
    // const cylinder = new THREE.Mesh( cylinderGeometry, cylinderMaterial );
    // cylinder.name = "cylinder";
    // sceneGraph.add(cylinder);

    // // Set position of the cylinder
    // cylinder.translateX(1.0).translateY(2.0).translateZ(1.0);
    // // Set shadow property
    // cylinder.castShadow = true; 


    // // ************************** //
    // // Create cylinder 2
    // // ************************** //
    // const cylinder2Geometry = new THREE.CylinderGeometry(0.1, 0.1, 2.0, 25, 1);
    // const cylinder2Material = new THREE.MeshPhongMaterial( {color:'rgb(255,100,100)'} );
    // const cylinder2 = new THREE.Mesh( cylinder2Geometry, cylinder2Material );
    // cylinder2.rotateZ(Math.PI/2);
    // cylinder.add(cylinder2);

    // cylinder2.name = "cylinder2";
    // cylinder2.castShadow = true; 
}


// Create some visual elements (text, etc) to help the visualization of the 3D scene
function loadVisualElementHelper(sceneGraph)
{
    // // ************************** //
    // // Load text
    // // ************************** //
    // const loader = new THREE.FontLoader();
    // loader.load( 'fonts/helvetiker_regular.typeface.json', function ( font ) {
    //     const textGeometry = new THREE.TextGeometry('(1,1,1)', {font: font, size: 0.2, height: 0.01, curveSegments: 12,} );
    //     const textMaterial = new THREE.MeshPhongMaterial( {color:'rgb(180,180,255)'} );
    //     const textObject = new THREE.Mesh(textGeometry, textMaterial);
    //     sceneGraph.add( textObject );
    //     textObject.translateX(1.25).translateY(1).translateZ(1.15);
    // } );


    // ************************** //
    // Create (x,y,z) frame centered in (0,0,0)
    // ************************** //
    const xFrameGeometry = new THREE.Geometry(); xFrameGeometry.vertices.push(new THREE.Vector3(0,0,0), new THREE.Vector3(1,0,0));
    const yFrameGeometry = new THREE.Geometry(); yFrameGeometry.vertices.push(new THREE.Vector3(0,0,0), new THREE.Vector3(0,1,0));
    const zFrameGeometry = new THREE.Geometry(); zFrameGeometry.vertices.push(new THREE.Vector3(0,0,0), new THREE.Vector3(0,0,1));
    const xFrame = new THREE.Line(xFrameGeometry, new THREE.LineBasicMaterial({color:'rgb(255,0,0)', linewidth:2}));
    const yFrame = new THREE.Line(yFrameGeometry, new THREE.LineBasicMaterial({color:'rgb(0,255,0)', linewidth:2}));
    const zFrame = new THREE.Line(zFrameGeometry, new THREE.LineBasicMaterial({color:'rgb(0,0,255)', linewidth:2}));
    sceneGraph.add(xFrame);
    sceneGraph.add(yFrame);
    sceneGraph.add(zFrame);
    xFrame.castShadow = false;
    yFrame.castShadow = false;
    zFrame.castShadow = false;


    // // ************************** //
    // // Create x axis passing through (1,1,1)
    // // ************************** //
    // const dashedLineMaterial = new THREE.LineDashedMaterial( {color: 0xff0000, linewidth: 2, scale: 1, dashSize: 0.1, gapSize: 0.1} );
    // const dashedLineGeometry = new THREE.Geometry();
    // dashedLineGeometry.vertices.push(new THREE.Vector3( 1, 1, -2) );
    // dashedLineGeometry.vertices.push(new THREE.Vector3( 1, 1,  4) );
    // const dashedLine = new THREE.LineSegments( dashedLineGeometry, dashedLineMaterial );
    // dashedLine.computeLineDistances();
    // sceneGraph.add(dashedLine);

}
