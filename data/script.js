
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

let scene, camera, renderer;

var index_state = null;
var old_index_state = null;

var cube = null;

function parentWidth(elem) {
  return elem.parentElement.clientWidth;
}

function parentHeight(elem) {
  return elem.parentElement.clientHeight;
}

function init3D(){
  const canvas = document.querySelector("canvas.webgl");
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);

  camera = new THREE.PerspectiveCamera(75, parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube")), 0.1, 1000);

  renderer = new THREE.WebGLRenderer({ antialias: true , canvas: canvas});
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));
  renderer.toneMapping = THREE.ACESFilmicToneMapping; //added contrast for filmic look
  renderer.toneMappingExposure = 1;
  renderer.outputEncoding = THREE.sRGBEncoding; //extended color space for the hdr

  document.getElementById('3Dcube').appendChild(renderer.domElement);

  // Orbit Controls: zoom in/out with scroll, pan with right-click, and drag to orbit
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.addEventListener("change", render); // use if there is no animation loop to render after any changes
  controls.minDistance = 2;
  controls.maxDistance = 10;
  controls.target.set(0, 0, -0.2);
  controls.update();


  // Create a geometry
  const geometry = new THREE.BoxGeometry(5, 1, 4);

  // Materials of each face
  var cubeMaterials = [
    new THREE.MeshBasicMaterial({color:0x03045e}),
    new THREE.MeshBasicMaterial({color:0x023e8a}),
    new THREE.MeshBasicMaterial({color:0x0077b6}),
    new THREE.MeshBasicMaterial({color:0x03045e}),
    new THREE.MeshBasicMaterial({color:0x023e8a}),
    new THREE.MeshBasicMaterial({color:0x0077b6}),
  ];

  const material = new THREE.MeshFaceMaterial(cubeMaterials);

  cube = new THREE.Mesh( geometry, material );

  scene.add(cube);
  camera.position.set(0, 0, 5);
  // camera.position.z = 15;
}

function render(){
  renderer.render(scene, camera);
}

// Resize the 3D object when the browser window changes size
function onWindowResize(){
  camera.aspect = parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube"));
  //camera.aspect = window.innerWidth /  window.innerHeight;
  camera.updateProjectionMatrix();
  //renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));

}

window.addEventListener('resize', onWindowResize, false);

// Create the 3D representation
init3D();

// Create events for the sensor readings
if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);

  source.addEventListener('gyro_readings', function(e) {
    //console.log("gyro_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("gyroX").innerHTML = obj.gyroX;
    document.getElementById("gyroY").innerHTML = obj.gyroY;
    document.getElementById("gyroZ").innerHTML = obj.gyroZ;

    // Change cube rotation after receiving the readings
    if (cube != null){
      cube.rotation.x = obj.gyroY;
      cube.rotation.z = obj.gyroX;
      cube.rotation.y = obj.gyroZ;
      render();
    }
  }, false);

  source.addEventListener('temperature_reading', function(e) {
    console.log("temperature_reading", e.data);
    document.getElementById("temp").innerHTML = e.data;
  }, false);

  source.addEventListener('accelerometer_readings', function(e) {
    console.log("accelerometer_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("accX").innerHTML = obj.accX;
    document.getElementById("accY").innerHTML = obj.accY;
    document.getElementById("accZ").innerHTML = obj.accZ;
  }, false);

  source.addEventListener('index_reading', function(e) {
    console.log("index_reading", e.data);
    document.getElementById("index_state").innerHTML = e.data;
    // Changing the index state
    index_state = e.data;

    // Depending on index state, load a different GLTF model

  const loader = new GLTFLoader();

  if (index_state == "1" && (index_state != old_index_state)){

    loader.load( 'https://cdn.jsdelivr.net/gh/lucahhot/EE327_Webserver@master/3D_Models/robotic_hand/scene.gltf', function ( gltf ) {

      // Removing the old object
      scene.remove(cube);

      scene.add( gltf.scene );
      cube = gltf.scene;
      render();

    }, undefined, function ( error ) {

      console.error( "Could not load 3D model!" );

    } );
  }

  if (index_state == "0" && (index_state != old_index_state)){

    loader.load( 'https://cdn.jsdelivr.net/gh/lucahhot/EE327_Webserver@master/3D_Models/yellow_robot_hand/scene.gltf', function ( gltf ) {

    // Removing the old object
    scene.remove(cube);

    scene.add( gltf.scene );
    cube = gltf.scene;
    render();

    }, undefined, function ( error ) {

      console.error( "Could not load 3D model!" );

    } );
  }


  // Update old_index_state to index_state
  old_index_state = index_state;

  }, false);

  source.addEventListener('middle_reading', function(e) {
    console.log("middle_reading", e.data);
    document.getElementById("middle_state").innerHTML = e.data;
  }, false);

  
}

