
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

let scene, camera, renderer;

var hand = null;

var finger_states = null;
var old_finger_states = null;

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

  const ambientLight = new THREE.AmbientLight( 0x404040 );
  scene.add( ambientLight );

  const dirLight = new THREE.DirectionalLight( 0xefefff, 1.5 );
  dirLight.position.set( 10, 10, 10 );
  scene.add( dirLight );

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

  hand = new THREE.Mesh( geometry, material );

  scene.add(hand);
  camera.position.set(0, 0, 15);
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

// String variable to keep track of finger states
var thumb_state = 1;
var index_state = 1;
var middle_state = 1;
var ring_state = 1;
var pinky_state = 1;

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
    // document.getElementById("gyroX").innerHTML = obj.gyroX;
    // document.getElementById("gyroY").innerHTML = obj.gyroY;
    // document.getElementById("gyroZ").innerHTML = obj.gyroZ;

    // Change hand rotation after receiving the readings
    if (hand != null){
      hand.rotation.x = obj.gyroY;
      hand.rotation.z = obj.gyroX;
      hand.rotation.y = obj.gyroZ;
      render();
    }
  }, false);

  // source.addEventListener('temperature_reading', function(e) {
  //   console.log("temperature_reading", e.data);
  //   document.getElementById("temp").innerHTML = e.data;
  // }, false);

  // source.addEventListener('accelerometer_readings', function(e) {
  //   console.log("accelerometer_readings", e.data);
  //   var obj = JSON.parse(e.data);
  //   document.getElementById("accX").innerHTML = obj.accX;
  //   document.getElementById("accY").innerHTML = obj.accY;
  //   document.getElementById("accZ").innerHTML = obj.accZ;
  // }, false);

  source.addEventListener('thumb_reading', function(e) {
    // console.log("thumb_reading", e.data);
    document.getElementById("thumb_state").innerHTML = e.data;
    thumb_state = e.data;
    loadModel();
  }, false);

  source.addEventListener('index_reading', function(e) {
    // console.log("index_reading", e.data);
    document.getElementById("index_state").innerHTML = e.data;
    index_state = e.data;
    loadModel();
  }, false);

  source.addEventListener('middle_reading', function(e) {
    // console.log("middle_reading", e.data);
    document.getElementById("middle_state").innerHTML = e.data;
    middle_state = e.data;
    loadModel();
  }, false);

  source.addEventListener('ring_reading', function(e) {
    // console.log("ring_reading", e.data);
    document.getElementById("ring_state").innerHTML = e.data;
    ring_state = e.data;
    loadModel();
  }, false);

  source.addEventListener('pinky_reading', function(e) {
    // console.log("pinky_reading", e.data);
    document.getElementById("pinky_state").innerHTML = e.data;
    pinky_state = e.data;
    loadModel();
  }, false);
  
}

function loadModel(){

  // Loading the correct 3D hand model 

  // Concantenating the finger readings into a single string

  finger_states = "hand_model_" + thumb_state + index_state + middle_state + ring_state + pinky_state;

  console.log("finger_states",finger_states);

  // Loading the GLTF models from github repository
  
  const loader = new GLTFLoader();

  // If finger_states is the same as old_finger_states, don't load a new model
  if(finger_states != old_finger_states){

    console.log("Loading 3D Hand Model...")

    let load_address = "https://cdn.jsdelivr.net/gh/lucahhot/EE327_Webserver@master/3D_Models/".concat(finger_states).concat("/").concat(finger_states).concat(".gltf");
    
    loader.load(load_address, function( gltf ){

      // Removing the old object
      scene.remove(hand);

      // Scaling and adding new loaded object
      gltf.scene.scale.set(0.1, 0.1, 0.1);
      scene.add(gltf.scene);
      hand = gltf.scene;
      render();

    }, undefined, function ( error )  {

      // If the model is not found/doesn't exist, keep the existing model loaded
      console.log("Could not find 3D model on github repo, keeping currently loaded model.")

    } );

  }

  old_finger_states = finger_states;
  
}





