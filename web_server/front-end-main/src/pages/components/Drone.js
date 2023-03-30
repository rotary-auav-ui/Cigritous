import React, { useRef, useEffect } from "react";
import * as THREE from "three";

const Drone = () => {
  const containerRef = useRef();
  useEffect(() => {
    // Initialize scene, camera and renderer
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(window.innerWidth, window.innerHeight);
    containerRef.current.appendChild(renderer.domElement);

    // Create a drone
    const droneGeometry = new THREE.BoxGeometry(1, 1, 1);
    const droneMaterial = new THREE.MeshBasicMaterial({ color: "#6841b0" });
    const drone = new THREE.Mesh(droneGeometry, droneMaterial);
    scene.add(drone);

    // Position the camera and render the scene
    camera.position.z = 5;
    const render = () => {
      requestAnimationFrame(render);
      drone.rotation.x += 0.01;
      drone.rotation.y += 0.01;
      renderer.render(scene, camera);
    };
    render();
  }, []);

  return (
    <div style={{ position: "relative" }}>
      <div ref={containerRef} />
      <div style={{ position: "absolute", bottom: "30%", left: "0", width: "100%", textAlign: "center" }}>Mohon Maaf, Halaman Sedang Maintenance Yach...</div>
    </div>
  );
};

export default Drone;
