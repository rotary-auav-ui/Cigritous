import React, { useState } from "react";
import { useFrame, useThree } from "react-three-fiber";

function Cube() {
  const [attitude, setAttitude] = useState({
    yaw: 0.0,
    pitch: 0.0,
    roll: 0.0,
  });
  const [position, setPosition] = useState([0, 0.5, 0]);
  const [rotation, setRotation] = useState([attitude.yaw, attitude.pitch, attitude.roll]);
  const { clock } = useThree();

  useFrame((state, delta) => {
    setAttitude((prevAttitude) => ({
      yaw: prevAttitude.yaw + 0.01,
      pitch: prevAttitude.pitch + 0.01,
      roll: prevAttitude.roll + 0.01,
    }));
    setRotation([attitude.yaw, attitude.pitch, attitude.roll]);
    setPosition([0, Math.sin(clock.getElapsedTime()) * 0.5, 0]);
  });

  return (
    <mesh position={position} rotation={rotation}>
      <boxGeometry />
      <meshStandardMaterial color="#BA365D" />
    </mesh>
  );
}

export default Cube;
