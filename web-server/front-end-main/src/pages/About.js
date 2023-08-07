import React, { useEffect, useState } from "react";
import ParagraphView from "./components/ParagraphView";
import { Stack, Typography, Grid, Card, CardHeader, CardContent, IconButton, Box, Input } from "@mui/material";
import logo from "../public/logo_1.png";
import moment from "moment/moment";
import { Canvas, useFrame, useThree } from "react-three-fiber";
import { Physics, usePlane, useBox } from "@react-three/cannon";
import CustomButton from "./components/CustomButton";
import "bootstrap/dist/css/bootstrap.min.css";
import image1 from "../public/DRONE2.jpeg";
import image2 from "../public/DRONE4.jpg";
import image3 from "../public/DRONE5.jpg";
import Cube from "./components/Cube";

const About = () => {
  moment.locale("id");
  const [hoursTime, setHoursTime] = useState("");
  const [daysTime, setDaysTime] = useState("");

  const [hoverDashboard, setHoverDashboard] = useState(false);
  const [hoverAbout, setHoverAbout] = useState(false);
  const [hoverControls, setHoverControls] = useState(false);

  const handleDashboardHover = () => setHoverDashboard(!hoverDashboard);
  const handleAboutHover = () => setHoverAbout(!hoverAbout);
  const handleControlsHover = () => setHoverControls(!hoverControls);

  useEffect(() => {
    const interval = setInterval(() => {
      setHoursTime(moment().format("H:mm:ss"));
      setDaysTime(moment().format("ddd, DD MMMM YYYY"));
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  return (
    <Stack direction={"row"} gap={"30px"}>
      <Stack flexBasis={"25%"} width={"80%"} maxWidth={"25%"} alignItems="center" gap="10px" sx={{ background: "#000000", height: "100vh", padding: "30px" }}>
        <img src={logo} alt="Logo" width="120px" />

        <Typography>{hoursTime}</Typography>
        <Typography>{daysTime}</Typography>
        <Stack direction={"column"} padding="20px" gap="20px"></Stack>
        <Stack direction="column" spacing={1}>
          <CustomButton href="/" label="Dashboard" hover={hoverDashboard} handleHover={handleDashboardHover} />
          <CustomButton href="/About" label="About" hover={hoverAbout} handleHover={handleAboutHover} />
          <CustomButton href="/Controls" label="Controls" hover={hoverControls} handleHover={handleControlsHover} />
        </Stack>
        <Stack direction={"column"} padding="20px" gap="0px"></Stack>

        <Canvas dpr={[1, 2]} shadows camera={{ position: [-5, 5, 5], fov: 18 }}>
          <ambientLight />
          <spotLight angle={0.25} penumbra={0.5} position={[10, 10, 3]} castShadow />
          <Physics allowSleep={true}>
            <Cube />
          </Physics>
        </Canvas>
      </Stack>

      <Box flexBasis={"100%"} width={"100%"} sx={{ overflowY: "scroll", maxHeight: "100vh" }}>
        <Typography
          sx={{
            color: "#BA365D",
            fontSize: "30px",
            margin: "20px auto",
            fontWeight: "bold",
          }}
          textAlign="center"
        >
          About VishGround
        </Typography>
        <div style={{ display: "flex", justifyContent: "center", alignItems: "center", height: "50vh" }}>
          <Stack direction={"row"} gap={"20px"}>
            <img src={image1} alt="Image 1" width="250px" style={{ borderRadius: "50%", border: "5px solid #3D3356" }} />
            <img src={image2} alt="Image 2" width="250px" style={{ borderRadius: "50%", border: "5px solid #3D3356" }} />
            <img src={image3} alt="Image 3" width="250px" style={{ borderRadius: "50%", border: "5px solid #3D3356" }} />
          </Stack>
        </div>
        <div>
          <ParagraphView
            text="VishGround is a system used to control and monitor unmanned aerial vehicles 
            (UAVs) or other aircraft remotely from the ground. It serves as a command center that enables 
            human operators to control aircraft operations in real-time. Vishwakharma Team is a team from the University of Indonesia specializing in VTOL (Vertical Take-Off and Landing) technology. They are dedicated to advancing and innovating in the field of aerial vehicles. The team consists of passionate and motivated individuals who strive to push the boundaries of aviation engineering.They actively participate in various competitions and projects, showcasing their expertise and technical skills.Their commitment to excellence and continuous learning makes them a valuable asset in the field of aviation technology."
          />
        </div>
      </Box>
    </Stack>
  );
};

export default About;
