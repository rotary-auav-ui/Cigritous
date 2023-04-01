import React, { useEffect, useState } from "react";
import ParagraphView from "./components/ParagraphView";
import { Stack, Typography, Grid, Card, CardHeader, CardContent, IconButton, Box, Input } from "@mui/material";
import logo from "../logo_0.png";
import moment from "moment/moment";
import { Canvas, useFrame, useThree } from "react-three-fiber";
import { Physics, usePlane, useBox } from "@react-three/cannon";
import Button from "@mui/material/Button";
import "bootstrap/dist/css/bootstrap.min.css";
import image1 from "../DRONE2.jpeg";
import image2 from "../DRONE4.jpg";
import image3 from "../DRONE5.jpg";
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
          <Button
            onMouseEnter={handleDashboardHover}
            onMouseLeave={handleDashboardHover}
            style={{
              color: hoverDashboard ? "#6841b0" : "white",
              fontSize: 20,
            }}
            href="/"
          >
            Dashboard
          </Button>
          <Button
            onMouseEnter={handleAboutHover}
            onMouseLeave={handleAboutHover}
            style={{
              color: hoverAbout ? "#6841b0" : "white",
              fontSize: 20,
            }}
            href="/About"
          >
            About
          </Button>
          <Button
            onMouseEnter={handleControlsHover}
            onMouseLeave={handleControlsHover}
            style={{
              color: hoverControls ? "#6841b0" : "white",
              fontSize: 20,
            }}
            href="/Controls"
          >
            Controls
          </Button>
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
          About Cigritous
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
            text="Our goal is to create a system called 'Crop Monitoring with Automated UAV Spray Response,' which utilizes technology to safeguard crop quality against weather factors and pests. This system has the potential to enhance the economy of developing countries by assisting farmers in minimizing costs. Precise farming techniques can be facilitated with GPS technologies, which enable data collection, field mapping, farm planning, and yield management. Moreover, GPS technologies can be used to provide automated directions to UAVs.
            This Project comprehensive monitoring and spraying system that involves monitoring various parameters in each crop, such as moisture, humidity, and temperature. These parameters will be transmitted to a central module, which includes a Bosch Sensortec BME688 gas sensor, global position data from each sensor, and an internet connection for remote or cloud-based monitoring.
            
            If any crops are identified as underwatered or at risk due to gas presence, ozone, temperature, moisture, or humidity levels, the central module will trigger the HoverGames Drone UAV to take off and spray the designated location with water or pesticide. The UAV is equipped with an i.MX 8M companion computer for pest tracking and precision landing, as well as a Telemetry Radio for relaying the UAV's condition to the central module and the user. Once the spraying task is completed, the UAV will autonomously return to its base."
          />
        </div>
      </Box>
    </Stack>
  );
};

export default About;
