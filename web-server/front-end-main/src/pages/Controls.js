import React, { useEffect, useState } from "react";
import { Stack, Typography, Grid, Card, Box } from "@mui/material";
import logo from "../logo_0.png";
import moment from "moment/moment";
import axios from "axios";
import { Canvas } from "react-three-fiber";
import { Physics, usePlane } from "@react-three/cannon";
import GoogleMapReact from "google-map-react";
import { Chart as ChartJS, CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend } from "chart.js";
import Button from "@mui/material/Button";
import "bootstrap/dist/css/bootstrap.min.css";
import mqtt from "mqtt/dist/mqtt";
import CorCard from "./components/CoordinateCard";
import Cube from "./components/Cube";
import LocationPin from "./components/LocationPin";
import SensorCard from "./components/SensorCard";
import LocationDrone from "./components/LocationDrone";

var options = {
  port: 38789,
  host: "wss://driver.cloudmqtt.com",
  clientId: "mqttjs_" + Math.random().toString(16).substr(2, 8),
  username: "cbobzrgp",
  password: "CKvOQLxrtuqc",
};

ChartJS.register(CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend);

const handleTakeOff = () => {
  let hasPublished = false;
  const client = mqtt.connect("wss://driver.cloudmqtt.com:1884", options);
  client.on(
    "connect",
    () => {
      console.log("MQTT client connected to the server.");
      if (!hasPublished) {
        client.publish("/drone/take_land", String(1), { qos: 0 });
        hasPublished = true;
      }
      return () => client.end();
    },
    []
  );
};

const handleLanding = () => {
  let hasPublished = false;
  const client = mqtt.connect("wss://driver.cloudmqtt.com:1884", options);
  client.on(
    "connect",
    () => {
      console.log("MQTT client connected to the server.");
      if (!hasPublished) {
        client.publish("/drone/take_land", String(0), { qos: 0 });
        hasPublished = true;
      }
      return () => client.end();
    },
    []
  );
};

const Controls = () => {
  moment.locale("id");
  const [hoursTime, setHoursTime] = useState("");
  const [daysTime, setDaysTime] = useState("");
  const [mapsFlight, setMapsFlight] = useState([]);
  const [droneFlightLtd, setDroneFlightLtd] = useState([]);
  const [droneFlightLng, setDroneFlightLng] = useState([]);
  const [mapsFlightLtd, setMapsFlightLtd] = useState([]);
  const [mapsFlightLng, setMapsFlightLng] = useState([]);
  const [droneStatus, setDroneStatus] = useState([]);
  const [droneBattery, setDroneBattery] = useState([]);
  const [droneAltitude, setDroneAltitude] = useState([]);
  const [droneSpeedX, setDroneSpeedX] = useState([]);
  const [droneSpeedY, setDroneSpeedY] = useState([]);
  const [droneSpeedZ, setDroneSpeedZ] = useState([]);
  const [droneProgress, setDroneProgress] = useState([]);
  const [droneHeading, setDroneHeading] = useState([]);
  const [droneTimestamp, setDroneTimeStamp] = useState([]);

  let arrCoor = [...mapsFlight];

  const [titik, setTitik] = useState(0);

  useEffect(() => {
    async function fetchData() {
      const response = await axios.get("https://vtol-cigritous-backend.herokuapp.com/updatesumnode");
      const dataTitik = response.data;
      if (dataTitik.length > 0 && dataTitik[0].angka > 0) {
        setTitik(dataTitik[0].angka);
      } else if (titik > 0) {
        const dataSum = {
          id: 1,
          angka: titik,
        };
        try {
          console.log("Data SUM: ", dataSum);
          await axios.post("https://vtol-cigritous-backend.herokuapp.com/insertsumnode", dataSum);
          console.log("Data Central sent to the backend");
        } catch (error) {
          console.error("Error sending data to the backend: ", error);
        }
      }
    }
    fetchData();
  }, [titik]);

  let totalNode = 20;

  const nodes = [];

  for (let index = 1; index <= totalNode; index++) {
    nodes.push(index);
  }

  const [dataCor, setDataCor] = useState({
    node: [],
    latitude: [],
    longitude: [],
    coordinate: [],
  });

  useEffect(() => {
    const fetchData = async (a) => {
      try {
        console.log("masuk fetch");
        const response = await axios.get(`https://vtol-cigritous-backend.herokuapp.com/updatecoordinate/${a}`);
        const data = response.data;

        if (data.length !== 0) {
          setDataCor({
            node: data.map((item) => item.node),
            latitude: data.map((item) => item.latitude),
            longitude: data.map((item) => item.longitude),
            coordinate: data.map((item) => item.coordinate),
          });

          setMapsFlight((prevArr) => [...prevArr, data.map((item) => item.coordinate)]);
          setMapsFlightLtd((prevArr) => [...prevArr, data.map((item) => item.latitude)]);
          setMapsFlightLng((prevArr) => [...prevArr, data.map((item) => item.longitude)]);

          console.log("inc : ", a);
        } else {
          console.log(`No data returned for ${a}`);
        }
      } catch (error) {
        console.error("Error fetching data: ", error);
      }
    };

    for (let a = 1; a <= titik; a++) {
      fetchData(a);
    }
  }, [titik]);

  const [hoverCard, setHoverCard] = useState(Array(nodes.length).fill(false));
  const [hoverDashboard, setHoverDashboard] = useState(false);
  const [hoverAbout, setHoverAbout] = useState(false);
  const [hoverControls, setHoverControls] = useState(false);

  const handleDashboardHover = () => setHoverDashboard(!hoverDashboard);
  const handleAboutHover = () => setHoverAbout(!hoverAbout);
  const handleControlsHover = () => setHoverControls(!hoverControls);

  const [mapType, setMapType] = useState("roadmap");

  const handleViewChange = () => {
    setMapType(mapType === "roadmap" ? "satellite" : "roadmap");
  };

  const handleCardHover = (index) => {
    const newHoverCard = [...hoverCard];
    newHoverCard[index] = !newHoverCard[index];
    setHoverCard(newHoverCard);
  };

  const [attitude, setAttitude] = useState({
    yaw: 0.0,
    pitch: 0.0,
    roll: 0.0,
    att: 0.0,
    lat: -6.365232,
    lng: 106.824506,
  });

  const defaultProps = {
    center: {
      lat: attitude.lat,
      lng: attitude.lng,
    },
    fly: {
      lat: -6.3648,
      lng: 106.8245,
    },
    zoom: 18,
    options: {
      disableDefaultUI: true,
      dragging: false,
      scrollwheel: false,
      panControl: false,
      zoomControl: false,
      gestureHandling: "none",
    },
  };

  useEffect(() => {
    const interval = setInterval(() => {
      setHoursTime(moment().format("H:mm:ss"));
      setDaysTime(moment().format("ddd, DD MMMM YYYY"));
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  useEffect(() => {
    const client = mqtt.connect("wss://driver.cloudmqtt.com:1884", options);
    client.on("connect", () => {
      console.log("MQTT client connected to the server.");
      client.subscribe("/drone/status");
      client.subscribe("/drone/battery");
      client.subscribe("/drone/progress");
      client.subscribe("/drone/lat");
      client.subscribe("/drone/lng");
      client.subscribe("/drone/alt");
      client.subscribe("/drone/vx");
      client.subscribe("/drone/vy");
      client.subscribe("/drone/vz");
      client.subscribe("/drone/yaw_curr");
      client.subscribe("/drone/time");
      for (let i = 1; i <= 20; i++) {
        client.subscribe("/" + i + "/coordinate");
      }
    });

    console.log("masuk config");
    client.on("message", (topic, message) => {
      console.log("tessss");
      if (topic === "/drone/status") {
        if (message.toString() === "0") {
          setDroneStatus("Disarmed");
        } else if (message.toString() === "1") {
          setDroneStatus("Armed");
        }
      }
      if (topic === "/drone/battery") {
        setDroneBattery(message.toString());
      }
      if (topic === "/drone/progress") {
        setDroneProgress(message.toString());
      }
      if (topic === "/drone/lat") {
        setDroneFlightLtd((message / 10e6).toString());
      }
      if (topic === "/drone/lng") {
        setDroneFlightLng((message / 10e6).toString());
      }
      if (topic === "/drone/alt") {
        setDroneAltitude(message.toString());
      }
      if (topic === "/drone/vx") {
        setDroneSpeedX(message.toString());
      }
      if (topic === "/drone/vy") {
        setDroneSpeedY(message.toString());
      }
      if (topic === "/drone/vz") {
        setDroneSpeedZ(message.toString());
      }
      if (topic === "/drone/yaw_curr") {
        setDroneHeading(message.toString());
      }
      if (topic === "/drone/time") {
        setDroneTimeStamp(message.toString());
      }
      for (let i = 1; i <= 20; i++) {
        if (topic === "/" + i + "/coordinate") {
          arrCoor[i - 1] = JSON.parse(message);
          setMapsFlight(arrCoor);
        }
      }
    });
    return () => {
      client.end();
    };
  }, [mapsFlight, droneFlightLng, droneFlightLtd]);

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
          Controls Unnamed Drone
        </Typography>
        <Stack direction={"row"} gap={"10px"} padding="20px">
          <button
            onMouseEnter={() => handleCardHover(24)}
            onMouseLeave={() => handleCardHover(24)}
            style={{ backgroundColor: "#3D3356", color: "white", padding: "10px 30px", border: "none", boxShadow: hoverCard[24] ? "0px 0px 20px 0px #000000" : "none" }}
            onClick={handleViewChange}
          >
            Switch to {mapType === "roadmap" ? "Satellite" : "Roadmap"} view
          </button>
        </Stack>
        <Stack direction={"column"} padding="20px" gap="20px">
          <Stack style={{ height: "50vh", width: "100%" }}>
            <GoogleMapReact
              bootstrapURLKeys={{
                key: "AIzaSyD3RzE2fq7JvhFmDTbXyjj22jqIAytT7XU",
                language: "id",
              }}
              defaultCenter={defaultProps.center}
              defaultZoom={defaultProps.zoom}
              options={{ mapTypeId: mapType }}
            >
              <LocationDrone lat={droneFlightLtd} lng={droneFlightLng} text="Drone" color="white" startLat={droneFlightLtd} startLong={droneFlightLng} />
              {mapsFlightLtd?.map((lat, idx) => (
                <LocationPin lat={lat} lng={mapsFlightLng[idx]} text={`Node ke-${idx + 1}`} color="yellow" />
              ))}
            </GoogleMapReact>
          </Stack>
          <div style={{ display: "flex", justifyContent: "space-between" }}>
            <button
              onMouseEnter={() => handleCardHover(6)}
              onMouseLeave={() => handleCardHover(6)}
              style={{ backgroundColor: "#3D3356", color: "white", padding: "10px 30px", border: "none", boxShadow: hoverCard[6] ? "0px 0px 20px 0px #000000" : "none" }}
              onClick={handleTakeOff}
            >
              Take Off Drone
            </button>
            <Card
              onMouseEnter={() => handleCardHover(8)}
              onMouseLeave={() => handleCardHover(8)}
              style={{ backgroundColor: "#3D3356", color: "white", padding: "10px 30px", border: "none", boxShadow: hoverCard[8] ? "0px 0px 20px 0px #000000" : "none" }}
            >
              Task Progress : {droneProgress}%{" "}
            </Card>
            <button
              onMouseEnter={() => handleCardHover(7)}
              onMouseLeave={() => handleCardHover(7)}
              style={{ backgroundColor: "#3D3356", color: "white", padding: "10px 30px", border: "none", boxShadow: hoverCard[7] ? "0px 0px 20px 0px #000000" : "none" }}
              onClick={handleLanding}
            >
              Landing Drone
            </button>
          </div>
          <Stack direction={"column"} padding="10px" gap="10px"></Stack>
        </Stack>
        <Stack direction={"column"} padding="20px" gap="10px">
          <CorCard title="Coordinate Position Drone" value={"Lat : " + droneFlightLtd + " || Lng : " + droneFlightLng} handleCardHover={() => handleCardHover(3)} hoverCard={hoverCard[3]} />
          <Stack direction={"column"} padding="20px" gap="10px">
            <Grid container spacing={2} columns={3} width="100%" justifyContent={"center"}>
              <Grid item xs={1}>
                <SensorCard title="Status Drone" value={droneStatus} handleCardHover={() => handleCardHover(1)} hoverCard={hoverCard[1]} />
              </Grid>
              <Grid item xs={1}>
                <SensorCard title="Timestamp Drone" value={droneTimestamp} handleCardHover={() => handleCardHover(9)} hoverCard={hoverCard[9]} />
              </Grid>
              <Grid item xs={1}>
                <SensorCard title="Status Battery" value={droneBattery + " %"} handleCardHover={() => handleCardHover(2)} hoverCard={hoverCard[2]} />
              </Grid>
              <Grid item xs={1}>
                <SensorCard title="Altitude Drone" value={droneAltitude} handleCardHover={() => handleCardHover(3)} hoverCard={hoverCard[3]} />
              </Grid>
              <Grid item xs={1}>
                <SensorCard title="Heading Drone" value={droneHeading} handleCardHover={() => handleCardHover(10)} hoverCard={hoverCard[10]} />
              </Grid>
            </Grid>
            <Stack direction={"column"} padding="20px" gap="10px">
              <Grid container spacing={2} columns={3} width="100%" justifyContent={"center"}>
                <Grid item xs={1}>
                  <SensorCard title="Speed Drone (X)" value={droneSpeedX} handleCardHover={() => handleCardHover(4)} hoverCard={hoverCard[4]} />
                </Grid>
                <Grid item xs={1}>
                  <SensorCard title="Speed Drone (Y)" value={droneSpeedY} handleCardHover={() => handleCardHover(10)} hoverCard={hoverCard[10]} />
                </Grid>
                <Grid item xs={1}>
                  <SensorCard title="Speed Drone (Z)" value={droneSpeedZ} handleCardHover={() => handleCardHover(11)} hoverCard={hoverCard[11]} />
                </Grid>
              </Grid>
            </Stack>
          </Stack>
        </Stack>
      </Box>
    </Stack>
  );
};

export default Controls;
