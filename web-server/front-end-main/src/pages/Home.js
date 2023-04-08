import React, { useEffect, useState } from "react";
import { Stack, Typography, Grid, IconButton, Box, Input } from "@mui/material";
import logo from "../logo_0.png";
import moment from "moment/moment";
import { Canvas, useFrame, useThree } from "react-three-fiber";
import { Physics, usePlane } from "@react-three/cannon";
import axios from "axios";
import GoogleMapReact from "google-map-react";
import { Chart as ChartJS, CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend } from "chart.js";
import { MDBContainer } from "mdbreact";
import "bootstrap/dist/css/bootstrap.min.css";
import mqtt from "mqtt/dist/mqtt";
import CorCard from "./components/CoordinateCard";
import SensorCard from "./components/SensorCard";
import Cube from "./components/Cube";
import LineChartCentral from "./components/LineChartCentral";
import LineChartNode from "./components/LineChartNode";
import LocationPin from "./components/LocationPin";
import LocationDrone from "./components/LocationDrone";
import CustomButton from "./components/CustomButton";

process.env.NODE_TLS_REJECT_UNAUTHORIZED = "0";

var options = {
  port: 38789,
  host: "wss://driver.cloudmqtt.com",
  clientId: "mqttjs_" + Math.random().toString(16).substr(2, 8),
  username: "cbobzrgp",
  password: "CKvOQLxrtuqc",
};

ChartJS.register(CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend);

const Home = () => {
  moment.locale("id");
  const [hoursTime, setHoursTime] = useState("");
  const [daysTime, setDaysTime] = useState("");
  const [timeStamp, SetTimeStamp] = useState("");
  const [mapsFlight, setMapsFlight] = useState([]);
  const [droneFlightLtd, setDroneFlightLtd] = useState([]);
  const [droneFlightLng, setDroneFlightLng] = useState([]);
  const [mapsFlightLtd, setMapsFlightLtd] = useState([]);
  const [mapsFlightLng, setMapsFlightLng] = useState([]);

  let totalNode = 20;

  const nodes = [];

  for (let index = 1; index <= totalNode; index++) {
    nodes.push(index);
  }

  const [centralTemp, setCentralTemp] = useState("");
  const [centralPress, setCentralPress] = useState("");
  const [centralHumid, setCentralHumid] = useState("");
  const [centralGas, setCentralGas] = useState("");
  const [nodeTemp, setNodeTemp] = useState([]);
  const [nodeMoist, setNodeMoist] = useState([]);
  const [nodeHumid, setNodeHumid] = useState([]);

  let arrTemp = [...nodeTemp];
  let arrHumid = [...nodeHumid];
  let arrMoist = [...nodeMoist];
  let arrLat = [...mapsFlightLtd];
  let arrLng = [...mapsFlightLng];

  const [showNode, setShowNode] = useState(Array(nodes.length).fill(false));
  const [hoverCard, setHoverCard] = useState(Array(nodes.length).fill(false));
  const [hoverDashboard, setHoverDashboard] = useState(false);
  const [hoverAbout, setHoverAbout] = useState(false);
  const [hoverControls, setHoverControls] = useState(false);

  const handleDashboardHover = () => setHoverDashboard(!hoverDashboard);
  const handleAboutHover = () => setHoverAbout(!hoverAbout);
  const handleControlsHover = () => setHoverControls(!hoverControls);

  const [showCentral, setShowCentral] = useState(false);

  const handleCardHover = (index) => {
    const newHoverCard = [...hoverCard];
    newHoverCard[index] = !newHoverCard[index];
    setHoverCard(newHoverCard);
  };

  const handleNodeClick = (index) => {
    let newShowNode = [...showNode];
    newShowNode[index] = !newShowNode[index];
    setShowNode(newShowNode);
  };

  const [attitude, setAttitude] = useState({
    yaw: 0.0,
    pitch: 0.0,
    roll: 0.0,
    att: 0.0,
    lat: -6.365232,
    lng: 106.824506,
  });

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

  const [mapType, setMapType] = useState("roadmap");

  const handleViewChange = () => {
    setMapType(mapType === "roadmap" ? "satellite" : "roadmap");
  };

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

  const handleResetLocation = () => {
    setMapsFlight([]);
    setMapsFlightLtd([]);
    setMapsFlightLng([]);
    setTitik(0);

    axios
      .post("https://vtol-cigritous-backend.herokuapp.com/resetsum")
      .then((response) => {
        console.log(response.data); // log the response from the server
      })
      .catch((error) => {
        console.log(error); // log any errors that occurred during the request
      });

    axios
      .post("https://vtol-cigritous-backend.herokuapp.com/resetcor")
      .then((response) => {
        console.log(response.data); // log the response from the server
      })
      .catch((error) => {
        console.log(error); // log any errors that occurred during the request
      });
  };

  useEffect(() => {
    const interval = setInterval(() => {
      SetTimeStamp(moment().format("HH:mm"));
      setHoursTime(moment().format("H:mm:ss"));
      setDaysTime(moment().format("ddd, DD MMMM YYYY"));
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  useEffect(
    () => {
      const client = mqtt.connect("wss://driver.cloudmqtt.com:1884", options);
      client.on("connect", () => {
        console.log("MQTT client connected to the server.");
        client.subscribe("/central/temp");
        client.subscribe("/central/press");
        client.subscribe("/central/humid");
        client.subscribe("/central/gas");
        client.subscribe("/drone/lat");
        client.subscribe("/drone/lng");
        for (let i = 1; i <= mapsFlight.length; i++) {
          client.publish("/" + i + "/coordinate", JSON.stringify(mapsFlight[i - 1]), { qos: 0 });
          client.publish(
            "/" + i + "/latitude",
            parseFloat(mapsFlightLtd[i - 1])
              .toFixed(6)
              .toString(),
            { qos: 0 }
          );
          client.publish(
            "/" + i + "/longitude",
            parseFloat(mapsFlightLng[i - 1])
              .toFixed(6)
              .toString(),
            { qos: 0 }
          );
        }

        for (let i = 1; i <= 20; i++) {
          client.subscribe("/" + i + "/temp");
          client.subscribe("/" + i + "/humid");
          client.subscribe("/" + i + "/moist");
          client.subscribe("/" + i + "/latitude");
          client.subscribe("/" + i + "/longitude");
        }
      });

      console.log("masuk config");
      client.on("message", (topic, message) => {
        console.log("tessss");
        console.log(centralGas);
        if (topic === "/drone/lat") {
          setDroneFlightLtd(message.toString());
        }
        if (topic === "/drone/lng") {
          setDroneFlightLng(message.toString());
        }
        if (topic === "/central/temp") {
          setCentralTemp(message.toString());
        }
        if (topic === "/central/press") {
          setCentralPress(message.toString());
        }
        if (topic === "/central/humid") {
          setCentralHumid(message.toString());
        }
        if (topic === "/central/gas") {
          setCentralGas(message.toString());
        }
        for (let i = 1; i <= 20; i++) {
          if (topic === "/" + i + "/temp") {
            arrTemp[i - 1] = message.toString();
            setNodeTemp(arrTemp);
          }
          if (topic === "/" + i + "/moist") {
            arrMoist[i - 1] = message.toString();
            console.log(message.toString());
            setNodeMoist(arrMoist);
          }
          if (topic === "/" + i + "/humid") {
            arrHumid[i - 1] = message.toString();
            setNodeHumid(arrHumid);
          }
          if (topic === "/" + i + "/latitude") {
            arrLat[i - 1] = message.toString();
            setMapsFlightLtd(arrLat);
          }
          if (topic === "/" + i + "/longitude") {
            arrLng[i - 1] = message.toString();
            setMapsFlightLng(arrLng);
          }
        }
      });
      return () => {
        client.end();
      };
    },
    [mapsFlight],
    [nodeTemp],
    [nodeHumid],
    [nodeMoist]
  );

  const [shouldSkip, setShouldSkip] = useState(true);

  useEffect(() => {
    const intervalId = setInterval(() => {
      const currentTime = new Date();
      const minutes = currentTime.getMinutes();
      const seconds = currentTime.getSeconds();
      const milliseconds = currentTime.getMilliseconds();

      // Calculate the time remaining until the next 10-minute mark
      const timeUntilNextTenMinutes = (2 - (minutes % 2)) * 60 * 1000 - seconds * 1000 - milliseconds;

      setTimeout(() => {
        setShouldSkip(false);
      });
    }, 5000);

    return () => {
      clearInterval(intervalId);
    };
  }, [shouldSkip]);

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

  useEffect(() => {
    console.log("Data Cor : ", mapsFlight[0]);
    console.log("Data Cor : ", mapsFlight[1]);
    console.log("Data Cor : ", mapsFlight[2]);
  }, [dataCor, mapsFlight]);

  useEffect(() => {
    const sendData = async () => {
      const dataCentral = {
        temperature: centralTemp,
        humidity: centralHumid,
        pressure: centralPress,
        ozone: centralGas,
        timestamp: timeStamp,
      };
      try {
        console.log("Data Central : ", dataCentral);
        if (!shouldSkip) {
          axios.post("https://vtol-cigritous-backend.herokuapp.com/insertcentral", dataCentral);
          console.log("Data Central sent to the backend");
        }
      } catch (error) {
        console.error("Error sending data to the backend: ", error);
      }

      for (let i = 1; i <= titik; i++) {
        const dataCoordinate = {
          node: i,
          latitude: mapsFlightLtd[i - 1],
          longitude: mapsFlightLng[i - 1],
          coordinate: mapsFlight[i - 1],
        };
        const dataNode = {
          node: i,
          temperature: arrTemp[i - 1],
          humidity: arrHumid[i - 1],
          moisture: arrMoist[i - 1],
          timestamp: timeStamp,
        };
        console.log("Data Node : ", dataNode);
        try {
          if (!shouldSkip) {
            axios.post("https://vtol-cigritous-backend.herokuapp.com/insertcoordinate", dataCoordinate);
            axios.post("https://vtol-cigritous-backend.herokuapp.com/insertnode", dataNode);
            console.log("Data Node sent to the backend");
            console.log(mapsFlight[i - 1]);
          }
        } catch (error) {
          console.error("Error sending data to the backend: ", error);
        }
      }
      setShouldSkip(true);
    };
    sendData();
  }, [arrTemp, arrHumid, arrMoist, shouldSkip, mapsFlight, mapsFlightLng, mapsFlightLtd]);

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
          Dashboard Cigritous
        </Typography>
        <Box padding="20px">
          <Typography fontSize="14px">Jumlah Node</Typography>
          <Input id="my-input" value={titik} sx={{ borderBottom: "1px solid #fffffff" }} onChange={(e) => setTitik(e.target.value)} />
        </Box>
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
              onClick={(e) => {
                if (mapsFlightLtd.length < titik) {
                  let arr = [...mapsFlight];
                  let arr1 = [...mapsFlightLtd];
                  let arr2 = [...mapsFlightLng];
                  arr.push({ lat: e.lat, lng: e.lng });
                  arr1.push(e.lat);
                  arr2.push(e.lng);
                  setMapsFlight(arr);
                  setMapsFlightLtd(arr1);
                  setMapsFlightLng(arr2);
                }
              }}
            >
              <LocationDrone lat={droneFlightLtd} lng={droneFlightLng} text="Drone" color="white" startLat={droneFlightLtd} startLong={droneFlightLng} />
              {mapsFlightLtd?.map((lat, idx) => (
                <LocationPin lat={lat} lng={mapsFlightLng[idx]} text={`Node ${idx + 1}`} color="yellow" />
              ))}
            </GoogleMapReact>
          </Stack>
          <div style={{ display: "flex", justifyContent: "space-between" }}>
            <button
              onMouseEnter={() => handleCardHover(24)}
              onMouseLeave={() => handleCardHover(24)}
              style={{ backgroundColor: "#3D3356", color: "white", padding: "10px 30px", border: "none", boxShadow: hoverCard[24] ? "0px 0px 20px 0px #000000" : "none" }}
              onClick={handleViewChange}
            >
              Switch to {mapType === "roadmap" ? "Satellite" : "Roadmap"} view
            </button>
            <button
              onMouseEnter={() => handleCardHover(1)}
              onMouseLeave={() => handleCardHover(1)}
              style={{ backgroundColor: "#3D3356", color: "white", padding: "10px 30px", border: "none", boxShadow: hoverCard[1] ? "0px 0px 20px 0px #000000" : "none" }}
              onClick={handleResetLocation}
            >
              Reset Location
            </button>
          </div>
          <Stack direction={"column"} padding="10px" gap="10px"></Stack>

          <div>
            {titik >= 0 && (
              <>
                <button
                  onMouseEnter={() => handleCardHover(2)}
                  onMouseLeave={() => handleCardHover(2)}
                  style={{ backgroundColor: "#3D3356", color: "white", padding: "10px 30px", border: "none", boxShadow: hoverCard[2] ? "0px 0px 20px 0px #000000" : "none" }}
                  onClick={() => setShowCentral(!showCentral)}
                >
                  Node 1 (Central)
                </button>

                <Stack direction={"column"} padding="20px" gap="10px">
                  {showCentral && (
                    <>
                      <CorCard title="Coordinate Position Central" value={`Ltd : ${JSON.stringify(mapsFlightLtd[0])} | Lng : ${JSON.stringify(mapsFlightLng[0])}`} handleCardHover={() => handleCardHover(3)} hoverCard={hoverCard[3]} />
                      <Stack direction={"column"} padding="20px" gap="10px">
                        <Grid container spacing={2} columns={3} width="100%" justifyContent={"center"}>
                          <Grid item xs={1}>
                            <SensorCard title="Temp Central" value={centralTemp + " °C"} handleCardHover={() => handleCardHover(4)} hoverCard={hoverCard[4]} />
                          </Grid>
                          <Grid item xs={1}>
                            <SensorCard title="Humidity Central" value={centralHumid + " %"} handleCardHover={() => handleCardHover(5)} hoverCard={hoverCard[5]} />
                          </Grid>
                          <Grid item xs={1}>
                            <SensorCard title="Pressure Central" value={centralPress + " %"} handleCardHover={() => handleCardHover(6)} hoverCard={hoverCard[6]} />
                          </Grid>
                          <Grid item xs={1}>
                            <SensorCard title="Ozone Central" value={centralGas + " %"} handleCardHover={() => handleCardHover(7)} hoverCard={hoverCard[7]} />
                          </Grid>
                        </Grid>
                      </Stack>
                    </>
                  )}
                </Stack>
              </>
            )}

            {nodes.slice(1, titik).map((node, index) => (
              <>
                <button
                  key={index}
                  onMouseEnter={() => handleCardHover(index)}
                  onMouseLeave={() => handleCardHover(index)}
                  style={{
                    backgroundColor: "#3D3356",
                    color: "white",
                    padding: "10px 30px",
                    border: "none",
                    boxShadow: hoverCard[index] ? "0px 0px 20px 0px #000000" : "none",
                  }}
                  onClick={() => handleNodeClick(index)}
                >
                  Node {index + 2}
                </button>
                <Stack direction={"column"} padding="20px" gap="10px">
                  {showNode[index] && (
                    <>
                      <CorCard
                        title={`Coordinate Position Node ${index + 2}`}
                        value={`Ltd : ${JSON.stringify(mapsFlightLtd[index + 1])} | Lng : ${JSON.stringify(mapsFlightLng[index + 1])}`}
                        handleCardHover={() => handleCardHover(index * 3)}
                        hoverCard={hoverCard[index * 3]}
                      />
                      <Stack direction={"column"} padding="20px" gap="10px">
                        <Grid container spacing={2} columns={3} width="100%" justifyContent={"center"}>
                          <Grid item xs={1}>
                            <SensorCard title={`Temp Node ${index + 2}`} value={`${nodeTemp[index]} °C`} handleCardHover={() => handleCardHover(index * 3 + 1)} hoverCard={hoverCard[index * 3 + 1]} />
                          </Grid>
                          <Grid item xs={1}>
                            <SensorCard title={`Humidity Node ${index + 2}`} value={`${nodeHumid[index]} %`} handleCardHover={() => handleCardHover(index * 3 + 2)} hoverCard={hoverCard[index * 3 + 2]} />
                          </Grid>
                          <Grid item xs={1}>
                            <SensorCard title={`Moisture Node ${index + 2}`} value={`${nodeMoist[index]} %`} handleCardHover={() => handleCardHover(index * 3 + 3)} hoverCard={hoverCard[index * 3 + 3]} />
                          </Grid>
                        </Grid>
                      </Stack>
                    </>
                  )}
                </Stack>
              </>
            ))}
          </div>

          <MDBContainer>
            <LineChartCentral title={"Line Chart Central"} />
            {nodes.slice(1, titik).map((node, index) => (
              <>
                <LineChartNode title={`Line Chart Node ${index + 2}`} node={index + 2} />
              </>
            ))}
          </MDBContainer>
        </Stack>
      </Box>
    </Stack>
  );
};

export default Home;
