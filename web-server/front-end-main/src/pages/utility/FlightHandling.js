import mqtt from "mqtt/dist/mqtt";
import options from "./OptionsMQTT";

const setFLightConnection=(status) =>{
  let hasPublished = false;
  const port = 1884;
  const client = mqtt.connect(`wss://driver.cloudmqtt.com:${port}`, options);
    client.on(
      "connect",
      () => {
        console.log("MQTT client connected to the server.");
        if (!hasPublished) {
          client.publish("/drone/take_land", String(status), { qos: 0 });
          hasPublished = true;
        }
        return () => client.end();
      },
      []
    );
}

export const handleLanding = () => {
  setFLightConnection(0);
  };

export const handleTakeOff = () => {
  setFLightConnection(1);
};

