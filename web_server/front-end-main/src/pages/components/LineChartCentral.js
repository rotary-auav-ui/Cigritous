import React, { useState, useEffect } from "react";
import { Line } from "react-chartjs-2";
import axios from "axios";

function LineChartCentral({ title }) {
  const [data, setData] = useState({
    temperature: [],
    humidity: [],
    pressure: [],
    ozone: [],
    timestamp: [],
  });

  useEffect(() => {
    async function fetchData() {
      const response = await axios.get("https://vtol-cigritous-backend.herokuapp.com/updatecentral");
      const data = response.data;
      setData({
        temperature: data.map((item) => item.temperature).reverse(),
        humidity: data.map((item) => item.humidity).reverse(),
        pressure: data.map((item) => item.pressure).reverse(),
        ozone: data.map((item) => item.ozone).reverse(),
        timestamp: data.map((item) => item.timestamp).reverse(),
      });
    }
    fetchData();
  }, [data]);

  const labels = data.timestamp;

  const datasets = [
    {
      label: "Temperature",
      data: data.temperature,
      backgroundColor: "rgba(255, 99, 132, 0.2)",
      borderColor: "rgba(255, 99, 132, 1)",
      borderWidth: 1,
    },
    {
      label: "Humidity",
      data: data.humidity,
      backgroundColor: "rgba(54, 162, 235, 0.2)",
      borderColor: "rgba(54, 162, 235, 1)",
      borderWidth: 1,
    },
    {
      label: "Pressure",
      data: data.pressure,
      backgroundColor: "rgba(255, 206, 86, 0.2)",
      borderColor: "rgba(255, 206, 86, 1)",
      borderWidth: 1,
    },
    {
      label: "Ozone",
      data: data.ozone,
      backgroundColor: "rgba(75, 192, 192, 0.2)",
      borderColor: "rgba(75, 192, 192, 1)",
      borderWidth: 1,
    },
  ];

  return (
    <>
      <h3 className="text-center text-4xl font-bold text-pink-600 my-10">{title}</h3>
      <article className="w-full overflow-x-auto h-96 bg-white">
        <Line data={{ labels, datasets }} options={{ maintainAspectRatio: false }} />
      </article>
    </>
  );
}

export default LineChartCentral;
