import React from "react";
import { FaPlane } from "react-icons/fa";
import { Typography, IconButton } from "@mui/material";

const LocationDrone = ({ text, color }) => (
  <IconButton sx={{ display: "inline-block", transform: "none", transform: "translate(-50%, -50%)" }}>
    <FaPlane color="white" />
    <Typography component="p" sx={{ color: color }}>
      {text}
    </Typography>
  </IconButton>
);

export default LocationDrone;
