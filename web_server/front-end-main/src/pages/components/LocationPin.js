import React from "react";
import LocationOnIcon from "@mui/icons-material/LocationOn";
import { Typography, IconButton } from "@mui/material";

const LocationPin = ({ text, color }) => (
  <IconButton sx={{ display: "inline-block", transform: "none", transform: "translate(-50%, -50%)" }}>
    <LocationOnIcon sx={{ color: color }} />
    <Typography component="p" sx={{ color: color }}>
      {text}
    </Typography>
  </IconButton>
);

export default LocationPin;
