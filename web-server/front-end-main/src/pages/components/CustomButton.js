import React from "react";
import Button from "@mui/material/Button";

const CustomButton = ({ href, label, hover, handleHover }) => (
  <Button
    onMouseEnter={handleHover}
    onMouseLeave={handleHover}
    style={{
      color: hover ? "#6841b0" : "white",
      fontSize: 20,
    }}
    href={href}
  >
    {label}
  </Button>
);

export default CustomButton;
