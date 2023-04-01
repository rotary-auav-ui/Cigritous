import React from "react";

const SensorCard = ({ title, value, handleCardHover, hoverCard }) => {
  return (
    <div onMouseEnter={handleCardHover} onMouseLeave={handleCardHover} className={`bg-black ${hoverCard ? "shadow-lg" : ""}`}>
      <div className="bg-purple-dark text-white text-center">
        <div className="py-4 text-2xl">{title}</div>
      </div>
      <div className="bg-purple-light h-40 flex items-center justify-center">
        <div>
          <h2 className="text-white text-3xl">{value}</h2>
        </div>
      </div>
    </div>
  );
};
export default SensorCard;
