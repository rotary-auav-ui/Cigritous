import React from "react";

const CoordinateCard = ({ title, value, handleCardHover, hoverCard }) => {
  return (
    <div onMouseEnter={handleCardHover} onMouseLeave={handleCardHover} className={`bg-black ${hoverCard ? "shadow-lg" : ""}`}>
      <div className="bg-purple-dark text-white text-center py-4 text-2xl">{title}</div>
      <div className="bg-purple-light h-24 flex items-center justify-center">
        <div>
          <h2 className="text-white text-3xl">{value}</h2>
        </div>
      </div>
    </div>
  );
};

export default CoordinateCard;
