import React, { useState, useEffect } from "react";

const ParagraphView = ({ text, typeSpeed = 20 }) => {
  const [displayedText, setDisplayedText] = useState("");

  useEffect(() => {
    let charIndex = 0;
    const typeInterval = setInterval(() => {
      charIndex += 1;
      setDisplayedText(text.slice(0, charIndex));
      if (charIndex === text.length) {
        clearInterval(typeInterval);
      }
    }, typeSpeed);
    return () => clearInterval(typeInterval);
  }, [text, typeSpeed]);

  return <p style={{ textAlign: "center", marginTop: "5vh", transform: "translateY(0%)", margin: "10% 10%" }}>{displayedText}</p>;
};

export default ParagraphView;
