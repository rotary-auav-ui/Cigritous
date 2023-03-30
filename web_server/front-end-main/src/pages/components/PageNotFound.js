import React, { useState } from "react";

const PageNotFound = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggle = () => {
    setIsOpen(!isOpen);
  };
  return (
    <div className={`${isOpen ? "fixed h-screen w-full" : ""} flex flex-col min-h-screen min-w-[300px]`}>
      <div className="flex flex-col mx-20 h-full justify-center items-center">
        <Drone />
        <p className="text-center absolute bottom-0 mb-6"> Halaman Sedang Maintenance Yach...</p>
      </div>
    </div>
  );
};
export default PageNotFound;
