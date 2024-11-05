// components/Layout.js
import React, { useState, useEffect } from "react";
import Navbar from "./navibar"; // Navbar 컴포넌트 import
import { Outlet } from "react-router-dom";
import "../styles/Layout.css";
import { useNavigate } from "react-router-dom";

const Layout = () => {
  const navigate = useNavigate();
  // 현재 시간 상태 추가
  const [currentTime, setCurrentTime] = useState("");

  useEffect(() => {
    const updateTime = () => {
      const now = new Date();
      const formattedTime = now.toLocaleTimeString("ko-KR", {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
      });
      setCurrentTime(formattedTime);
    };
    updateTime();
    const timer = setInterval(updateTime, 1000);
    return () => clearInterval(timer);
  }, []);

  return (
    <>
      <div className="layout">
        {/* 헤더를 최상단에 위치 */}
        <header className="header-container">
          {/* 로고, h1, 현재 시간 등을 포함 */}
          {/* 헤더 컨테이너 */}
          <div className="logo">
            <img
              src="/assets/logo/SafetyCar-logo.png"
              alt="Safety Car Logo"
              className="logoimg"
              onClick={() => navigate("/")}
            />
          </div>
          <h1>안전 관제 센터</h1>
          <div className="current-time">
            <p>현재 시각</p>
            <div>{currentTime}</div>
          </div>
        </header>
        <div className="main-content">
          {/* Navbar와 페이지 콘텐츠를 포함하는 영역 */}
          <Navbar />
          <div className="content">
            <Outlet />
          </div>
        </div>
      </div>
    </>
  );
};

export default Layout;
