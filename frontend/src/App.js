import React, { useEffect, useState } from "react";
import { BrowserRouter as Router, Route, Routes } from "react-router-dom";
import LoginPage from "./pages/LoginPage";
import HomePage from "./pages/MainPage";
import StepPage1 from "./pages/coordinate/StepPage1";
import StepPage2 from "./pages/coordinate/StepPage2";
import StepPage3 from "./pages/coordinate/StepPage3";
import StepPage4 from "./pages/coordinate/StepPage4";
import StepPage5 from "./pages/coordinate/StepPage5";
import Layout from "./components/Layout";

function App() {
  return (
    <div>
      <Router>
        <Routes>
          <Route path="/login" element={<LoginPage />} />

          {/* Navbar가 있는 라우트 */}
          <Route element={<Layout />}>
            <Route path="/" element={<HomePage />} />
            <Route path="/step1" element={<StepPage1 />} />
            <Route path="/step2" element={<StepPage2 />} />
            <Route path="/step3" element={<StepPage3 />} />
            <Route path="/step4" element={<StepPage4 />} />
            <Route path="/step5" element={<StepPage5 />} />
            {/* 다른 라우트들도 여기에 추가 */}
          </Route>
        </Routes>
      </Router>
    </div>
  );
}

export default App;
