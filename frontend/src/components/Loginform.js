import React, { useState } from "react";
// import axios from "axios";
import InputForm from "./Input";
import { useNavigate } from "react-router-dom";

function LoginForm() {
  const [id] = useState("admin@domain.com"); // 관리자 계정 자동 입력
  const [password, setPassword] = useState("");
  const navigate = useNavigate(); // useNavigate 훅 사용

  const handleLogin = async () => {
    // 로그인 로직
    try {
      const response = await fetch("/api/users/login", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          userEmail: id,
          userPassword: password,
        }),
      });

      if (response.ok) {
        // 로그인 성공 시 대시보드로 이동
        navigate("/home");
      } else {
        // 로그인 실패 처리
        console.log("Login failed");
      }
    } catch (error) {
      console.error("Error:", error);
    }
  };

  return (
    <div style={{ maxWidth: "300px", margin: "0 auto" }}>
      <div style={{ marginBottom: "20px" }}>
        <InputForm
          type="password"
          value={password}
          label="PASSWORD"
          onChange={(e) => setPassword(e.target.value)}
        />
      </div>
      <button
        style={{
          width: "100%",
          padding: "20px",
          backgroundColor: "#2d2938",
          color: "#fff",
          border: "none",
          borderRadius: "5px",
        }}
        onClick={handleLogin}
      >
        Login
      </button>
    </div>
  );
}

export default LoginForm;
