import React, { useState } from "react";
import "../styles/Input.css"; // CSS 파일 임포트

function InputForm({ type, label, disabled = false }) {
  const [isFocused, setIsFocused] = useState(false); // focus 상태 관리

  return (
    <div className={`input-container ${isFocused ? "focused" : ""}`}>
      {/* <label>{label}</label> */}
      <input
        type={type}
        placeholder={label}
        onFocus={() => !disabled && setIsFocused(true)} // 비활성화가 아닌 경우에만 focus 상태 관리
        onBlur={() => !disabled && setIsFocused(false)} // 비활성화가 아닌 경우에만 focus 상태 관리
        disabled={disabled} // disabled 속성 적용
      />
    </div>
  );
}

export default InputForm;
