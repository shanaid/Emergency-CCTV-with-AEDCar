import React, { useState } from "react";
import { useNavigate } from "react-router-dom"; // useNavigate 훅을 import
import "../styles/navibar.css"; // CSS 파일 import
import axios from "axios";
import Modal from "./modal";

const Navbar = () => {
  const navigate = useNavigate();
  const PYTHON_URL = process.env.REACT_APP_PYTHON_URL;
  const API_URL = process.env.REACT_APP_API_URL;

  // 신고 모달 상태
  const [isModalOpen, setIsModalOpen] = useState(false);

  // Rotate 모달 상태 추가
  const [isRotateModalOpen, setIsRotateModalOpen] = useState(false);

  // 신고 모달 열기/닫기 함수
  const handleOpenModal = () => {
    setIsModalOpen(true); // 모달 열기
  };

  const handleCloseModal = () => {
    setIsModalOpen(false); // 모달 닫기
  };

  // Rotate 모달 열기/닫기 함수 추가
  const handleOpenRotateModal = () => {
    setIsRotateModalOpen(true); // Rotate 모달 열기
  };

  const handleCloseRotateModal = () => {
    setIsRotateModalOpen(false); // Rotate 모달 닫기
  };

  // Rotate 확인 함수 수정
  const handleRotateConfirm = async () => {
    try {
      const response = await axios.get(`${PYTHON_URL}/safety_Car/halt`);
      console.log("Response:", response.data);
    } catch (error) {
      console.error("Error making GET request:", error);
      alert("요청 처리 중 오류가 발생했습니다.");
    } finally {
      setIsRotateModalOpen(false); // 모달 닫기
    }
  };

  // 신고 전송 함수
  const handleSendSMS = () => {
    const space = "삼성화재 유성캠퍼스(SSAFY 교육동)";
    axios
      .post(`${API_URL}/sms/send?space=${space}`)
      .then((response) => {
        console.log(response.data);
        console.log("신고 성공");
        alert("신고가 성공적으로 접수되었습니다.");
      })
      .catch((error) => {
        console.error("신고 접수 중 에러 발생", error);
        alert("신고에 실패했습니다. 다시 시도해주세요.");
      })
      .finally(() => {
        setIsModalOpen(false); // 모달 닫기
      });
  };

  return (
    <div className="navbar">
      <div className="nav-item" onClick={() => navigate("/")}>
        <img src="/assets/navi/Icon.png" alt="Home" className="nav-icon" />
        {/* Home 아이콘 */}
      </div>
      <div className="nav-item" onClick={() => navigate("/step1")}>
        <img
          src="/assets/navi/Settings.png"
          alt="Settings"
          className="nav-icon"
        />
        {/* Settings 아이콘 */}
      </div>
      <div className="nav-item" onClick={handleOpenModal}>
        <img src="/assets/navi/Mail.png" alt="Mail" className="nav-icon" />
        {/* Mail 아이콘 */}
      </div>
      <div className="nav-item" onClick={handleOpenRotateModal}>
        <img
          src="/assets/navi/Rotate cw (1).png"
          alt="Rotate"
          className="nav-icon"
        />
        {/* Rotate 아이콘 */}
      </div>

      {/* 신고 모달 */}
      <Modal
        isOpen={isModalOpen}
        onClose={handleCloseModal}
        title="신고 확인"
        content={"119 신고 센터에 문자가 전송됩니다.\n신고를 진행하시겠습니까?"}
      >
        <button onClick={handleSendSMS}>확인</button>
        <button onClick={handleCloseModal}>취소</button>
      </Modal>

      {/* Rotate 모달 추가 */}
      <Modal
        isOpen={isRotateModalOpen}
        onClose={handleCloseRotateModal}
        title="SafetyCar 강제 복귀"
        content="SafetyCar를 복귀시키겠습니까?"
      >
        <button onClick={handleRotateConfirm}>확인</button>
        <button onClick={handleCloseRotateModal}>취소</button>
      </Modal>
    </div>
  );
};

export default Navbar;
