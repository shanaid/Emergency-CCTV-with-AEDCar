// Monitor.js
import { motion } from "framer-motion";
import React, { useState, useRef } from "react";
import "../styles/Mainpage.css"; // CSS 파일 import
import MapComponent from "../components/map";
import axios from "axios";
import Modal from "../components/modal";

const WEBSOCKET_URL = process.env.REACT_APP_WEBSOCKET_URL;
const PYTHON_URL = process.env.REACT_APP_PYTHON_URL;
const ws = new WebSocket(WEBSOCKET_URL);

function Monitor() {
  const [expandedIndex, setExpandedIndex] = useState(null);
  const [frameSrcArr, setFrameSrcArr] = useState([null, null, null, null]);
  const [simulatorImage, setSimulatorImage] = useState(null);
  const [points, setPoints] = useState([]);
  const imageRef1 = useRef(null);
  const imageRef2 = useRef(null);

  // 모달 관련 상태 추가
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [selectedImageRef, setSelectedImageRef] = useState(null);
  const [selectedImgId, setSelectedImgId] = useState(null);
  const [clickEvent, setClickEvent] = useState(null);

  ws.onmessage = async function (msg) {
    let newArr = [...frameSrcArr];
    const int8Array = new Int8Array(await msg.data.slice(0, 1).arrayBuffer());
    const idx = int8Array[0];
    newArr[idx] = URL.createObjectURL(msg.data.slice(1));
    setFrameSrcArr(newArr);
    const blob = new Blob([msg.data.slice(1)], { type: "image/jpeg" });
    const blobUrl = URL.createObjectURL(blob);
    newArr[idx] = blobUrl;

    // 각 카메라 프레임을 Local Storage에 저장
    if (idx === 0) {
      saveFrameToLocalStorage(blob, 0);
    } else if (idx === 1) {
      saveFrameToLocalStorage(blob, 1);
    }
  };

  // 이미지 저장 함수
  const saveFrameToLocalStorage = (blob, cameraIndex) => {
    const reader = new FileReader();
    reader.readAsDataURL(blob);
    reader.onloadend = function () {
      const base64Data = reader.result;
      localStorage.setItem(`savedImageCamera${cameraIndex}`, base64Data);
    };
  };

  const handleMouseEnter = (index) => {
    setExpandedIndex(index);
  };

  const handleMouseLeave = () => {
    setExpandedIndex(null);
  };

  const handleImageLoad = (url) => {
    setSimulatorImage(url);
  };

  // 이미지 클릭 시 모달을 열고 정보 저장
  const handleImageClick = (e, imageRef, imgId) => {
    setSelectedImageRef(imageRef);
    setSelectedImgId(imgId);
    // 이벤트 객체의 필요한 정보만 저장
    setClickEvent({
      clientX: e.clientX,
      clientY: e.clientY,
    });
    setIsModalOpen(true);
  };

  // 모달에서 확인 버튼 클릭 시 실행
  const handleImageClickConfirm = () => {
    if (selectedImageRef && selectedImgId && clickEvent) {
      const image = selectedImageRef.current;
      const rect = image.getBoundingClientRect();
      const x = clickEvent.clientX - rect.left;
      const y = clickEvent.clientY - rect.top;
      const scaleX = image.naturalWidth / rect.width;
      const scaleY = image.naturalHeight / rect.height;
      const adjustedX = x * scaleX;
      const adjustedY = y * scaleY;

      // 서버로 좌표 전송
      const formData = new FormData();
      formData.append("x", adjustedX);
      formData.append("y", adjustedY);
      formData.append("img_id", selectedImgId);

      axios
        .post(`${PYTHON_URL}/get_floor_coordinates/`, formData)
        .then((response) => {
          if (response.data.error) {
            alert(response.data.error);
          } else {
            const x_floor = response.data.x_floor;
            const y_floor = response.data.y_floor;
            alert(`바닥 좌표: (${x_floor.toFixed(2)}, ${y_floor.toFixed(2)})`);
          }
        })
        .catch((error) => {
          console.error("바닥 좌표 요청 에러:", error);
          alert("바닥 좌표 요청 중 오류가 발생했습니다.");
        })
        .finally(() => {
          setIsModalOpen(false);
        });
    }
  };

  // MapComponent에서 좌표를 받는 함수
  const handlePointReceive = (point) => {
    console.log(point);

    // point가 없거나 x, y가 undefined인 경우
    if (
      !point ||
      typeof point.x === "undefined" ||
      typeof point.y === "undefined"
    ) {
      setPoints([]);
      return;
    }

    const newX = (point.x / 500) * 400;
    const newY = (point.y / 500) * 400;

    setPoints([{ x: newX, y: newY }]);
  };

  return (
    <>
      <div className="container" style={{ display: "flex" }}></div>
      <div className="monitorSection">
        {/* 첫 번째 모니터: CCTV 1 */}
        <div className="monitorContainer">
          <motion.div
            className={`monitorFrameLeft ${
              expandedIndex === 0 ? "monitorFrameHovered" : ""
            }`}
            onMouseEnter={() => handleMouseEnter(0)}
            onMouseLeave={handleMouseLeave}
          >
            <div className="monitorScreen">
              <img
                src={frameSrcArr[0]}
                alt="CCTV 0"
                ref={imageRef1}
                onClick={(e) => handleImageClick(e, imageRef1, 1)}
              />
            </div>
            <div className="monitorStand"></div>
          </motion.div>
        </div>

        {/* 두 번째 모니터: CCTV 2 */}
        <div className="monitorContainer">
          <motion.div
            className={`monitorFrameRight ${
              expandedIndex === 1 ? "monitorFrameHovered" : ""
            }`}
            onMouseEnter={() => handleMouseEnter(1)}
            onMouseLeave={handleMouseLeave}
          >
            <div className="monitorScreen">
              <img
                src={frameSrcArr[1]}
                alt="CCTV 1"
                ref={imageRef2}
                onClick={(e) => handleImageClick(e, imageRef2, 2)}
              />
            </div>
            <div className="monitorStand"></div>
          </motion.div>
        </div>
      </div>

      {/* 시뮬레이터 지도 섹션 */}
      <div className="simulatorOverlay">
        <MapComponent
          onImageLoad={handleImageLoad}
          onPointReceive={handlePointReceive}
        />

        {/* 시뮬레이터 이미지 추가 */}
        {simulatorImage && (
          <div
            className="simulatorImageContainer"
            style={{ position: "relative" }}
          >
            <img
              src={simulatorImage}
              alt="Simulator"
              style={{
                width: "100%",
                height: "100%",
                objectFit: "contain",
                borderRadius: "20px",
                opacity: "0.3",
              }}
            />
            {/* 좌표 표시 */}
            {points.length > 0 &&
              points.map((point, index) => (
                <div
                  key={index}
                  style={{
                    position: "absolute",
                    left: point.x,
                    top: point.y,
                    width: "10px",
                    height: "10px",
                    borderRadius: "50%",
                    backgroundColor: "red",
                    transform: "translate(-50%, -50%)",
                  }}
                />
              ))}
          </div>
        )}
      </div>

      {/* 모달 컴포넌트 추가 */}
      <Modal
        isOpen={isModalOpen}
        onClose={() => setIsModalOpen(false)}
        title="강제 출동"
        content="클릭한 위치로 SafetyCar를 출동시키겠습니까?"
      >
        <button onClick={handleImageClickConfirm}>확인</button>
        <button onClick={() => setIsModalOpen(false)}>취소</button>
      </Modal>
    </>
  );
}

export default Monitor;
