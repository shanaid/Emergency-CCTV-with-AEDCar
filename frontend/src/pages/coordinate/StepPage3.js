import React, { useState, useRef, useEffect } from "react";
import { useNavigate, useLocation } from "react-router-dom";
import axios from "axios";
import "../../styles/StepPages.css";

const PYTHON_URL = process.env.REACT_APP_PYTHON_URL;

const Step3 = () => {
  const [tilePoints1, setTilePoints1] = useState([]);
  const [tilePoints2, setTilePoints2] = useState([]);
  const imageRef1 = useRef(null);
  const imageRef2 = useRef(null);
  const navigate = useNavigate();
  const location = useLocation();
  const { image1Src: initialImage1Src, image2Src: initialImage2Src } =
    location.state || {}; // Step2에서 받은 이미지 데이터

  const [image1Src, setImage1Src] = useState(initialImage1Src || "");
  const [image2Src, setImage2Src] = useState(initialImage2Src || "");

  // 이미지 데이터가 정상적으로 들어왔는지 확인
  useEffect(() => {
    if (!image1Src || !image2Src) {
      console.error("이미지 데이터가 없습니다.");
      alert("이전 단계의 이미지 데이터를 불러오는데 문제가 발생했습니다.");
      // navigate("/step2"); // 이미지 데이터가 없으면 이전 단계로 돌아가기
    }
  }, [image1Src, image2Src, navigate]);

  // 이미지 클릭 이벤트 핸들러
  const handleImageClick = (e, imgRef, setPoints, maxPoints) => {
    if (e.nativeEvent.which !== 1) return; // 왼쪽 클릭만 처리
    const img = imgRef.current;
    const rect = img.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const scaleX = img.naturalWidth / rect.width;
    const scaleY = img.naturalHeight / rect.height;
    const adjustedX = x * scaleX;
    const adjustedY = y * scaleY;

    setPoints((prevPoints) => {
      if (prevPoints.length < maxPoints) {
        return [...prevPoints, [adjustedX, adjustedY]];
      } else {
        return prevPoints;
      }
    });
  };

  // 서버로 타일 포인트 업로드
  const handleUploadTilePoints = () => {
    if (tilePoints1.length < 4 || tilePoints2.length < 4) {
      alert("각 이미지에서 4개의 포인트를 선택해야 합니다.");
      return;
    }

    const formData = new FormData();
    formData.append("pts1_tile", JSON.stringify(tilePoints1));
    formData.append("pts2_tile", JSON.stringify(tilePoints2));

    // 서버로 좌표와 이미지를 전송하고 처리된 이미지를 받음
    axios
      .post(`${PYTHON_URL}/upload_tile_points/`, formData)
      .then((response) => {
        const processedImage1 =
          "data:image/jpeg;base64," + response.data.image1;
        const processedImage2 =
          "data:image/jpeg;base64," + response.data.image2;

        // 좌표를 전달하지 않고, 이미지 데이터만 전달
        navigate("/step4", {
          state: { image1Src: processedImage1, image2Src: processedImage2 },
        });
      })
      .catch((error) => {
        console.error("타일 포인트 업로드 에러:", error);
        alert("타일 포인트 업로드 중 오류가 발생했습니다.");
      });
  };

  return (
    <>
      <div className="right-container">
        <h3>STEP3. 타일 매칭</h3>
        <div className="image-container">
          <div className="image-box">
            <img
              src={image1Src}
              alt="타일 이미지 1"
              ref={imageRef1}
              style={{
                cursor: tilePoints1.length < 4 ? "crosshair" : "default",
              }}
              onClick={(e) => handleImageClick(e, imageRef1, setTilePoints1, 4)}
            />
            <p>
              Image1 <span> {tilePoints1.length}/4</span>
            </p>
          </div>
          <div className="image-box">
            <img
              src={image2Src}
              alt="타일 이미지 2"
              ref={imageRef2}
              style={{
                cursor: tilePoints2.length < 4 ? "crosshair" : "default",
              }}
              onClick={(e) => handleImageClick(e, imageRef2, setTilePoints2, 4)}
            />
            <p>
              Image2 <span> {tilePoints2.length}/4</span>
            </p>
          </div>
        </div>

        <div className="option-container">
          <button
            onClick={handleUploadTilePoints}
            className="submit-btn"
            disabled={tilePoints1.length < 4 || tilePoints2.length < 4}
          >
            다음
          </button>
        </div>
      </div>
    </>
  );
};

export default Step3;
