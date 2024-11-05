import React, { useState, useRef, useEffect } from "react";
import { useNavigate, useLocation } from "react-router-dom";
import axios from "axios";
import "../../styles/StepPages.css";

const PYTHON_URL = process.env.REACT_APP_PYTHON_URL;

const Step4 = () => {
  const [alignPoints1, setAlignPoints1] = useState([]);
  const [alignPoints2, setAlignPoints2] = useState([]);
  const imageRef1 = useRef(null);
  const imageRef2 = useRef(null);
  const navigate = useNavigate();
  const location = useLocation();

  // Step3에서 넘겨받은 이미지 데이터를 가져옵니다.
  const { image1Src, image2Src } = location.state || {};
  const [mergedImageSrc, setMergedImageSrc] = useState("");

  useEffect(() => {
    console.log("전송 전의 이미지 상태 확인");

    if (!image1Src || !image2Src) {
      alert("이전 단계에서 이미지가 제공되지 않았습니다.");
    }
  }, [image1Src, image2Src, navigate]);

  const handleImageClick = (e, imgRef, setPoints, maxPoints) => {
    if (e.nativeEvent.which !== 1) return;
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

  // 대응점 좌표를 서버로 전송하고 Step5로 이동
  const handleUploadAlignPoints = () => {
    const formData = new FormData();
    formData.append("pts1_align", JSON.stringify(alignPoints1));
    formData.append("pts2_align", JSON.stringify(alignPoints2));

    axios
      .post(`${PYTHON_URL}/upload_align_points/`, formData)
      .then((response) => {
        // 서버에서 반환된 합성 이미지를 상태에 저장하고 Step5로 이동
        const mergedImage =
          "data:image/jpeg;base64," + response.data.merged_image;
        setMergedImageSrc(mergedImage);
        navigate("/step5", {
          state: {
            mergedImageSrc: mergedImage, // 서버로부터 받은 합성 이미지
          },
        });
      })
      .catch((error) => {
        console.error("대응점 업로드 에러:", error);
        alert("대응점 업로드 중 오류가 발생했습니다.");
      });
  };

  return (
    <>
      <div className="right-container">
        <h3>STEP4. 대응점 선택</h3>
        <div className="image-container">
          <div className="image-box">
            <img
              src={image1Src}
              alt="대응점 선택 이미지 1"
              ref={imageRef1}
              style={{
                cursor: alignPoints1.length < 4 ? "crosshair" : "default",
                maxWidth: "100%",
                height: "auto",
              }}
              onClick={(e) =>
                handleImageClick(e, imageRef1, setAlignPoints1, 4)
              }
            />
            <p>
              Image1 <span> {alignPoints1.length}/4</span>
            </p>
          </div>

          <div className="image-box">
            <img
              src={image2Src}
              alt="대응점 선택 이미지 2"
              ref={imageRef2}
              style={{
                cursor: alignPoints2.length < 4 ? "crosshair" : "default",
                maxWidth: "100%",
                height: "auto",
              }}
              onClick={(e) =>
                handleImageClick(e, imageRef2, setAlignPoints2, 4)
              }
            />
            <p>
              Image2 <span> {alignPoints2.length}/4</span>
            </p>
          </div>
        </div>

        <div className="option-container">
          <button
            onClick={handleUploadAlignPoints}
            className="submit-btn"
            disabled={alignPoints1.length < 4 || alignPoints2.length < 4}
          >
            다음
          </button>
        </div>
      </div>
    </>
  );
};

export default Step4;
