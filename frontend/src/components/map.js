import React, { useEffect, useState } from 'react';


const PYTHON_URL = process.env.REACT_APP_PYTHON_URL;
const LOCAL_URL = process.env.REACT_APP_LOCAL_URL;

const MapComponent = ({ onImageLoad, onPointReceive }) => {
  const [intervalId, setIntervalId] = useState(null);

  const createImage = async () => {
    try {
      const response = await fetch(`${PYTHON_URL}/get-map`, {
        method: 'GET',
      });

      if (response.ok) {
        const imageBlob = await response.blob();
        const url = URL.createObjectURL(imageBlob);
        onImageLoad(url); // 상위 컴포넌트에 이미지 URL 전달
      } else {
        console.error('이미지 요청 실패');
      }
    } catch (error) {
      console.error('요청 중 오류 발생:', error);
    }
  };

  const fetchCoordinates = async () => {
    try {
      const response = await fetch(`${PYTHON_URL}/get-coordinate`, {
        method: 'GET',
      });

      if (response.ok) {
        const data = await response.json(); 
        onPointReceive({ x: data.safetyCar[0], y: data.safetyCar[1] }); // 좌표를 상위 컴포넌트에 전달
      } else {
        console.error('좌표 요청 실패');
      }
    } catch (error) {
      console.error('좌표 요청 중 오류 발생:', error);
    }
  };

  useEffect(() => {
    createImage();

    // 1초마다 좌표 요청
    const id = setInterval(fetchCoordinates, 1500);
    setIntervalId(id);

    return () => clearInterval(id); // 컴포넌트 언마운트 시 인터벌 클리어
  }, []);

  return null; // 아무것도 렌더링하지 않음
};

export default MapComponent;
