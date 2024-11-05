// App.js
import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import './App.css';

function App() {
  const [step, setStep] = useState(1);
  const [image1Src, setImage1Src] = useState(null);
  const [image2Src, setImage2Src] = useState(null);
  const [mergedImageSrc, setMergedImageSrc] = useState(null);
  const [selectedImage, setSelectedImage] = useState(1); // 현재 선택된 이미지 (1 또는 2)

  // 프레임 캡처용 비디오 요소에 대한 참조 (숨겨진 비디오)
  const videoCaptureRef1 = useRef(null);
  const videoCaptureRef2 = useRef(null);

  // 단계 5에서 표시할 비디오 요소에 대한 참조 (사용자에게 보여지는 비디오)
  const videoDisplayRef1 = useRef(null);
  const videoDisplayRef2 = useRef(null);

  // 이미지 요소에 대한 참조
  const imageRef1 = useRef(null);
  const imageRef2 = useRef(null);

  // 사용자 선택한 포인트
  const [floorPoints1, setFloorPoints1] = useState([]);
  const [floorPoints2, setFloorPoints2] = useState([]);

  // 타일 모서리 선택을 위한 포인트
  const [tilePoints1, setTilePoints1] = useState([]);
  const [tilePoints2, setTilePoints2] = useState([]);

  // 대응점 선택을 위한 포인트
  const [alignPoints1, setAlignPoints1] = useState([]);
  const [alignPoints2, setAlignPoints2] = useState([]);

  useEffect(() => {
    const video1 = videoCaptureRef1.current;
    const video2 = videoCaptureRef2.current;

    // 비디오 요소가 렌더링된 후에 이벤트 리스너를 추가
    if (video1 && video2) {
      // 비디오의 메타데이터가 로드된 후에 첫 프레임을 캡처
      const handleLoadedMetadata1 = () => {
        video1.currentTime = 0;
      };

      const handleLoadedMetadata2 = () => {
        video2.currentTime = 0;
      };

      const handleSeeked1 = () => {
        const frame1 = captureFrame(video1);
        setImage1Src(frame1);
      };

      const handleSeeked2 = () => {
        const frame2 = captureFrame(video2);
        setImage2Src(frame2);
      };

      video1.addEventListener('loadedmetadata', handleLoadedMetadata1);
      video2.addEventListener('loadedmetadata', handleLoadedMetadata2);

      video1.addEventListener('seeked', handleSeeked1);
      video2.addEventListener('seeked', handleSeeked2);

      return () => {
        video1.removeEventListener('loadedmetadata', handleLoadedMetadata1);
        video2.removeEventListener('loadedmetadata', handleLoadedMetadata2);

        video1.removeEventListener('seeked', handleSeeked1);
        video2.removeEventListener('seeked', handleSeeked2);
      };
    }
  }, []); // 초기 마운트 시에만 실행

  // 프레임 캡처 함수
  const captureFrame = (video) => {
    const canvas = document.createElement('canvas');
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
    const ctx = canvas.getContext('2d');
    ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
    return canvas.toDataURL('image/jpeg');
  };

  // 이미지 클릭 이벤트 핸들러 (최대 클릭 수 제한)
  const handleImageClick = (e, imgRef, setPoints, maxPoints) => {
    if (e.nativeEvent.which !== 1) return; // 왼쪽 마우스 버튼만 처리
    const img = imgRef.current;
    const rect = img.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const scaleX = img.naturalWidth / rect.width;
    const scaleY = img.naturalHeight / rect.height;
    const adjustedX = x * scaleX;
    const adjustedY = y * scaleY;

    setPoints(prevPoints => {
      if (prevPoints.length < maxPoints) {
        return [...prevPoints, [adjustedX, adjustedY]];
      } else {
        return prevPoints;
      }
    });
  };

  // 비디오 클릭 이벤트 핸들러 (바닥 좌표 요청)
  const handleVideoClick = (e, videoRef, imgId) => {
    const video = videoRef.current;
    const rect = video.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const scaleX = video.videoWidth / rect.width;
    const scaleY = video.videoHeight / rect.height;
    const adjustedX = x * scaleX;
    const adjustedY = y * scaleY;

    const formData = new FormData();
    formData.append('x', adjustedX);
    formData.append('y', adjustedY);
    formData.append('img_id', imgId);

    axios.post('http://localhost:8000/get_floor_coordinates/', formData)
      .then(response => {
        const x_floor = response.data.x_floor;
        const y_floor = response.data.y_floor;
        alert(`바닥 좌표: (${x_floor.toFixed(2)}, ${y_floor.toFixed(2)})`);
      })
      .catch(error => {
        console.error('바닥 좌표 요청 에러:', error);
        alert('바닥 좌표 요청 중 오류가 발생했습니다.');
      });
  };

  // 이미지 업로드 및 바닥의 네 끝점 좌표 서버로 전송
  const handleUploadImages = () => {
    const formData = new FormData();
    formData.append('image1', dataURLtoBlob(image1Src), 'frame1.jpg');
    formData.append('image2', dataURLtoBlob(image2Src), 'frame2.jpg');
    formData.append('pts1_floor', JSON.stringify(floorPoints1));
    formData.append('pts2_floor', JSON.stringify(floorPoints2));

    axios.post('http://localhost:8000/upload_images/', formData)
      .then(response => {
        setStep(response.data.step);
        setImage1Src('data:image/jpeg;base64,' + response.data.image1);
        setImage2Src('data:image/jpeg;base64,' + response.data.image2);
      })
      .catch(error => {
        console.error('이미지 업로드 에러:', error);
        alert('이미지 업로드 중 오류가 발생했습니다.');
      });
  };

  // dataURL을 Blob으로 변환하는 함수
  const dataURLtoBlob = (dataURL) => {
    const arr = dataURL.split(',');
    const mimeMatch = arr[0].match(/:(.*?);/);
    const mime = mimeMatch ? mimeMatch[1] : 'image/jpeg';
    const bstr = atob(arr[1]);
    let n = bstr.length;
    const u8arr = new Uint8Array(n);
    while(n--) {
        u8arr[n] = bstr.charCodeAt(n);
    }
    return new Blob([u8arr], { type: mime });
  };

  // 이미지 회전/반전 명령을 서버로 전송하는 함수
  const handleAdjustImages = (key) => {
    const formData = new FormData();
    formData.append('key', key);
    formData.append('img_id', selectedImage);

    axios.post('http://localhost:8000/adjust_images/', formData)
    .then(response => {
      setStep(response.data.step);
      setImage1Src('data:image/jpeg;base64,' + response.data.image1);
      setImage2Src('data:image/jpeg;base64,' + response.data.image2);
    })
    .catch(error => {
      console.error('이미지 조정 에러:', error);
      alert('이미지 조정 중 오류가 발생했습니다.');
    });
  };

  // 조정할 이미지를 선택하는 함수
  const handleSelectImage = (imgId) => {
    setSelectedImage(imgId);
  };

  // 타일 모서리 좌표를 서버로 전송하는 함수
  const handleUploadTilePoints = () => {
    const formData = new FormData();
    formData.append('pts1_tile', JSON.stringify(tilePoints1));
    formData.append('pts2_tile', JSON.stringify(tilePoints2));

    axios.post('http://localhost:8000/upload_tile_points/', formData)
      .then(response => {
        setStep(response.data.step);
        setImage1Src('data:image/jpeg;base64,' + response.data.image1);
        setImage2Src('data:image/jpeg;base64,' + response.data.image2);
      })
      .catch(error => {
        console.error('타일 포인트 업로드 에러:', error);
        alert('타일 포인트 업로드 중 오류가 발생했습니다.');
      });
  };

  // 대응점 좌표를 서버로 전송하는 함수
  const handleUploadAlignPoints = () => {
    const formData = new FormData();
    formData.append('pts1_align', JSON.stringify(alignPoints1));
    formData.append('pts2_align', JSON.stringify(alignPoints2));

    axios.post('http://localhost:8000/upload_align_points/', formData)
      .then(response => {
        setStep(response.data.step);
        setMergedImageSrc('data:image/jpeg;base64,' + response.data.merged_image);
      })
      .catch(error => {
        console.error('대응점 업로드 에러:', error);
        alert('대응점 업로드 중 오류가 발생했습니다.');
      });
  };

  return (
    <div className="App">
      <h1>현재 단계: {step}</h1>
      
      {/* 숨겨진 비디오 요소 (프레임 캡처용) */}
      <video ref={videoCaptureRef1} style={{ display: 'none' }} preload="metadata">
        <source src={process.env.PUBLIC_URL + '/cctv1.mp4'} type="video/mp4" />
      </video>
      <video ref={videoCaptureRef2} style={{ display: 'none' }} preload="metadata">
        <source src={process.env.PUBLIC_URL + '/cctv2.mp4'} type="video/mp4" />
      </video>

      {step === 1 && (
        <div>
          <h2>1. 이미지에서 바닥의 네 끝점 선택</h2>
          <div style={{ display: 'flex', flexDirection: 'row', gap: '20px' }}>
            <div>
              <h3>이미지 1</h3>
              {image1Src && (
                <img
                  src={image1Src}
                  alt="캡처된 이미지 1"
                  ref={imageRef1}
                  style={{ cursor: floorPoints1.length < 4 ? 'crosshair' : 'default', maxWidth: '100%', height: 'auto' }}
                  onClick={(e) => handleImageClick(e, imageRef1, setFloorPoints1, 4)}
                />
              )}
              <p>선택한 포인트 수: {floorPoints1.length} / 4</p>
            </div>
            <div>
              <h3>이미지 2</h3>
              {image2Src && (
                <img
                  src={image2Src}
                  alt="캡처된 이미지 2"
                  ref={imageRef2}
                  style={{ cursor: floorPoints2.length < 4 ? 'crosshair' : 'default', maxWidth: '100%', height: 'auto' }}
                  onClick={(e) => handleImageClick(e, imageRef2, setFloorPoints2, 4)}
                />
              )}
              <p>선택한 포인트 수: {floorPoints2.length} / 4</p>
            </div>
          </div>
          <button onClick={handleUploadImages} disabled={floorPoints1.length < 4 || floorPoints2.length < 4}>
            이미지 업로드 및 다음 단계로 진행
          </button>
        </div>
      )}
      
      {step === 2 && (
        <div>
          <h2>2. 이미지 회전/반전</h2>
          <div style={{ marginBottom: '10px' }}>
            <button onClick={() => handleSelectImage(1)}>이미지 1 선택</button>
            <button onClick={() => handleSelectImage(2)} style={{ marginLeft: '10px' }}>이미지 2 선택</button>
            <p>현재 선택된 이미지: 이미지 {selectedImage}</p>
          </div>
          <div style={{ marginBottom: '10px' }}>
            <button onClick={() => handleAdjustImages('r')}>90도 시계 방향 회전</button>
            <button onClick={() => handleAdjustImages('e')} style={{ marginLeft: '10px' }}>90도 반시계 방향 회전</button>
            <button onClick={() => handleAdjustImages('h')} style={{ marginLeft: '10px' }}>좌우 반전</button>
            <button onClick={() => handleAdjustImages('v')} style={{ marginLeft: '10px' }}>상하 반전</button>
            <button onClick={() => handleAdjustImages('n')} style={{ marginLeft: '10px' }}>다음 단계로 진행</button>
          </div>
          <div style={{ display: 'flex', flexDirection: 'row', gap: '20px' }}>
            <div>
              <h3>변환된 이미지 1</h3>
              {image1Src && (
                <img
                  src={image1Src}
                  alt="변환된 이미지 1"
                  style={{ maxWidth: '100%', height: 'auto' }}
                />
              )}
            </div>
            <div>
              <h3>변환된 이미지 2</h3>
              {image2Src && (
                <img
                  src={image2Src}
                  alt="변환된 이미지 2"
                  style={{ maxWidth: '100%', height: 'auto' }}
                />
              )}
            </div>
          </div>
        </div>
      )}
      
      {step === 3 && (
        <div>
          <h2>3. 타일 모서리 선택</h2>
          <div style={{ display: 'flex', flexDirection: 'row', gap: '20px' }}>
            <div>
              <h3>이미지 1</h3>
              {image1Src && (
                <img
                  src={image1Src}
                  alt="타일 선택 이미지 1"
                  ref={imageRef1}
                  style={{ cursor: tilePoints1.length < 4 ? 'crosshair' : 'default', maxWidth: '100%', height: 'auto' }}
                  onClick={(e) => handleImageClick(e, imageRef1, setTilePoints1, 4)}
                />
              )}
              <p>선택한 포인트 수: {tilePoints1.length} / 4</p>
            </div>
            <div>
              <h3>이미지 2</h3>
              {image2Src && (
                <img
                  src={image2Src}
                  alt="타일 선택 이미지 2"
                  ref={imageRef2}
                  style={{ cursor: tilePoints2.length < 4 ? 'crosshair' : 'default', maxWidth: '100%', height: 'auto' }}
                  onClick={(e) => handleImageClick(e, imageRef2, setTilePoints2, 4)}
                />
              )}
              <p>선택한 포인트 수: {tilePoints2.length} / 4</p>
            </div>
          </div>
          <button
            onClick={handleUploadTilePoints}
            disabled={tilePoints1.length < 4 || tilePoints2.length < 4}
          >
            타일 포인트 업로드 및 다음 단계로 진행
          </button>
        </div>
      )}
      
      {step === 4 && (
        <div>
          <h2>4. 이미지 합성을 위한 대응점 선택</h2>
          <div style={{ display: 'flex', flexDirection: 'row', gap: '20px' }}>
            <div>
              <h3>이미지 1</h3>
              {image1Src && (
                <img
                  src={image1Src}
                  alt="대응점 선택 이미지 1"
                  ref={imageRef1}
                  style={{ cursor: alignPoints1.length < 4 ? 'crosshair' : 'default', maxWidth: '100%', height: 'auto' }}
                  onClick={(e) => handleImageClick(e, imageRef1, setAlignPoints1, 4)}
                />
              )}
              <p>선택한 포인트 수: {alignPoints1.length} / 4</p>
            </div>
            <div>
              <h3>이미지 2</h3>
              {image2Src && (
                <img
                  src={image2Src}
                  alt="대응점 선택 이미지 2"
                  ref={imageRef2}
                  style={{ cursor: alignPoints2.length < 4 ? 'crosshair' : 'default', maxWidth: '100%', height: 'auto' }}
                  onClick={(e) => handleImageClick(e, imageRef2, setAlignPoints2, 4)}
                />
              )}
              <p>선택한 포인트 수: {alignPoints2.length} / 4</p>
            </div>
          </div>
          <button
            onClick={handleUploadAlignPoints}
            disabled={alignPoints1.length < 4 || alignPoints2.length < 4}
          >
            대응점 업로드 및 다음 단계로 진행
          </button>
        </div>
      )}
      
      {step === 5 && (
        <div>
          <h2>5. 영상에서 바닥 좌표 확인</h2>
          <p>비디오를 클릭하여 바닥 좌표를 확인하세요.</p>
          <div style={{ display: 'flex', flexDirection: 'row', gap: '20px', alignItems: 'flex-start' }}>
            <div>
              <h3>영상 1</h3>
              <video
                ref={videoDisplayRef1}
                src={process.env.PUBLIC_URL + '/cctv1.mp4'}
                controls
                style={{ maxWidth: '100%', height: 'auto', cursor: 'crosshair' }}
                onClick={(e) => handleVideoClick(e, videoDisplayRef1, 1)}
              />
            </div>
            <div>
              <h3>영상 2</h3>
              <video
                ref={videoDisplayRef2}
                src={process.env.PUBLIC_URL + '/cctv2.mp4'}
                controls
                style={{ maxWidth: '100%', height: 'auto', cursor: 'crosshair' }}
                onClick={(e) => handleVideoClick(e, videoDisplayRef2, 2)}
              />
            </div>
            <div>
              <h3>합성된 이미지</h3>
              {mergedImageSrc && (
                <img
                  src={mergedImageSrc}
                  alt="합성된 이미지"
                  style={{ maxWidth: '500px', height: 'auto' }}
                />
              )}
            </div>
          </div>
        </div>
      )}

    </div>
  );
}

export default App;
