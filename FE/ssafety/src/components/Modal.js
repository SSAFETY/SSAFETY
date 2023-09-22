import React from 'react';

export const formatCreationTime = (creationTime) => {
  // creationTime을 문자열로 변환하여 앞에 0을 붙입니다.
  const timeString = String(creationTime).padStart(14, '0');
  const time = timeString.split(',');
  // 연도, 월, 일, 시, 분, 초 부분 추출
  const year = time[0];
  const month = time[1];
  const day = time[2];
  const hour = time[3];
  const minute = time[4];

  // 변환된 문자열 생성
  const formattedTime = `${year}년 ${month}월 ${day}일 ${hour}시 ${minute}분`;

  return formattedTime;
};

// Modal.js 파일 내부의 모달 컴포넌트 업데이트
// Modal.js 파일 내부의 모달 컴포넌트 업데이트
const Modal = ({ isOpen, closeModal, data }) => {
    if (!isOpen) return null;
  
    return (
      <div className="modal">
        <div className="modal-content">
          <button className="close-modal" onClick={closeModal}>
            Close
          </button>
          <h2>상세 정보</h2>
          <p>위반 장소: {data.city} {data.depth3}</p>
          <p>위반 종류: {data.aiResult}</p>
          <p>일시: {formatCreationTime(data.creationTime)}</p>
          <p>차량 번호: {data.vehicleNumber}</p>
        </div>
      </div>
    );
  };
  
  export default Modal;
  
  
