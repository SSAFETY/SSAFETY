import React from 'react';

const Modal = ({ isOpen, closeModal, data }) => {
  if (!isOpen) return null;
  console.log(data)
  const formatCreationTime = (creationTimeArray) => {
    const [year, month, day, hour, minute] = creationTimeArray;
    const formattedTime = `${year}년 ${month}월 ${day}일 ${hour}시 ${minute}분`;
    return formattedTime;
  };

  return (
    <div className="modal">
      <div className="modal-content">
        <button className="close-modal" onClick={closeModal}>
          Close
        </button>
        <h2>상세 정보</h2>
        <p>도 (시): {data.city}</p>
        <p>구: {data.depth3}</p>
        <p>위반 종류: {data.aiResult}</p>
        <p>일자: {formatCreationTime(data.creationTime)}</p>
        <div className="video-container">
            <video controls>
              <source src={data.videoUrl} type="video/mp4" />
              Your browser does not support the video tag.
            </video>
        </div>
      </div>
    </div>
  );
};

export default Modal;
