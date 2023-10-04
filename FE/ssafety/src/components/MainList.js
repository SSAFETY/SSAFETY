import React, { useEffect, useState } from 'react';
import '../css/Home.css';
import axios from 'axios';
import '../css/Modal.css';

import { formatCreationTime } from './Modal';

const MainList = () => {
  const [data, setData] = useState([]);
  const [page, setPage] = useState(0);
  const [totalPages, setTotalPages] = useState(0);
  const limit = 10;
  const [isModalOpen, setIsModalOpen] = useState(false); // 모달 열림 상태
  const [selectedData, setSelectedData] = useState(null); // 모달에 표시할 데이터
  const [selectedImageData, setSelectedImageData] = useState(null);
  const [isLoading, setIsLoading] = useState(false); // 데이터 로딩 상태

  const openModal = async (data) => {
    try {
      const response = await axios.get(`http://localhost:8080/api/detailData?id=${data.id}`);
      const modalData = response.data;
      setSelectedData({ ...data, ...modalData });
      setIsModalOpen(true);
    } catch (error) {
      console.error(error);
    }
  };

  const closeModal = () => {
    setIsModalOpen(false);
  };

  const handleOverlayClick = (event) => {
    if (isModalOpen && event.target.classList.contains('modal-overlay')) {
      closeModal();
    }
  };

  useEffect(() => {
    // 데이터 요청 함수
    const fetchData = async () => {
      setIsLoading(true); // 데이터 로딩 시작
      try {
        const response = await axios.get(`https://j9a102.p.ssafy.io/getAll?page=${page}`);
        const responseData = response.data;
        const totalPages = Math.ceil(responseData.totalElements / limit);
        setTotalPages(totalPages);
        setData(responseData.content);
        setIsLoading(false);
      } catch (error) {
        console.error(error);
        setIsLoading(false);
      }
    };
  
    fetchData();
  }, [page]);

  const getPageData = () => {
    const startIndex = page * limit;
    const endIndex = startIndex + limit;
    return data.slice(startIndex, endIndex);
  };

  const displayedData = getPageData();

  const handleComplete = async () => {
    if (selectedData) {
      const phoneNumber = selectedData.phoneNum; // 선택된 데이터의 전화번호 추출
      try {
        const response = await axios.get(`https://j9a102.p.ssafy.io/api/success`, {phoneNumber});
      } catch (error) {
        console.error('Error:', error);
      }
    }
    closeModal();
  };
  
  const handleReject = async () => {
    if (selectedData) {
      const phoneNumber = selectedData.phoneNum;
      try {
        // 전화번호를 서버로 보내는 POST 요청
        const response = await axios.get(`https://j9a102.p.ssafy.io/api/fail`, {phoneNumber});
      } catch (error) {
        console.error('Error:', error);
      }
    }
    closeModal();
  };

  return (
    <div className="main-list-container">
      <div className="table-container">
      <div className="pagination">
        <button onClick={() => setPage(page - 1)} disabled={page === 0}>
          Previous
        </button>
        {Array.from({ length: totalPages }, (_, index) => (
          <button
            key={index}
            onClick={() => setPage(index)}
            className={page === index ? 'active' : ''}
          >
            {index + 1}
          </button>
        ))}
        <button onClick={() => setPage(page + 1)} disabled={page === totalPages - 1}>
          Next
        </button>
      </div>
      {isLoading ? (
        <p>Loading...</p>
      ) : (
        <table className="rwd-table">
          <thead>
            <tr>
              <th>위반 장소</th>
              <th>위반 종류</th>
              <th>일시</th>
              <th>차량 번호</th>
            </tr>
          </thead>
          <tbody id="table-body">
            {data.map((item, index) => {
              const timeDifference = calculateTimeDifference(item.creationTime);
              const isWithin24Hours = timeDifference < 24;
              let rowClass = '';
              if (isWithin24Hours) {
                rowClass = 'highlighted-row';
              }
              return (
                <tr
                  key={index}
                  className={rowClass}
                  onClick={() => openModal(item)} // 모달 열기
                >
                  <td data-th="address">
                    {item.city} {item.depth3}
                  </td>
                  <td data-th="violation">{item.aiResult}</td>
                  <td data-th="time">{formatCreationTime(item.creationTime)}</td>
                  <td data-th="carnum">
                    <img
                      src={item.vehicleNumber}
                      alt="차량 번호"
                      style={{ width: '100%', height: '100%', objectFit: 'cover' }}
                    />
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      )}
    </div>
      {isModalOpen && (
        <>
          <div className="modal-overlay" onClick={handleOverlayClick}></div>
          <div className={`modal ${isModalOpen ? 'active' : ''}`}>
            <div className="modal-content">
              <button className="close-modal" onClick={closeModal}></button>
              <h2>상세 정보</h2>
              {selectedImageData && (
                <div className="image-container">
                  <img
                    src={selectedImageData}
                    alt="차량 이미지"
                    style={{ width: '100%', height: '100%', objectFit: 'cover' }}
                  />
                </div>
              )}
              <table className="modal-table">
                <tbody>
                  <tr>
                    <td>위반 장소:</td>
                    <td>
                      {selectedData.city} {selectedData.depth3}
                    </td>
                  </tr>
                  <tr>
                    <td>위반 종류:</td>
                    <td>{selectedData.aiResult}</td>
                  </tr>
                  <tr>
                    <td>일시:</td>
                    <td>{formatCreationTime(selectedData.creationTime)}</td>
                  </tr>
                  <tr>
                    <td>차량 번호:</td>
                    <td>
                      <img src={selectedData.vehicleNumber} alt="차량 번호" />
                    </td>
                  </tr>
                </tbody>
              </table>
              {selectedData.videoUrl && (
                <div className="video-container">
                  <h3>사건 영상</h3>
                  <video controls>
                    <source src={selectedData.videoUrl} type="video/mp4" />
                    Your browser does not support the video tag.
                  </video>
                </div>
              )}
              <button className="green-button" onClick={handleComplete}>
          처리 완료
        </button>

        <button className="red-button" onClick={handleReject}>
          반려
        </button>
            </div>
          </div>
        </>
      )}
    </div>
  );
}

function calculateTimeDifference(creationTime) {
    const currentTime = new Date();
    const creationTimestamp = new Date(
      creationTime[0],
      creationTime[1] - 1,
      creationTime[2],
      creationTime[3],
      creationTime[4],
      creationTime[5]
    );
    const timeDifference = (currentTime - creationTimestamp) / (1000 * 60 * 60);
    return timeDifference;
}
  
export default MainList;