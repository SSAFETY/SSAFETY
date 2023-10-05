import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { feature } from 'topojson-client';
import korea from '../mapData/korea-topo.json';
import '../css/Home.css';
import { useNavigate } from 'react-router-dom';
import axios from 'axios';
import Swal from 'sweetalert2';
import '../css/Modal.css';

import { formatCreationTime } from './Modal';

const featureData = feature(korea, korea.objects['korea-topo']);

const KoreaMap = () => {
  const chart = useRef(null);
  const navigate = useNavigate();
  const [data, setData] = useState([]);
  const [page, setPage] = useState(0);
  const [totalPages, setTotalPages] = useState(0);
  const limit = 10;
  const [isModalOpen, setIsModalOpen] = useState(false); // 모달 열림 상태
  const [selectedData, setSelectedData] = useState(null); // 모달에 표시할 데이터
  const [selectedImageData, setSelectedImageData] = useState(null);
  const [isLoading, setIsLoading] = useState(false); // 데이터 로딩 상태

const openModal = async (data) => {
  // 모달을 열 때 데이터를 미리 로드
  try {
    const response = await axios.get(`https://j9a102.p.ssafy.io:8080/api/detailData?id=${data.id}`);
    const modalData = response.data;
    setSelectedData({ ...data, ...modalData }); // 기존 데이터와 모달 데이터를 합침
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

  const printD3 = () => {
    const width = window.innerWidth;
    const height = window.innerHeight;

    const projection = d3.geoMercator().scale(1).translate([0, 0]);
    const path = d3.geoPath().projection(projection);
    const bounds = path.bounds(featureData);

    const dx = bounds[1][0] - bounds[0][0];
    const dy = bounds[1][1] - bounds[0][1];
    const x = (bounds[0][0] + bounds[1][0]) / 2;
    const y = (bounds[0][1] + bounds[1][1]) / 2;
    const scale = 1 / Math.max(dx / width, dy / height);
    const translate = [width / 2 - scale * x - 350, height / 2 - scale * y];

    projection.scale(scale).translate(translate);

    const svg = d3.select(chart.current).append('svg').attr('width', width).attr('height', height);

    const mapLayer = svg.append('g');
    

    mapLayer
      .selectAll('path')
      .data(featureData.features)
      .enter()
      .append('path')
      .attr('d', path)
      .style('fill', 'skyblue')
      .style('transition', 'transform 0.2s');

    const popupGroup = svg.append('g').style('display', 'none');

    mapLayer
      .selectAll('path')
      .on('mouseover', function (event, d) {
        d3.select(this).style('fill', 'blue').attr('transform', 'translate(0, -5)');

        popupGroup.style('display', 'block');
        popupGroup.selectAll('*').remove();

        popupGroup
          .append('circle')
          .attr('cx', path.centroid(d)[0])
          .attr('cy', path.centroid(d)[1] - 30)
          .attr('r', 30)
          .style('fill', 'white')
          .style('stroke', 'blue')
          .style('stroke-width', 2);

        // 팝업 텍스트
        popupGroup
          .append('text')
          .attr('x', path.centroid(d)[0])
          .attr('y', path.centroid(d)[1] - 30)
          .attr('text-anchor', 'middle')
          .attr('alignment-baseline', 'middle')
          .text(d.properties.CTP_KOR_NM)
          .style('fill', 'black')
          .style('font-size', '12px');
      })
      .on('mouseout', function (event, d) {
        d3.select(this).style('fill', 'skyblue').attr('transform', 'translate(0, 0)');

        popupGroup.style('display', 'none');
      })
      .on('click', function (event, d) {
        if (d.properties.CTP_KOR_NM === '서울특별시') {
          navigate('detailsi');
        } else {
          Swal.fire('아직 지원하지 않는 구역입니다!');
        }
      });
  };

  useEffect(() => {
    printD3();
  }, []);

  useEffect(() => {
    // 데이터 요청 함수
    const fetchData = async () => {
      setIsLoading(true); // 데이터 로딩 시작
      try {
        const response = await axios.get(`http://localhost:8080/api/getAll?page=${page}`);
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
  }, [page]); // page 변수만 종속성으로 설정

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
        const response = await axios.post('https://j9a102.p.ssafy.io/api/success', { phoneNumber });
      } catch (error) {
        console.error('Error:', error);
      }
    }
    closeModal();
  };
  
  const handleReject = async () => {
    if (selectedData) {
      const phoneNumber = selectedData.phoneNum; // 선택된 데이터의 전화번호 추출
      try {
        const response = await axios.post('https://j9a102.p.ssafy.io/api/fail', { phoneNumber });
        // 요청이 성공하면 서버 응답을 처리하거나 다른 작업을 수행할 수 있습니다.
      } catch (error) {
        // 요청이 실패하면 에러를 처리하거나 오류 메시지를 표시할 수 있습니다.
        console.error('Error:', error);
      }
    }
    closeModal();
  };

  return (
    <div className="korea-map-container">
      <div className="korea-map" ref={chart}></div>
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

        {/* 빨간색 버튼 (반려) */}
        <button className="red-button" onClick={handleReject}>
          반려
        </button>
            </div>
          </div>
        </>
      )}
    </div>
  );
};

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

export default KoreaMap;
