import React, { useState, useEffect } from 'react';
import DatePicker from 'react-datepicker';
import 'react-datepicker/dist/react-datepicker.css';
import '../css/Violation.css';
import Swal from "sweetalert2";

// JSON 파일 경로
import districtsData from '../mapData/listdata.json';

const Violation = () => {
  // 선택한 도 (시) 및 구를 저장할 상태
  const [selectedProvince, setSelectedProvince] = useState('');
  const [selectedCity, setSelectedCity] = useState('');
  const [cityDistricts, setCityDistricts] = useState([]); // 초기값을 빈 배열로 설정

  // 선택한 위반 종류를 저장할 상태
  const [selectedViolation, setSelectedViolation] = useState('');

  // 선택한 일자를 저장할 상태
  const [selectedDate, setSelectedDate] = useState(null);

  // 필터된 데이터를 저장할 상태
  const [filteredData, setFilteredData] = useState([]);

  // 검색 조건 변경 시 자동 검색
  useEffect(() => {
    filterData();
  }, [selectedProvince, selectedCity, selectedViolation, selectedDate]);

  // 도 (시) 선택 핸들러
  const handleProvinceChange = (e) => {
    const selectedProvinceValue = e.target.value;
    setSelectedProvince(selectedProvinceValue);
    const selectedCityDistricts = districtsData[selectedProvinceValue] || [];
    setSelectedCity('');
    setCityDistricts(selectedCityDistricts);
  };

  // 구 선택 핸들러
  const handleCityChange = (e) => {
    setSelectedCity(e.target.value);
  };

  // 위반 종류 선택 핸들러
  const handleViolationChange = (e) => {
    setSelectedViolation(e.target.value);
  };

  // 일자 선택 핸들러
  const handleDateChange = (date) => {
    const timezoneOffsetMinutes = date.getTimezoneOffset();
    const adjustedDate = new Date(date.getTime() - timezoneOffsetMinutes * 60000);
    setSelectedDate(adjustedDate);
  };

  // 검색 함수
  const filterData = async () => {
    try {
      const dateValue = selectedDate ? selectedDate.toISOString().split('T')[0] : null;
      const response = await fetch('https://j9a102.p.ssafy.io/searchReports', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          state: selectedProvince,
          depth3: selectedCity,
          aiResult: selectedViolation,
          date: dateValue,
        }),
      });
      if (response.ok) {
        const data = await response.json();
        setFilteredData(data.content); // "content"에 실제 데이터가 들어있는 경우에 대한 처리
      } else {
        console.error('서버에서 오류 응답을 받았습니다.');
        setFilteredData([]); // 오류 발생 시 빈 배열로 초기화
      }
    } catch (error) {
      console.error('검색 요청 중 오류가 발생했습니다.', error);
      setFilteredData([]); // 오류 발생 시 빈 배열로 초기화
    }
  };

  const formatCreationTime = (creationTime) => {
    // creationTime을 문자열로 변환하여 앞에 0을 붙입니다.
    const timeString = String(creationTime).padStart(14, '0');
    const time = timeString.split(",")
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
  

  return (
    <div className="violation-container">
      {/* 검색 조건 */}
      <div className="violation-form">
        {/* 도 (시) 드롭다운 목록 */}
        <select value={selectedProvince} onChange={handleProvinceChange} className="violation-dropdown">
          <option value="">도 (시) 선택</option>
          {Object.keys(districtsData).map((province) => (
            <option key={province} value={province}>
              {province}
            </option>
          ))}
        </select>

        {/* 구 드롭다운 목록 */}
        <select value={selectedCity} onChange={handleCityChange} className="violation-dropdown">
          <option value="">구 선택</option>
          {cityDistricts.map((city) => (
            <option key={city} value={city}>
              {city}
            </option>
          ))}
        </select>

        {/* 위반 종류 선택 */}
        <select value={selectedViolation} onChange={handleViolationChange} className="violation-dropdown">
          <option value="">위반 종류 선택</option>
          <option value="차선 침범">차선 침범</option>
          <option value="신호 위반">신호 위반</option>
        </select>

        {/* 일자 선택 */}
        <DatePicker
          selected={selectedDate}
          onChange={handleDateChange}
          dateFormat="yyyy-MM-dd"
          isClearable
          className="date-picker"
          calendarIcon={<i className="fa fa-calendar" />}
          placeholderText="날짜를 선택해주세요"
        />

      </div>

      {/* 검색 결과 */}
      <div className="violation-result">
        <table>
          <thead>
            <tr>
              <th>도 (시)</th>
              <th>구</th>
              <th>위반 종류</th>
              <th>일자</th>
            </tr>
          </thead>
          <tbody>
            {filteredData.map((item) => (
              <tr key={item.id}>
                <td>{item.city}</td>
                <td>{item.depth3}</td>
                <td>{item.aiResult}</td>
                <td>{formatCreationTime(item.creationTime)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default Violation;
