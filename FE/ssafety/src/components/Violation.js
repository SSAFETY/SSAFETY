import React, { useState, useEffect } from 'react';
import DatePicker from 'react-datepicker';
import 'react-datepicker/dist/react-datepicker.css';
import '../css/Violation.css';

// JSON 파일 경로
import districtsData from '../mapData/listdata.json';

const Violation = () => {
  const [currentPage, setCurrentPage] = useState(1); // 현재 페이지 번호
  const [resultsPerPage] = useState(10); // 한 페이지에 표시할 결과 개수
  const [selectedProvince, setSelectedProvince] = useState('');
  const [selectedCity, setSelectedCity] = useState('');
  const [cityDistricts, setCityDistricts] = useState([]);
  const [selectedViolation, setSelectedViolation] = useState('');
  const [selectedDate, setSelectedDate] = useState(null);
  const [filteredData, setFilteredData] = useState([]);

  useEffect(() => {
    filterData();
  }, [selectedProvince, selectedCity, selectedViolation, selectedDate]);

  const handleProvinceChange = (e) => {
    const selectedProvinceValue = e.target.value;
    setSelectedProvince(selectedProvinceValue);
    const selectedCityDistricts = districtsData[selectedProvinceValue] || [];
    setSelectedCity('');
    setCityDistricts(selectedCityDistricts);
  };

  const handleCityChange = (e) => {
    setSelectedCity(e.target.value);
  };

  const handleViolationChange = (e) => {
    setSelectedViolation(e.target.value);
  };

  const handleDateChange = (date) => {
    const timezoneOffsetMinutes = date.getTimezoneOffset();
    const adjustedDate = new Date(date.getTime() - timezoneOffsetMinutes * 60000);
    setSelectedDate(adjustedDate);
  };

  const formatCreationTime = (creationTimeArray) => {
    const [year, month, day, hour, minute] = creationTimeArray;
    
    const date = new Date(year, month - 1, day, hour, minute);
  
    const formattedTime = `${year}년 ${month}월 ${day}일 ${hour}시 ${minute}분`;
    return formattedTime;
  };
  

  const filterData = async () => {
    try {
      const dateValue = selectedDate ? selectedDate.toISOString().split('T')[0] : null;
      const response = await fetch('https://j9a102.p.ssafy.io/api/searchReports', {
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
        console.log(data)
        setFilteredData(data.content);
      } else {
        console.error('서버에서 오류 응답을 받았습니다.');
        setFilteredData([]);
      }
    } catch (error) {
      console.error('검색 요청 중 오류가 발생했습니다.', error);
      setFilteredData([]);
    }
  };

  const totalPages = Math.ceil(filteredData.length / resultsPerPage);
  const indexOfLastResult = currentPage * resultsPerPage;
  const indexOfFirstResult = indexOfLastResult - resultsPerPage;
  const currentResults = filteredData.slice(indexOfFirstResult, indexOfLastResult);

  const handlePageChange = (pageNumber) => {
    setCurrentPage(pageNumber);
  };

  return (
    <div className="violation-container">
      <div className="violation-form">
        <select value={selectedProvince} onChange={handleProvinceChange} className="violation-dropdown">
          <option value="">도 (시) 선택</option>
          {Object.keys(districtsData).map((province) => (
            <option key={province} value={province}>
              {province}
            </option>
          ))}
        </select>

        <select value={selectedCity} onChange={handleCityChange} className="violation-dropdown">
          <option value="">구 선택</option>
          {cityDistricts.map((city) => (
            <option key={city} value={city}>
              {city}
            </option>
          ))}
        </select>

        <select value={selectedViolation} onChange={handleViolationChange} className="violation-dropdown">
          <option value="">위반 종류 선택</option>
          <option value="차선 침범">차선 침범</option>
          <option value="과속">과속</option>
          <option value="주정차 위반">주정차 위반</option>
          <option value="버스전용차로 위반">버스전용차로 위반</option>
        </select>

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

      <div className="violation-result">
  {currentResults.length === 0 ? (
    <p>해당 조건에 맞는 데이터가 없습니다.</p>
  ) : (
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
        {currentResults.map((item) => (
          <tr key={item.id}>
            <td>{item.city}</td>
            <td>{item.depth3}</td>
            <td>{item.aiResult}</td>
            <td>{formatCreationTime(item.creationTime)}</td>
          </tr>
        ))}
      </tbody>
    </table>
  )}
</div>

      <div className="pagination">
        {Array.from({ length: totalPages }, (_, index) => (
          <button
            key={index}
            onClick={() => handlePageChange(index + 1)}
            className={currentPage === index + 1 ? 'active' : ''}
          >
            {index + 1}
          </button>
        ))}
      </div>
    </div>
  );
};

export default Violation;
