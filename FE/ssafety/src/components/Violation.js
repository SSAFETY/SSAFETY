import React, { useState, useEffect } from 'react';
import DatePicker from 'react-datepicker';
import 'react-datepicker/dist/react-datepicker.css';
import '../css/Violation.css';

import districtsData from '../mapData/listdata.json';

import { FormControl, InputLabel, MenuItem, Select } from '@mui/material';
import Pagination from '@mui/material/Pagination';

const Violation = () => {
  const [currentPage, setCurrentPage] = useState(0);
  const [resultsPerPage] = useState(10);
  const [selectedProvince, setSelectedProvince] = useState('');
  const [selectedCity, setSelectedCity] = useState('');
  const [cityDistricts, setCityDistricts] = useState([]);
  const [selectedViolation, setSelectedViolation] = useState('');
  const [selectedDate, setSelectedDate] = useState(null);
  const [filteredData, setFilteredData] = useState(null);

  useEffect(() => {
    filterData();
  }, [selectedProvince, selectedCity, selectedViolation, selectedDate, currentPage]);

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
    if (date) {
      const timezoneOffsetMinutes = date.getTimezoneOffset();
      const adjustedDate = new Date(date.getTime() - timezoneOffsetMinutes * 60000);
      setSelectedDate(adjustedDate);
    } else {
      setSelectedDate(null);
    }
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
        setFilteredData(data);
      } else {
        console.error('서버에서 오류 응답을 받았습니다.');
        setFilteredData([]);
      }
    } catch (error) {
      console.error('검색 요청 중 오류가 발생했습니다.', error);
      setFilteredData([]);
    }
  };

  const totalPages = filteredData ? Math.ceil(filteredData.length / resultsPerPage) : 0;
  const indexOfLastResult = (currentPage + 1) * resultsPerPage;
  const indexOfFirstResult = indexOfLastResult - resultsPerPage;
  const currentResults = filteredData
    ? filteredData.slice(indexOfFirstResult, indexOfLastResult)
    : [];

  const handlePageClick = (data) => {
    setCurrentPage(data.selected);
  };

  return (
    <div className="violation-container">
      <div className="violation-form">
      <div className="form-control-horizontal">
        <FormControl variant="outlined">
          <InputLabel htmlFor="province-select">도 (시) 선택</InputLabel>
          <Select
            value={selectedProvince}
            onChange={handleProvinceChange}
            label="도 (시) 선택"
            id="province-select"
          >
            <MenuItem value="">
              <em>도 (시) 선택</em>
            </MenuItem>
            {Object.keys(districtsData).map((province) => (
              <MenuItem key={province} value={province}>
                {province}
              </MenuItem>
            ))}
          </Select>
        </FormControl>

        <FormControl variant="outlined">
          <InputLabel htmlFor="city-select">구 선택</InputLabel>
          <Select
            value={selectedCity}
            onChange={handleCityChange}
            label="구 선택"
            id="city-select"
          >
            <MenuItem value="">
              <em>구 선택</em>
            </MenuItem>
            {cityDistricts.map((city) => (
              <MenuItem key={city} value={city}>
                {city}
              </MenuItem>
            ))}
          </Select>
        </FormControl>

        <FormControl variant="outlined">
          <InputLabel htmlFor="violation-select">위반 종류 선택</InputLabel>
          <Select
            value={selectedViolation}
            onChange={handleViolationChange}
            label="위반 종류 선택"
            id="violation-select"
          >
            <MenuItem value="">
              <em>위반 종류 선택</em>
            </MenuItem>
            <MenuItem value="차선 침범">차선 침범</MenuItem>
            <MenuItem value="과속">과속</MenuItem>
            <MenuItem value="주정차 위반">주정차 위반</MenuItem>
            <MenuItem value="버스전용차로 위반">버스전용차로 위반</MenuItem>
          </Select>
        </FormControl>
        </div>

        <div className="date-picker-container">
  <div className="date-picker-input">
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
</div>



      </div>

      <div className="violation-result">
        {filteredData === null ? (
          <p>데이터를 불러오는 중입니다...</p>
        ) : filteredData.length === 0 ? (
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
        <Pagination
          count={totalPages}
          page={currentPage + 1}
          onChange={(event, page) => handlePageClick({ selected: page - 1 })}
          color="primary"
        />
      </div>
    </div>
  );
};

export default Violation;
