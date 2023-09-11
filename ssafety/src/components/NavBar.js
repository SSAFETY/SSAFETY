// NavBar.js
import React from 'react';
import { Link } from 'react-router-dom';

const NavBar = () => {
  return (
    <nav>
      <ul>
        <li>
          <Link to="/">위반 목록</Link>
        </li>
        <li>
          <Link to="/about">통계</Link>
        </li>
        <li>
          <Link to="/contact">경로 설정</Link>
        </li>
        <li>
          <Link to="/contact">실시간 영상</Link>
        </li>
      </ul>
    </nav>
  );
};

export default NavBar;
