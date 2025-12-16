import React from 'react';
import Navbar from '@theme-original/Navbar';
import { useLocation } from '@docusaurus/router';

export default function NavbarWrapper(props) {
  const location = useLocation();
  
  // Add a robot icon to the navbar title for all pages except the home page
  const isHomePage = location.pathname === '/' || location.pathname === '/ai_book_2/';
  
  return (
    <>
      <Navbar {...props} />
    </>
  );
}