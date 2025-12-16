// import React from 'react';
// import Navbar from '@theme-original/Navbar';
// import { useLocation } from '@docusaurus/router';
// import Link from '@docusaurus/Link';

// export default function NavbarWrapper(props) {
//   const location = useLocation();

//   // Add a robot icon to the navbar title for all pages except the home page
//   const isHomePage = location.pathname === '/' || location.pathname === '/ai_book_2/';

//   return (
//     <>
//       {/* Custom GitHub banner above navbar */}
//       <div style={{
//         background: '#2c3e50',
//         color: 'white',
//         padding: '5px 0',
//         textAlign: 'center',
//         fontSize: '14px'
//       }}>
//         <Link
//           to="https://github.com/Mylife-chandafatima"
//           style={{
//             color: 'white',
//             textDecoration: 'none',
//             marginRight: '15px',
//             // display: 'inline-flex',
//             alignItems: 'center',
//             gap: '5px'
//           }}
//           target="_blank"
//         >
//           👤 Follow my GitHub
//         </Link>
//         <Link
//           to="https://github.com/Mylife-chandafatima/ai_book_2"
//           style={{
//             color: 'white',
//             textDecoration: 'none',
//             display: 'inline-flex',
//             alignItems: 'center',
//             gap: '5px'
//           }}
//           target="_blank"
//         >
//           ⭐ Star this textbook
//         </Link>
//       </div>
//       <Navbar {...props} />
//     </>
//   );
// }

import React from 'react';
import Navbar from '@theme-original/Navbar';
import { useLocation } from '@docusaurus/router';

export default function NavbarWrapper(props) {
  const location = useLocation();

  const isHomePage =
    location.pathname === '/' || location.pathname === '/ai_book_2/';

  return (
    <>
      {/* Custom GitHub banner above navbar */}
      <div
        style={{
          background: '#2c3e50',
          color: 'white',
          padding: '5px 0',
          textAlign: 'center',
          fontSize: '14px',
        }}
      >
        <a
          href="https://github.com/Mylife-chandafatima"
          target="_blank"
          rel="noopener noreferrer"
          style={{
            color: 'white',
            textDecoration: 'none',
            marginRight: '15px',
            display: 'inline-flex',
            alignItems: 'center',
            gap: '5px',
          }}
        >
          👤 Follow my GitHub
        </a>

        <a
          href="https://github.com/Mylife-chandafatima/ai_book_2"
          target="_blank"
          rel="noopener noreferrer"
          style={{
            color: 'white',
            textDecoration: 'none',
            display: 'inline-flex',
            alignItems: 'center',
            gap: '5px',
          }}
        >
          ⭐ Star this textbook
        </a>
      </div>

      <Navbar {...props} />
    </>
  );
}
