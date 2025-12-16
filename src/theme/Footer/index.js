import React from 'react';
import Footer from '@theme-original/Footer';
import Link from '@docusaurus/Link';

export default function FooterWrapper(props) {
  return (
    <>
      <Footer {...props} />
      {/* Custom GitHub promotion footer */}
      <div style={{
        background: 'linear-gradient(135deg, #0b0f19, #1f2a44)',
        color: 'white',
        padding: '20px',
        textAlign: 'center',
        marginTop: '20px'
      }}>
        <div style={{ marginBottom: '10px', fontSize: '24px' }}>🤖</div>
        <h3 style={{ margin: '10px 0', fontSize: '18px' }}>Explore More on GitHub</h3>
        <div style={{ display: 'flex', justifyContent: 'center', gap: '20px', flexWrap: 'wrap' }}>
          <Link 
            to="https://github.com/Mylife-chandafatima" 
            style={{ 
              color: 'white', 
              textDecoration: 'none', 
              padding: '8px 16px', 
              background: 'rgba(255,255,255,0.1)', 
              borderRadius: '20px',
              display: 'flex',
              alignItems: 'center',
              gap: '5px'
            }}
            target="_blank"
          >
            <span>👤</span> My GitHub Profile
          </Link>
          <Link 
            to="https://github.com/Mylife-chandafatima/ai_book_2" 
            style={{ 
              color: 'white', 
              textDecoration: 'none', 
              padding: '8px 16px', 
              background: 'rgba(255,255,255,0.1)', 
              borderRadius: '20px',
              display: 'flex',
              alignItems: 'center',
              gap: '5px'
            }}
            target="_blank"
          >
            <span>📚</span> Textbook Repo
          </Link>
          <Link 
            to="https://github.com/Mylife-chandafatima/ai_book_2" 
            style={{ 
              color: 'white', 
              textDecoration: 'none', 
              padding: '8px 16px', 
              background: 'rgba(255,255,255,0.1)', 
              borderRadius: '20px',
              display: 'flex',
              alignItems: 'center',
              gap: '5px'
            }}
            target="_blank"
          >
            <span>⚙️</span> More Projects
          </Link>
        </div>
        <p style={{ marginTop: '15px', fontSize: '14px', opacity: 0.8 }}>
          Follow for more robotics & AI content
        </p>
      </div>
    </>
  );
}