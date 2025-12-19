import React from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';

const UrduFallbackPage = () => {
  return (
    <Layout title="Urdu Fallback Page">
      <div 
        style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          minHeight: '70vh',
          padding: '2rem',
          textAlign: 'center',
          direction: 'rtl', // RTL support for Urdu
        }}
      >
        <div 
          style={{
            backgroundColor: 'var(--ifm-card-background-color, #fff)',
            borderRadius: 'var(--ifm-global-radius)',
            boxShadow: 'var(--ifm-global-shadow-lw)',
            padding: '2rem',
            maxWidth: '600px',
            width: '100%',
          }}
        >
          <h1 
            style={{
              color: 'var(--ifm-heading-color)',
              marginBottom: '1.5rem',
              fontSize: '1.5rem',
            }}
          >
            معذرت، ہم پوری ویب سائٹ ترجمہ نہیں کر رہے ہیں۔
          </h1>
          <p 
            style={{
              fontSize: '1.1rem',
              color: 'var(--ifm-font-color-base)',
              marginBottom: '2rem',
            }}
          >
            Sorry, we are not translating the whole website.
          </p>
          
          <div 
            style={{
              display: 'flex',
              flexDirection: 'column',
              gap: '1rem',
              alignItems: 'center',
            }}
          >
            <Link
              to="/"
              className="button button--primary button--lg"
              style={{
                width: '100%',
                maxWidth: '300px',
                marginBottom: '0.5rem',
              }}
            >
              گھر پر واپس جائیں
            </Link>
            <Link
              to="/translator"
              className="button button--secondary button--lg"
              style={{
                width: '100%',
                maxWidth: '300px',
              }}
            >
              مترجم
            </Link>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default UrduFallbackPage;