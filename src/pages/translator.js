import React from 'react';
import Layout from '@theme/Layout';
import EnglishUrduTranslator from '../components/EnglishUrduTranslator';

function TranslatorPage() {
  return (
    <Layout
      title="English to Urdu Translator"
      description="A translator tool for converting English text to Urdu for the Physical AI & Humanoid Robotics book">
      <main style={{ padding: '2rem 0', maxWidth: '1200px', margin: '0 auto' }}>
        <div style={{ padding: '0 20px' }}>
          <h1 style={{ textAlign: 'center', marginBottom: '2rem' }}>English to Urdu Translator</h1>
          <p style={{ textAlign: 'center', marginBottom: '2rem', color: '#666' }}>
            Translate text between English and Urdu for the Physical AI & Humanoid Robotics book
          </p>

          <EnglishUrduTranslator />
        </div>
      </main>
    </Layout>
  );
}

export default TranslatorPage;