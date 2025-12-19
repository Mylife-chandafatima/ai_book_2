import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

const Custom404 = () => {
  const { i18n } = useDocusaurusContext();
  const currentLocale = i18n.currentLocale;

  // Check if we're in the Urdu locale
  const isUrduLocale = currentLocale === 'ur';

  // Use the Urdu-specific message only for Urdu locale
  const title = isUrduLocale ? 'صفحہ نہیں ملا' : 'Page Not Found';
  const message = isUrduLocale
    ? 'ہم ویب سائٹ کو مکمل طور پر اردو میں ترجمہ کرنا شروع نہیں کیا ہے۔'
    : 'We are not translating the entire website to Urdu.';
  const buttonText = isUrduLocale ? 'ہوم پیج پر جائیں' : 'Go back to Home';

  // For non-Urdu locales, show a more generic message
  const defaultMessage = 'The page you are looking for does not exist.';

  return (
    <Layout title={title}>
      <main className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1 className="hero__title">{title}</h1>
            <p className="hero__subtitle">
              {isUrduLocale ? message : defaultMessage}
            </p>
            <div className="margin-vert--lg">
              <Link
                className="button button--primary button--lg"
                to="/"
              >
                {buttonText}
              </Link>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default Custom404;