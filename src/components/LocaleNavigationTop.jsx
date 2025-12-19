import React from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Link from '@docusaurus/Link';
import styles from './LocaleNavigationTop.module.css';

const LocaleNavigationTop = () => {
  const location = useLocation();
  const { i18n } = useDocusaurusContext();
  const currentLocale = i18n.currentLocale;

  // Only show the link on Urdu pages
  if (currentLocale !== 'ur') {
    return null;
  }

  return (
    <div className={styles.localeNavigationTop}>
      <div className={styles.localeNavigationTopInner}>
        <Link to="/" className={styles.localeNavLink}>
          واپس ہوم پیج پر جائیں
        </Link>
      </div>
    </div>
  );
};

export default LocaleNavigationTop;