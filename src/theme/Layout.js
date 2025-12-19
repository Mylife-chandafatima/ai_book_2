import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import LocaleNavigationTop from '@site/src/components/LocaleNavigationTop';
import ChatbotWidget from '@site/src/components/Chatbot/ChatbotWidget';

export default function Layout(props) {
  return (
    <OriginalLayout {...props}>
      <LocaleNavigationTop />
      {props.children}
      <ChatbotWidget />
    </OriginalLayout>
  );
}