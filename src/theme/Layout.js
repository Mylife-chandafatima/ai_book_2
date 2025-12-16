import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatbotWidget from '@site/src/components/ChatbotWidget/ChatbotWidget';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Layout(props) {
  const { siteConfig } = useDocusaurusContext();

  // Since Docusaurus runs client-side after hydration,
  // we can safely check if we're in the browser environment
  const [showChatbot, setShowChatbot] = React.useState(false);

  React.useEffect(() => {
    // Enable chatbot in all environments to make it visible
    setShowChatbot(true);
  }, []);

  return (
    <>
      <OriginalLayout {...props} />
      {/* Show chatbot on all pages */}
      {showChatbot && <ChatbotWidget />}
    </>
  );
}