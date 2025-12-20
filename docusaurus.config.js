// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Robot Perception, Control, and Intelligence',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://Mylife-chandafatima.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<username>.github.io/<repo-name>'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Mylife-chandafatima', // Usually your GitHub org/user name.
  projectName: 'ai_book_2', // Usually your repo name.
  trailingSlash: true, // Add trailing slash to avoid GitHub Pages redirect issues

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Mylife-chandafatima/ai_book_2/edit/main/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
        sitemap: {
          changefreq: 'weekly',
          priority: 0.5,
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      // Add custom fields for chatbot
      customFields: {
        enableChatbot: true,
      },
      metadata: [
        { name: 'keywords', content: 'ai, robotics, machine learning, book' },
      ],
      navbar: {
        title: '🤖 Physical AI & Robotics',
        logo: {
          alt: 'Robotics Book Logo',
          src: 'img/logo.svg', // Use the existing logo or favicon
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {
            to: '/translator',
            label: 'Translator',
            position: 'left',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/Mylife-chandafatima',
            label: 'GitHub Profile',
            position: 'right',
          },
          {
            href: 'https://github.com/Mylife-chandafatima/ai_book_2',
            label: 'Textbook Repo',
            position: 'right',
          },
        ],
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Module 1: Introduction to Physical AI',
                to: '/docs/modules/module-1/introduction',
              },
              {
                label: 'Module 2: Robot Perception',
                to: '/docs/modules/module-2/perception-overview',
              },
              {
                label: 'Module 3: Navigation & Mapping',
                to: '/docs/modules/module-3/navigation-overview',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/humanoid-robotics',
              },
              {
                label: 'ROS Discourse',
                href: 'https://discourse.ros.org/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub Profile',
                href: 'https://github.com/Mylife-chandafatima',
              },
              {
                label: 'Textbook Repository',
                href: 'https://github.com/Mylife-chandafatima/ai_book_2',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} <a href="https://github.com/Mylife-chandafatima" target="_blank" rel="noopener noreferrer">Physical AI & Humanoid Robotics Book</a>. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'bash', 'yaml', 'cpp'], // Add languages for ROS2 code
      },
      algolia: {
        // The application ID provided by Algolia
        appId: 'YOUR_APP_ID',
        // Public API key: it is safe to commit it
        apiKey: 'YOUR_SEARCH_API_KEY',
        indexName: 'your-book-index',
        contextualSearch: true,
        searchPagePath: 'search',
      },
    }),

};

module.exports = config;