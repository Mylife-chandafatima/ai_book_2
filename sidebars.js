// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro', 'getting-started'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: Introduction to Physical AI',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-1/introduction',
        'modules/module-1/robotics-basics',
        'modules/module-1/ai-foundations',
        'modules/module-1/humanoid-design-principles',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Robot Perception',
      collapsible: true,
      collapsed: true,
      items: [
        'modules/module-2/perception-overview',
        'modules/module-2/sensor-fusion',
        'modules/module-2/computer-vision',
        'modules/module-2/vslam-systems',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Navigation & Mapping',
      collapsible: true,
      collapsed: true,
      items: [
        'modules/module-3/navigation-overview',
        'modules/module-3/path-planning',
        'modules/module-3/localization',
        'modules/module-3/mapping-algorithms',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Motion Control',
      collapsible: true,
      collapsed: true,
      items: [
        'modules/module-4/control-theory',
        'modules/module-4/trajectory-generation',
        'modules/module-4/bipedal-locomotion',
        'modules/module-4/manipulation-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Learning & Adaptation',
      collapsible: true,
      collapsed: true,
      items: [
        'modules/module-5/ml-for-robotics',
        'modules/module-5/reinforcement-learning',
        'modules/module-5/imitation-learning',
        'modules/module-5/adaptation-methods',
      ],
    },
    {
      type: 'category',
      label: 'Module 6: Applications & Integration',
      collapsible: true,
      collapsed: true,
      items: [
        'modules/module-6/human-robot-interaction',
        'modules/module-6/autonomous-systems',
        'modules/module-6/industrial-applications',
        'modules/module-6/emerging-trends',
      ],
    },
    {
      type: 'doc',
      id: 'references',
    },
    {
      type: 'link',
      label: 'English to Urdu Translator',
      href: '/translator',
    },
    {
      type: 'link',
      label: 'External Resources',
      href: 'https://www.ros.org/',
    }
  ],
};

module.exports = sidebars;