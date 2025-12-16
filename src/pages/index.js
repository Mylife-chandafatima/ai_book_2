// import React from 'react';
// import clsx from 'clsx';
// import Link from '@docusaurus/Link';
// import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
// import Layout from '@theme/Layout';
// import styles from './index.module.css';

// function HomepageHeader() {
//   const { siteConfig } = useDocusaurusContext();
//   return (
//     <header className={clsx('hero hero--primary', styles.heroBanner)}>
//       <div className="container">
//         <h1 className="hero__title">{siteConfig.title}</h1>
//         <p className="hero__subtitle">{siteConfig.tagline}</p>
//         <div className={styles.buttons}>
//           <Link
//             className="button button--secondary button--lg"
//             to="/docs/intro">
//             Get Started
//           </Link>
//         </div>
//       </div>
//     </header>
//   );
// }

// export default function Home() {
//   const { siteConfig } = useDocusaurusContext();

//   return (
//     <Layout
//       title={`Welcome to ${siteConfig.title}`}
//       description="A Comprehensive Guide to Robot Perception, Control, and Intelligence">
//       <HomepageHeader />
//       <main>
//         <div className="container">
//           <div className="row">
//             <div className="col col--4 padding-horiz--md">
//               <h2>Physical AI</h2>
//               <p>Explore the intersection of artificial intelligence and physical systems.</p>
//             </div>
//             <div className="col col--4 padding-horiz--md">
//               <h2>Humanoid Robotics</h2>
//               <p>Learn about the design and control of humanoid robots.</p>
//             </div>
//             <div className="col col--4 padding-horiz--md">
//               <h2>Practical Applications</h2>
//               <p>Discover real-world implementations and use cases.</p>
//             </div>
//           </div>
//         </div>
//       </main>
//     </Layout>
//   );
// }

import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="Complete AI-Native textbook for mastering robotics, humanoids, ROS2, VLA systems, and digital twins."
    >
      {/* HERO SECTION */}
      <header
        style={{
          padding: "80px 20px",
          textAlign: "center",
          background: "linear-gradient(135deg, #0b0f19, #1f2a44)",
          color: "white",
        }}
      >
        <h1 style={{ fontSize: "54px", fontWeight: "bold", marginBottom: "20px" }}>
          Physical AI & Humanoid Robotics Textbook
        </h1>
        <p style={{ fontSize: "22px", maxWidth: "850px", margin: "0 auto", lineHeight: "1.6" }}>
          A complete and practical learning system where you master the future:
          humanoid robotics, ROS 2, large action models, simulation, VLA systems, hardware,
          and advanced AI for next-generation intelligent machines.
        </p>

        <div style={{ marginTop: "40px" }}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro"
          >
            Start Reading →
          </Link>
        </div>
      </header>

      {/* ABOUT SECTION */}
      <section style={{ padding: "60px 20px", maxWidth: "1000px", margin: "0 auto" }}>
        <h2 style={{ fontSize: "36px", marginBottom: "20px", textAlign: "center" }}>
          What This Textbook Covers
        </h2>
        <p style={{ fontSize: "20px", lineHeight: "1.7", color: "#444", textAlign: "center" }}>
          This is a complete AI-native engineering curriculum designed for physical AI, humanoid robots,
          embodied intelligence, ROS 2 programming, digital twin simulations, and Vision-Language-Action
          (VLA) systems. Each module builds your robotics superpowers step by step.
        </p>
      </section>

      {/* MODULE CARDS */}
      <section style={{ padding: "60px 20px", background: "#f9fafc" }}>
        <h2 style={{ fontSize: "32px", marginBottom: "40px", textAlign: "center" }}>
          Explore All Modules
        </h2>

        <div
          style={{
            display: "grid",
            gridTemplateColumns: "repeat(auto-fit, minmax(280px, 1fr))",
            gap: "25px",
            maxWidth: "1200px",
            margin: "0 auto",
          }}
        >
          {/* MODULE 1 */}
          <div style={cardStyle}>
            <h3 style={cardTitle}>Module 1: The Robotic Nervous System (ROS 2)</h3>
            <p style={cardText}>
              Learn ROS 2 — the nervous system of modern robots. Build nodes, topics,
              services, actions, publishers, subscribers, QoS, and real robot workflows.
            </p>
            <ul style={listStyle}>
              <li><Link style={linkStyle} to="/docs/module-1-ros2/index">Introduction to ROS 2</Link></li>
              <li><Link style={linkStyle} to="/docs/module-1-ros2/nodes-topics-services">Nodes, Topics & Services</Link></li>
              <li><Link style={linkStyle} to="/docs/module-1-ros2/rclpy-bridge">Python Client Library (rclpy)</Link></li>
              <li><Link style={linkStyle} to="/docs/module-1-ros2/urdf-models">URDF Robot Models</Link></li>
              <li><Link style={linkStyle} to="/docs/module-1-ros2/practical-examples">Practical Examples</Link></li>
            </ul>
            <Link style={cardBtn} to="/docs/module-1-ros2/index">
              Open Module →
            </Link>
          </div>

          {/* MODULE 2 */}
          <div style={cardStyle}>
            <h3 style={cardTitle}>Module 2: Robot Perception</h3>
            <p style={cardText}>
              Master computer vision, sensor fusion, SLAM algorithms, and perception
              systems for robots to understand their environment.
            </p>
            <ul style={listStyle}>
              <li><Link style={linkStyle} to="/docs/modules/module-2/perception-overview">Perception Overview</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-2/sensor-fusion">Sensor Fusion</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-2/computer-vision">Computer Vision</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-2/vslam-systems">Visual SLAM Systems</Link></li>
            </ul>
            <Link style={cardBtn} to="/docs/modules/module-2/perception-overview">
              Open Module →
            </Link>
          </div>

          {/* MODULE 3 */}
          <div style={cardStyle}>
            <h3 style={cardTitle}>Module 3: Navigation & Mapping</h3>
            <p style={cardText}>
              Learn how robots navigate and map environments using localization,
              path planning, and mapping algorithms.
            </p>
            <ul style={listStyle}>
              <li><Link style={linkStyle} to="/docs/modules/module-3/navigation-overview">Navigation Overview</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-3/path-planning">Path Planning Algorithms</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-3/localization">Localization Techniques</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-3/mapping-algorithms">Mapping Algorithms</Link></li>
            </ul>
            <Link style={cardBtn} to="/docs/modules/module-3/navigation-overview">
              Open Module →
            </Link>
          </div>

          {/* MODULE 4 */}
          <div style={cardStyle}>
            <h3 style={cardTitle}>Module 4: Motion Control</h3>
            <p style={cardText}>
              Understand control theory, trajectory generation, locomotion, and manipulation
              for precise robot movement and interaction.
            </p>
            <ul style={listStyle}>
              <li><Link style={linkStyle} to="/docs/modules/module-4/control-theory">Control Theory Fundamentals</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-4/trajectory-generation">Trajectory Generation</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-4/bipedal-locomotion">Bipedal Locomotion</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-4/manipulation-control">Manipulation Control</Link></li>
            </ul>
            <Link style={cardBtn} to="/docs/modules/module-4/control-theory">
              Open Module →
            </Link>
          </div>

          {/* MODULE 5 */}
          <div style={cardStyle}>
            <h3 style={cardTitle}>Module 5: Learning & Adaptation</h3>
            <p style={cardText}>
              Explore machine learning, reinforcement learning, imitation learning,
              and adaptation methods for intelligent robots.
            </p>
            <ul style={listStyle}>
              <li><Link style={linkStyle} to="/docs/module-5/ml-for-robotics">ML for Robotics</Link></li>
              <li><Link style={linkStyle} to="/docs/module-5/reinforcement-learning">Reinforcement Learning</Link></li>
              <li><Link style={linkStyle} to="/docs/module-5/imitation-learning">Imitation Learning</Link></li>
              <li><Link style={linkStyle} to="/docs/module-5/adaptation-methods">Adaptation Methods</Link></li>
            </ul>
            <Link style={cardBtn} to="/docs/module-5/ml-for-robotics">
              Open Module →
            </Link>
          </div>

          {/* MODULE 6 */}
          <div style={cardStyle}>
            <h3 style={cardTitle}>Module 6: Applications & Integration</h3>
            <p style={cardText}>
              Discover real-world applications, human-robot interaction, industrial use cases,
              and emerging trends in robotics.
            </p>
            <ul style={listStyle}>
              <li><Link style={linkStyle} to="/docs/modules/module-6/human-robot-interaction">Human-Robot Interaction</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-6/autonomous-systems">Autonomous Systems</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-6/industrial-applications">Industrial Applications</Link></li>
              <li><Link style={linkStyle} to="/docs/modules/module-6/emerging-trends">Emerging Trends</Link></li>
            </ul>
            <Link style={cardBtn} to="/docs/modules/module-6/human-robot-interaction">
              Open Module →
            </Link>
          </div>

          {/* REFERENCES */}
          <div style={cardStyle}>
            <h3 style={cardTitle}>References & Resources</h3>
            <p style={cardText}>
              Glossary, research papers, references, external resources, and further reading
              for mastering robotics and AI.
            </p>
            <Link style={cardBtn} to="/docs/references">
              Access Resources →
            </Link>
          </div>
        </div>
      </section>

      {/* FEATURES SECTION */}
      <section style={{ padding: "80px 20px", background: "white" }}>
        <h2 style={{ textAlign: "center", fontSize: "34px", marginBottom: "50px" }}>
          Why This Textbook is AI-Native & Future-Focused
        </h2>

        <div
          style={{
            maxWidth: "1000px",
            margin: "0 auto",
            display: "grid",
            gridTemplateColumns: "repeat(auto-fit, minmax(280px, 1fr))",
            gap: "35px",
          }}
        >
          <div style={featureBox}>
            <h3>AI-Driven Design</h3>
            <p>
              Built fully around modern robotics workflows, from LLM agents to VLA systems
              and intelligent controllers.
            </p>
          </div>

          <div style={featureBox}>
            <h3>Hands-On Learning</h3>
            <p>
              Every module includes practical steps, code examples, simulations, and
              real robot applications.
            </p>
          </div>

          <div style={featureBox}>
            <h3>Industry-Inspired Curriculum</h3>
            <p>
              The content reflects what Tesla Bots, Figure AI, Apptronik, and Sanctuary AI
              use in real humanoid robotics pipelines.
            </p>
          </div>
        </div>
      </section>

      {/* CTA SECTION */}
      <section
        style={{
          padding: "90px 20px",
          background: "#0b0f19",
          color: "white",
          textAlign: "center",
        }}
      >
        <h2 style={{ fontSize: "40px", marginBottom: "20px" }}>
          Begin Your Robotics Journey
        </h2>
        <p style={{ fontSize: "20px", marginBottom: "40px", color: "#ccc" }}>
          The future belongs to physical AI, embodied intelligence, and humanoid robotics.
          Start mastering it today.
        </p>

        <Link
          className="button button--primary button--lg"
          to="/docs/intro"
        >
          Start Reading →
        </Link>
      </section>
    </Layout>
  );
}

/* ======== STYLES ======== */
const cardStyle = {
  background: "white",
  padding: "25px",
  borderRadius: "12px",
  boxShadow: "0 4px 12px rgba(0,0,0,0.08)",
};

const cardTitle = {
  fontSize: "22px",
  fontWeight: "bold",
  marginBottom: "10px",
};

const cardText = {
  fontSize: "16px",
  color: "#555",
  marginBottom: "15px",
  lineHeight: "1.5",
};

const listStyle = {
  listStyleType: "none",
  paddingLeft: 0,
  marginBottom: "15px",
};

const linkStyle = {
  display: "block",
  color: "#0057ff",
  textDecoration: "none",
  marginBottom: "5px",
  fontSize: "14px",
};

const linkStyleHover = {
  color: "#003bb3",
  textDecoration: "underline",
};

const cardBtn = {
  textDecoration: "none",
  background: "#0057ff",
  padding: "10px 16px",
  color: "white",
  borderRadius: "8px",
  fontSize: "15px",
  fontWeight: "bold",
};

const featureBox = {
  padding: "25px",
  background: "#f5f7fa",
  borderRadius: "10px",
  textAlign: "left",
};