import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { useTheme } from '../components/theme/ThemeProvider';
import ModuleCards from '../components/ModuleCard/ModuleCards';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const { toggleTheme } = useTheme();

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
        <p className="hero__subtitle">Embodied Intelligence for the Real World</p>
        <p>
          Explore the integration of physical intelligence, humanoid systems, ROS 2, simulation,
          perception, planning, and real-world deployment in advanced robotics.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/modules/ros2-nervous-system/chapters/01/introduction">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Professional Docusaurus UI for Physical AI & Humanoid Robotics textbook">
      <HomepageHeader />
      <main>
        <ModuleCards />
      </main>
    </Layout>
  );
}