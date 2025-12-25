import React from 'react';
import clsx from 'clsx';
import ModuleCard from './ModuleCard';

import styles from './ModuleCards.module.css';

const ModuleCards = () => {
  // Module data with descriptions from existing content
  const modules = [
    {
      id: 'module-1',
      title: 'Module 1: ROS 2 Nervous System',
      description: 'Fundamentals of ROS 2 for humanoid robotics applications',
      path: '/docs/modules/ros2-nervous-system/chapters/01/introduction',
    },
    {
      id: 'module-2',
      title: 'Module 2: The Digital Twin (Gazebo & Unity)',
      description: 'Simulation environments for humanoid robotics development',
      path: '/docs/modules/digital-twin-sim/chapters/01/introduction-to-digital-twins',
    },
    {
      id: 'module-3',
      title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      description: 'AI processing and decision-making for humanoid systems',
      path: '/docs/modules/isaac-ai-brain/chapters/ai-brain-overview',
    },
    {
      id: 'module-4',
      title: 'Module 4: Vision-Language-Action (VLA) Systems',
      description: 'Integrated perception and action systems for humanoid robots',
      path: '/docs/modules/vla-systems/chapters/vla-overview',
    },
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className="row">
          <div className={clsx('col', styles.moduleGridContainer)}>
            <div className={clsx(styles.moduleGrid, 'module-grid')}>
              {modules.map((module) => (
                <div key={module.id} className="col col--6">
                  <ModuleCard module={module} />
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default ModuleCards;