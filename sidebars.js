// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Modules',
      items: [
        {
          type: 'category',
          label: 'Module 1: ROS 2 Nervous System',
          items: [
            'modules/ros2-nervous-system/chapters/01/introduction',
            'modules/ros2-nervous-system/chapters/02/ros2-fundamentals',
            'modules/ros2-nervous-system/chapters/03/python-agents-bridging',
            'modules/ros2-nervous-system/chapters/04/urdf-humanoids',
            'modules/ros2-nervous-system/chapters/05/practical-integration',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          items: [
            'modules/digital-twin-sim/chapters/01/introduction-to-digital-twins',
            'modules/digital-twin-sim/chapters/02/gazebo-physics-theory',
            'modules/digital-twin-sim/chapters/03/unity-rendering-theory',
            'modules/digital-twin-sim/chapters/04/sensor-theory',
            'modules/digital-twin-sim/chapters/05/end-to-end-example',
          ],
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          items: [
            'modules/isaac-ai-brain/chapters/ai-brain-overview/index',
            'modules/isaac-ai-brain/chapters/isaac-sim-synthetic-data/index',
            'modules/isaac-ai-brain/chapters/isaac-ros-accelerated-perception/index',
            'modules/isaac-ai-brain/chapters/vslam-localization/index',
            'modules/isaac-ai-brain/chapters/nav2-bipedal-navigation/index',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA) Systems',
          items: [
            'modules/vla-systems/chapters/vla-overview/index',
            'modules/vla-systems/chapters/voice-to-action/index',
            'modules/vla-systems/chapters/cognitive-planning/index',
            'modules/vla-systems/chapters/vision-language-integration/index',
            'modules/vla-systems/chapters/autonomous-humanoid/index',
          ],
        },
        // Add more modules here as they are developed
      ],
    },
  ],
};

module.exports = sidebars;