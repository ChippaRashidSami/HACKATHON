// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/nodes-topics-services',
        'module-1-ros2/python-agents-ros-control',
        'module-1-ros2/humanoid-modeling-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/physics-simulation-gazebo',
        'module-2-digital-twin/high-fidelity-unity',
        'module-2-digital-twin/simulated-sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-isaac/isaac-sim-synthetic-data',
        'module-3-isaac/isaac-ros-perception-vslam',
        'module-3-isaac/nav2-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) - Capstone',
      items: [
        'module-4-vla/speech-recognition',
        'module-4-vla/cognitive-planning',
        'module-4-vla/autonomous-humanoid',
      ],
    },
  ],
};

export default sidebars;