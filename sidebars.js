// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Modules',
      items: [
        {
          type: 'category',
          label: 'ROS 2 as Robotic Nervous System',
          link: {type: 'doc', id: 'modules/ros2-nervous-system/index'},
          items: [
            'modules/ros2-nervous-system/ros2-fundamentals',
            'modules/ros2-nervous-system/ai-ros-bridging',
            'modules/ros2-nervous-system/urdf-humanoids',
          ],
        },
        {
          type: 'category',
          label: 'Digital Twin Simulation with Gazebo and Unity',
          link: {type: 'doc', id: 'modules/digital-twin-simulation/index'},
          items: [
            'modules/digital-twin-simulation/gazebo-physics',
            'modules/digital-twin-simulation/sensor-simulation',
            'modules/digital-twin-simulation/unity-visualization',
          ],
        },
        {
          type: 'category',
          label: 'AI-Robot Brain (NVIDIA Isaac™)',
          link: {type: 'doc', id: 'modules/ai-robot-brain-nvidia-isaac/index'},
          items: [
            'modules/ai-robot-brain-nvidia-isaac/isaac-sim',
            'modules/ai-robot-brain-nvidia-isaac/isaac-ros',
            'modules/ai-robot-brain-nvidia-isaac/nav2-navigation',
          ],
        },
        {
          type: 'category',
          label: 'Vision-Language-Action Systems',
          link: {type: 'doc', id: 'modules/vision-language-action-systems/index'},
          items: [
            'modules/vision-language-action-systems/vision-language-models',
            'modules/vision-language-action-systems/language-guided-action',
            'modules/vision-language-action-systems/multimodal-reasoning',
          ],
        },
        {
          type: 'category',
          label: 'Vision-Language-Action (VLA)',
          link: {type: 'doc', id: 'modules/vla-integration/index'},
          items: [
            'modules/vla-integration/voice-to-action',
            'modules/vla-integration/llm-cognitive-planning',
            'modules/vla-integration/vision-guided-actions',
          ],
        },
      ],
    },
    'modules/ros2-nervous-system/index',
    'modules/ros2-nervous-system/ros2-fundamentals',
    'modules/ros2-nervous-system/ai-ros-bridging',
    'modules/ros2-nervous-system/urdf-humanoids',
    'modules/digital-twin-simulation/index',
    'modules/digital-twin-simulation/gazebo-physics',
    'modules/digital-twin-simulation/sensor-simulation',
    'modules/digital-twin-simulation/unity-visualization',
    'modules/ai-robot-brain-nvidia-isaac/index',
    'modules/ai-robot-brain-nvidia-isaac/isaac-sim',
    'modules/ai-robot-brain-nvidia-isaac/isaac-ros',
    'modules/ai-robot-brain-nvidia-isaac/nav2-navigation',
    'modules/vision-language-action-systems/index',
    'modules/vision-language-action-systems/vision-language-models',
    'modules/vision-language-action-systems/language-guided-action',
    'modules/vision-language-action-systems/multimodal-reasoning',
    'modules/vla-integration/index',
    'modules/vla-integration/voice-to-action',
    'modules/vla-integration/llm-cognitive-planning',
    'modules/vla-integration/vision-guided-actions',
  ],
};

export default sidebars;