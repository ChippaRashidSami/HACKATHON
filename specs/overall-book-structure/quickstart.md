# Quickstart Guide: Physical AI & Humanoid Robotics Book

**Feature**: Complete Book Structure
**Version**: 1.0
**Date**: 2025-12-16

## Overview
This guide provides a quick introduction to the Physical AI & Humanoid Robotics book, a comprehensive course on AI-powered humanoid robotics. The book is organized into four modules that progress from foundational concepts to advanced AI integration:

1. **Module 1**: The Robotic Nervous System (ROS 2)
2. **Module 2**: The Digital Twin (Gazebo & Unity)
3. **Module 3**: The AI-Robot Brain (NVIDIA Isaac™)
4. **Module 4**: Vision-Language-Action (VLA) - Capstone

## Prerequisites
- Understanding of basic programming concepts
- Familiarity with robotics terminology
- Node.js version 18.0 or higher
- npm or yarn package manager

## Installation and Setup

### Step 1: Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-humanoid-book.git
cd physical-ai-humanoid-book
```

### Step 2: Install Dependencies
```bash
npm install
# or
yarn install
```

### Step 3: Start Development Server
```bash
npm run start
# or
yarn start
```

This will start the Docusaurus development server at `http://localhost:3000`.

## Book Structure

The book is organized in the `docs/` directory with the following structure:

```
docs/
├── module-1-ros2/
│   ├── _category_.json
│   ├── nodes-topics-services.md
│   ├── python-agents-ros-control.md
│   └── humanoid-modeling-urdf.md
├── module-2-digital-twin/
│   ├── _category_.json
│   ├── physics-simulation-gazebo.md
│   ├── high-fidelity-unity.md
│   └── simulated-sensors.md
├── module-3-isaac/
│   ├── _category_.json
│   ├── isaac-sim-synthetic-data.md
│   ├── isaac-ros-perception-vslam.md
│   └── nav2-path-planning.md
└── module-4-vla/
    ├── _category_.json
    ├── speech-recognition.md
    ├── cognitive-planning.md
    └── autonomous-humanoid.md
```

## Module Overview

### Module 1: The Robotic Nervous System (ROS 2)
- **Focus**: ROS 2 as middleware for humanoid robot communication and control
- **Chapters**:
  1. ROS 2 Foundations: Nodes, Topics, and Services
  2. Python Agents and ROS Control with rclpy
  3. Humanoid Modeling with URDF

### Module 2: The Digital Twin (Gazebo & Unity)
- **Focus**: Physics-based simulation and virtual environments for humanoid robots
- **Chapters**:
  1. Physics Simulation with Gazebo
  2. High-Fidelity Interaction in Unity
  3. Simulated Sensors: LiDAR, Depth Cameras, and IMUs

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- **Focus**: High-performance AI perception, navigation, and training for humanoids
- **Chapters**:
  1. NVIDIA Isaac Sim and Synthetic Data Generation
  2. Isaac ROS: Accelerated Perception and VSLAM
  3. Nav2 for Humanoid Path Planning

### Module 4: Vision-Language-Action (VLA) - Capstone
- **Focus**: Connecting perception, language, and action in humanoid robots
- **Chapters**:
  1. Voice-to-Action with Speech Recognition
  2. LLM-Based Cognitive Planning for Robots
  3. Capstone: The Autonomous Humanoid

## Learning Progression

The modules are designed to build upon each other:

1. Start with Module 1 to understand foundational ROS 2 concepts
2. Proceed to Module 2 to learn simulation techniques
3. Advance to Module 3 for AI perception and navigation
4. Complete with Module 4, the capstone that integrates all concepts

## Building for Production

```bash
npm run build
```

This creates a static site in the `build/` directory ready for deployment.

## Deployment to GitHub Pages

The site is configured for GitHub Pages deployment. The build is typically handled by a GitHub Actions workflow, but you can also deploy manually:

```bash
npm run deploy
```

## Contributing to Content

1. Edit the markdown files in the respective module directories
2. Update `_category_.json` to modify sidebar settings if needed
3. Run `npm run start` to preview changes
4. Build the site with `npm run build` when ready for deployment

## Troubleshooting

**Issue**: Docusaurus fails to start
**Solution**: Ensure all dependencies are installed with `npm install`

**Issue**: New content doesn't appear
**Solution**: Restart the development server with `npm run start`

**Issue**: Links between modules are broken
**Solution**: Verify the sidebar configuration in `sidebars.js`