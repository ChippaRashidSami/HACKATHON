---
sidebar_position: 2
---

# Quick Start Guide

## Overview

The Physical AI & Humanoid Robotics book is organized into four comprehensive modules that build upon each other to provide a complete understanding of humanoid robotics development:

1. **Module 1: The Robotic Nervous System (ROS 2)** - Establishes the communication foundation
2. **Module 2: The Digital Twin (Gazebo & Unity)** - Covers simulation environments
3. **Module 3: The AI-Robot Brain (NVIDIA Isaac™)** - Focuses on AI perception and navigation
4. **Module 4: Vision-Language-Action (VLA) - Capstone** - Integrates all concepts in a complete system

## Getting Started

1. Start with [Module 1](/docs/module-1-ros2/nodes-topics-services) to establish your ROS 2 foundation
2. Progress sequentially through each module to build comprehensive knowledge
3. Each module contains 3 chapters that build on each other
4. Use the sidebar navigation to easily switch between modules and chapters

## Development Setup

To run this documentation locally:

1. Ensure you have Node.js 18 or higher installed
2. Clone the repository
3. Run `npm install` to install dependencies
4. Run `npm run start` to launch the development server
5. Navigate to http://localhost:3000 to view the documentation

## Content Structure

All content follows the pattern:
- `/docs/module-X-[topic]/` - Contains all chapter files for each module
- Each module directory includes:
  - `_category_.json` - Defines module metadata and sidebar configuration
  - 3 chapter files in Markdown format
- Configuration files:
  - `docusaurus.config.js` - Main site configuration
  - `sidebars.js` - Navigation structure

## Contributing

To add new content:
1. Add your Markdown files to the appropriate module directory
2. Update `sidebars.js` to include your new content in the navigation
3. Ensure your content follows the Docusaurus Markdown format