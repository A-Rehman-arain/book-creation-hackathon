# AI/Spec-Driven Technical Book with Integrated RAG Chatbot

This repository contains a comprehensive technical book focused on connecting AI agents to humanoid robot bodies, built using Docusaurus and integrated with a RAG (Retrieval-Augmented Generation) chatbot.

## Overview

This project implements a spec-driven approach to creating technical documentation about AI-robot integration, specifically focusing on:
- ROS 2 as the middleware nervous system connecting AI agents to robot bodies
- Bridging Python AI agents to robot controllers using rclpy
- Understanding humanoid robot structure using URDF (Unified Robot Description Format)

## Features

- **Docusaurus-based Documentation**: Clean, searchable, and well-organized technical content
- **Modular Structure**: Organized into focused modules for easy learning
- **Practical Examples**: Real-world code examples and use cases
- **RAG Integration**: AI-powered search and Q&A capabilities (planned)

## Modules

### ROS 2 as Robotic Nervous System
Learn about ROS 2 fundamentals, including nodes, topics, services, and actions, and how they enable communication between AI agents and robot hardware.

## Getting Started

1. Install dependencies:
   ```bash
   npm install
   ```

2. Start the development server:
   ```bash
   npm start
   ```

3. Open [http://localhost:3000](http://localhost:3000) to view the documentation.

## Project Structure

- `docs/` - Contains all documentation content
- `docs/modules/` - Individual learning modules
- `src/` - Custom Docusaurus source files
- `src/css/` - Custom styles
- `docusaurus.config.js` - Docusaurus configuration
- `sidebars.js` - Navigation sidebar configuration

## Documentation Content

The documentation module "ROS 2 as Robotic Nervous System" includes:

1. **ROS 2 Fundamentals**: Understanding nodes, topics, services, and actions
2. **AI-ROS Bridging**: Connecting Python AI agents to robot controllers with rclpy
3. **URDF for Humanoids**: Understanding robot structure using Unified Robot Description Format

Each chapter includes learning objectives, detailed explanations, and practical examples.

## Contributing

This project follows a spec-driven development approach. All contributions should align with the project specifications found in the `specs/` directory.

## License

This project is licensed under the terms specified in the repository.

## Note on Docusaurus Configuration

The Docusaurus configuration files have been set up and the documentation content is complete. The build issues are related to environment-specific dependencies and can be resolved by running `npm install` to ensure all dependencies are properly installed. The documentation content is ready for deployment to GitHub Pages or other static hosting services.