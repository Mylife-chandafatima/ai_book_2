# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

## Overview
This guide provides a quick setup and development workflow for Module 2 of the Docusaurus-based book on Physical AI & Humanoid Robotics. Module 2 covers Digital Twin concepts using Gazebo and Unity for physics simulation and high-fidelity rendering. Follow these steps to get started with contributing to Module 2 content.

## Prerequisites for Module 2
- Node.js (v18 or higher)
- Python 3.11
- Git
- Basic knowledge of Markdown and Docusaurus
- Gazebo simulation environment (Gazebo 11+)
- (Optional) Unity development environment (Unity 2022.3 LTS)
- ROS 2 Humble Hawksbill environment

## Setup Instructions for Module 2

### 1. Clone the Repository
```bash
git clone <repository-url>
cd book
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Install Python Dependencies
```bash
pip install -r requirements.txt
```

### 4. Start Local Development Server
```bash
npm start
```
This command starts a local development server and opens the documentation in your browser. Most changes are reflected live without restarting the server.

## Module 2 Structure
Module 2 is organized around Digital Twin concepts with specific focus on Gazebo and Unity:

```
book/
├── docs/
│   └── module-2-digital-twin/
│       ├── index.md              # Module 2 overview
│       ├── gazebo-simulation.md  # Gazebo fundamentals and physics
│       ├── unity-rendering.md    # Unity integration and rendering
│       └── sensor-simulation.md  # Sensor simulation in both platforms
├── examples/
│   ├── gazebo-examples/
│   │   ├── simple_world.world    # Basic Gazebo world file
│   │   ├── robot_model.urdf      # Robot model for simulation
│   │   └── physics_demo.py       # Physics demonstration script
│   └── unity-examples/
│       └── humanoid_scene.unity  # Unity scene for rendering
└── tests/
    ├── module2-build-test.js     # Module 2 build validation
    ├── gazebo-simulation-test.py # Gazebo simulation tests
    └── citation-validator.py     # Citation format validation
```

## Adding Content to Module 2

### 1. Create Topic Pages
Add new markdown files to the Module 2 directory:
```bash
touch docs/module-2-digital-twin/new-topic.md
```

### 2. Follow Content Standards for Module 2
Each Module 2 content page should include:

```markdown
---
title: [Module 2 Topic Title]
sidebar_position: [position_number]
description: [Brief description of the Module 2 topic]
---

# [Module 2 Topic Title]

## Overview
[Brief overview of the Digital Twin topic]

## Key Concepts
- Concept 1
- Concept 2
- Concept 3

## Implementation
[Detailed explanation with code examples specific to Gazebo/Unity]

## Simulation Steps
[Detailed steps for running Gazebo/Unity simulations]

## References
[APA-formatted citations for Module 2 content]
```

## Adding Gazebo Examples for Module 2

### 1. Create Gazebo World File
Add your Gazebo simulation files to the appropriate directory:
```bash
touch examples/gazebo-examples/my_world.world
```

### 2. Reference in Documentation
Use Docusaurus code blocks with appropriate language highlighting:
```markdown
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_world">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

### 3. Include Execution Instructions
Document how to run the Gazebo example:
```markdown
## Running the Gazebo Simulation

1. Make sure Gazebo is installed and accessible:
   ```bash
   gazebo --version
   ```

2. Launch the simulation:
   ```bash
   gazebo examples/gazebo-examples/my_world.world
   ```

3. The simulation should display physics with gravity properly configured.
```

## Adding Unity Examples for Module 2

### 1. Create Unity Scene File
Add your Unity assets to the appropriate directory:
```bash
touch examples/unity-examples/my_scene.unity
```

### 2. Document Unity Setup
Include detailed setup and execution instructions:
```markdown
## Setting up the Unity Scene

1. Open Unity Hub and create a new 3D project
2. Import the humanoid robot model
3. Configure lighting and rendering settings
4. The scene should render with high-fidelity graphics
```

## Quality Standards for Module 2

### Content Requirements
- Follow APA citation format for all references
- Include at least 5 peer-reviewed sources for Module 2
- Maintain 1200-1500 words for Module 2 (excluding code and references)
- Ensure all Gazebo/Unity examples are executable and reproducible
- Include simulation validation steps where applicable
- Target Flesch-Kincaid grade level 10-12

### Technical Verification for Module 2
- All Gazebo examples must run successfully in the target environment
- All Unity examples must execute with expected behavior
- Links must be validated and functional
- Images must have appropriate alt text for accessibility
- Cross-references within Module 2 must be accurate

## Building Module 2 Documentation

### Local Build for Module 2
```bash
npm run build
```

### Serve Module 2 Documentation
```bash
npm run serve
```

## Testing Strategy for Module 2

### Module 2 Build Verification
```bash
npm run build
```
This ensures Module 2 documentation builds without errors.

### Gazebo Simulation Testing
```bash
python tests/gazebo-simulation-test.py
```
This validates that Gazebo simulation examples run as expected.

### Citation Verification for Module 2
```bash
python tests/citation-validator.py
```
This checks that all Module 2 citations follow APA format.

### Link Validation for Module 2
```bash
npm run lint:links
```
This checks for broken internal and external links in Module 2.

## RAG Chatbot Integration for Module 2

Module 2 includes specific instructions for the RAG chatbot:

1. **Content Indexing**: Module 2 content is indexed for retrieval
2. **Context Management**: Chatbot maintains context within Module 2 boundaries
3. **Citation Tracking**: Chatbot provides source citations for Module 2 responses
4. **Cross-Module Queries**: Centralized index enables queries across modules

### Module 2 RAG Instructions
- Focus on simulation environments, sensor modeling, and physics validation
- Include Gazebo and Unity specific knowledge
- Reference ROS integration for sensor data
- Include best practices for Digital Twin implementation

## Publishing Workflow for Module 2

### 1. Review Process
- Technical accuracy verification for Gazebo/Unity content
- Content completeness check for Module 2
- Citation format validation for Module 2
- Simulation example testing for Module 2

### 2. Staging
- Deploy to staging environment
- Manual testing of Module 2 navigation and examples
- Cross-module link verification

### 3. Production Deployment
- Automated deployment to GitHub Pages
- Search index update
- Chatbot knowledge base refresh for Module 2

## Troubleshooting Module 2

### Common Issues

**Gazebo Simulation Errors**: Check that Gazebo is properly installed and the world files are correctly formatted.

**Unity Scene Failures**: Ensure Unity environment is properly configured and assets are correctly imported.

**Missing Navigation**: Confirm sidebar configuration includes Module 2 and file paths are correct.

## Next Steps for Module 2
1. Review the existing Module 2 content to understand the structure
2. Choose a topic within Module 2 to contribute to
3. Follow the content standards and quality requirements
4. Test your Gazebo/Unity examples locally before submitting
5. Submit a pull request for review