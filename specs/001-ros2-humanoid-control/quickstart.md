# Quickstart Guide: Docusaurus Book for Modules 1-4 Physical AI & Humanoid Robotics

## Overview
This guide provides a quick setup and development workflow for the first four modules of the Docusaurus-based book on Physical AI & Humanoid Robotics. Follow these steps to get started with contributing to Modules 1-4: ROS 2, Digital Twin, NVIDIA Isaac, and Vision-Language-Action.

## Prerequisites
- Node.js (v18 or higher)
- Python 3.11
- Git
- Basic knowledge of Markdown and Docusaurus
- Access to ROS 2 Humble Hawksbill environment (for testing examples)
- Gazebo simulation environment
- (Optional) Unity development environment and NVIDIA Isaac Sim

## Setup Instructions

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

## Book Structure
The book is organized into 4 foundational modules following the modular structure required by the project constitution:

```
book/
├── docs/                 # Documentation files organized by modules
│   ├── intro.md          # Introduction to the book
│   ├── module-1-ros2/    # Module 1: The Robotic Nervous System
│   ├── module-2-digital-twin/  # Module 2: The Digital Twin
│   ├── module-3-nvidia-isaac/  # Module 3: The AI-Robot Brain
│   └── module-4-vla/     # Module 4: Vision-Language-Action
├── examples/            # Executable code examples
│   ├── ros2-examples/
│   ├── gazebo-examples/
│   ├── isaac-examples/
│   └── vla-examples/
├── static/              # Static assets (images, etc.)
├── src/                 # Custom React components
├── docusaurus.config.js # Docusaurus configuration
├── sidebars.js          # Navigation sidebar configuration
└── package.json         # Project dependencies and scripts
```

## Adding Content to a Module

### 1. Create Topic Pages
Add new markdown files to your module directory:
```bash
touch docs/module-1-ros2/topic-name.md
```

### 2. Follow Content Standards
Each content page should include:

```markdown
---
title: [Topic Title]
sidebar_position: [position_number]
description: [Brief description of the topic]
---

# [Topic Title]

## Overview
[Brief overview of the topic]

## Key Concepts
- Concept 1
- Concept 2
- Concept 3

## Implementation
[Detailed explanation with code examples]

## Simulation Steps
[If applicable, detailed steps for running simulations]

## References
[APA-formatted citations]
```

## Adding Code Examples

### 1. Create Code File
Add your executable code to the appropriate examples directory:
```bash
touch examples/ros2-examples/example-name.py
```

### 2. Reference in Documentation
Use Docusaurus code blocks with appropriate language highlighting:
```markdown
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

### 3. Include Execution Instructions
Document how to run the example:
```markdown
## Running the Example

1. Make sure ROS 2 is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Run the example:
   ```bash
   python3 examples/ros2-examples/simple_publisher.py
   ```

3. You should see output indicating the node is publishing messages.
```

## Adding Simulation Examples

### 1. Create Simulation File
Add your simulation files to the appropriate directory:
```bash
touch examples/gazebo-examples/simple_world.world
```

### 2. Document Simulation Setup
Include detailed setup and execution instructions:
```markdown
## Setting up the Simulation

1. Ensure Gazebo is installed and accessible:
   ```bash
   gazebo --version
   ```

2. Launch the simulation:
   ```bash
   gazebo examples/gazebo-examples/simple_world.world
   ```

3. The robot model should appear in the environment with physics properly configured.
```

## Quality Standards

### Content Requirements
- Follow APA citation format for all references
- Include at least 5 peer-reviewed sources per module
- Maintain 1200-1500 words per module (excluding code and references)
- Ensure all code examples are executable and reproducible
- Include simulation validation steps where applicable
- Target Flesch-Kincaid grade level 10-12

### Technical Verification
- All code examples must run successfully in the target environment
- All simulations must execute with expected behavior
- Links must be validated and functional
- Images must have appropriate alt text for accessibility
- Cross-references between modules must be accurate

## Building the Documentation

### Local Build
```bash
npm run build
```

### Serve Built Documentation
```bash
npm run serve
```

## Testing Strategy

### Build Verification
```bash
npm run build
```
This ensures the documentation builds without errors.

### Code Example Testing
```bash
python tests/code_snippet_tests.py
```
This runs all code examples to verify they execute correctly.

### Simulation Testing
```bash
python tests/simulation_tests.py
```
This validates that simulation examples run as expected.

### Citation Verification
```bash
python tests/citation_validator.py
```
This checks that all citations follow APA format.

### Link Validation
```bash
npm run lint:links
```
This checks for broken internal and external links.

## RAG Chatbot Integration

Each module includes specific instructions for the RAG chatbot:

1. **Content Indexing**: All module content is indexed for retrieval
2. **Context Management**: Chatbot maintains context within module boundaries
3. **Citation Tracking**: Chatbot provides source citations for all responses
4. **Cross-Module Queries**: Centralized index enables queries across the first four modules

### Module-Specific RAG Instructions
- **Module 1**: Focus on ROS 2 fundamentals, node communication, and rclpy usage
- **Module 2**: Focus on simulation environments, sensor modeling, and physics validation
- **Module 3**: Focus on perception systems, navigation algorithms, and Isaac tools
- **Module 4**: Focus on LLM integration, voice processing, and cognitive planning

## Publishing Workflow

### 1. Review Process
- Technical accuracy verification
- Content completeness check
- Citation format validation
- Code and simulation example testing

### 2. Staging
- Deploy to staging environment
- Manual testing of navigation and examples
- Cross-module link verification

### 3. Production Deployment
- Automated deployment to GitHub Pages
- Search index update
- Chatbot knowledge base refresh

## Troubleshooting

### Common Issues

**Build Errors**: Check for syntax errors in markdown files and ensure all referenced files exist.

**Code Examples Not Working**: Verify the target environment matches the documentation and all dependencies are installed.

**Simulation Failures**: Ensure the simulation environment (Gazebo, Unity, Isaac) is properly configured.

**Missing Navigation**: Confirm sidebar configuration includes the new content and file paths are correct.

## Next Steps
1. Review the existing modules to understand the content structure
2. Choose a module to contribute to among the first four
3. Follow the content standards and quality requirements
4. Test your content locally before submitting
5. Submit a pull request for review