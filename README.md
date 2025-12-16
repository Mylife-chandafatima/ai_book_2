# Physical AI & Humanoid Robotics Book

This is a Docusaurus-based website for the Physical AI & Humanoid Robotics book. The site organizes the book content into modules with proper navigation, code highlighting, and diagram support.

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Building the Site

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

### GitHub Pages

1. Update the `url` and `baseUrl` in `docusaurus.config.js`:
   ```js
   url: 'https://your-username.github.io',
   baseUrl: '/your-repo-name/',
   organizationName: 'your-username',
   projectName: 'your-repo-name',
   ```

2. Run the deployment command:
   ```bash
   npm run deploy
   ```

This will build your site and push the generated static files to the `gh-pages` branch.

### Vercel

1. Push your code to a Git repository
2. Go to [Vercel](https://vercel.com) and import your repository
3. Set the build command to `npm run build`
4. Set the output directory to `build`
5. Deploy!

### Netlify

1. Push your code to a Git repository
2. Go to [Netlify](https://netlify.com) and import your repository
3. Set the build command to `npm run build`
4. Set the publish directory to `build`
5. Deploy!

## Project Structure

```
book/
├── docs/                 # Book content organized by modules
│   └── modules/          # Module-specific content
│       ├── module-1/
│       ├── module-2/
│       └── ...
├── src/
│   ├── components/       # Custom React components (diagrams, chatbot, etc.)
│   │   └── ChatbotWidget/ # RAG chatbot component
│   ├── theme/            # Docusaurus theme overrides
│   │   └── Layout.js     # Integrates chatbot into all pages
│   └── css/              # Custom styles
├── backend/              # FastAPI backend for RAG services
│   ├── src/
│   │   ├── models/       # Data models
│   │   ├── services/     # RAG services (ingestion, retrieval, generation)
│   │   ├── api/          # API endpoints
│   │   ├── config/       # Configuration
│   │   └── utils/        # Utilities
│   └── requirements.txt  # Python dependencies
├── static/               # Static assets (images, diagrams)
├── docusaurus.config.js  # Main Docusaurus configuration
├── sidebars.js           # Navigation sidebar configuration
└── package.json          # Dependencies and scripts
```

## Integrated RAG Chatbot

This book features an integrated Retrieval-Augmented Generation (RAG) chatbot that allows readers to ask questions about the book content and receive AI-powered answers with proper citations.

### Features

- **Book Mode**: Search the entire book corpus for answers
- **Selection Mode**: Ask questions about specific text you've selected on the page
- **Citation Support**: All answers include proper citations to book modules and chapters
- **Context-Aware**: The system understands the book's structure and content

### Running the Chatbot

1. **Install Python dependencies:**
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. **Set up environment variables:**
   ```bash
   # In the backend/ directory, copy and configure the example:
   cp .env.example .env
   # Edit .env with your Qdrant, Neon, and OpenAI Router credentials
   ```

3. **Run both frontend and backend:**
   ```bash
   npm run dev
   ```

4. **Access the chatbot:**
   - Navigate to `http://localhost:3000`
   - Use the chatbot widget in the bottom-right corner
   - Toggle between Book Mode and Selection Mode as needed

## Adding New Content

To add new modules or chapters:

1. Create a new markdown file in the appropriate module directory (e.g., `docs/modules/module-1/new-chapter.md`)
2. Add the new file to the `sidebars.js` configuration
3. Use frontmatter to control sidebar positioning:

```md
---
sidebar_position: 2
title: 'New Chapter Title'
---
```

## Code Syntax Highlighting

The site supports syntax highlighting for Python, Bash, YAML, and C++ (for ROS2 code). Use standard markdown code blocks with language identifiers:

```python
# Python code for ROS2
import rclpy
from rclpy.node import Node
```

## MDX Diagrams

Custom diagrams are implemented as React components in `src/components/RobotDiagrams.js`. To use them in your markdown files:

```md
import { RobotSensorDiagram } from '../src/components/RobotDiagrams';

<RobotSensorDiagram />
```

## APA-Style Citations

For APA-style citations, use standard markdown lists with proper formatting. The CSS will style them appropriately.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add/update documentation as needed
5. Submit a pull request

## Support

For support, please open an issue in the repository or contact the maintainers.