# Data Model: Docusaurus Book Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

## Overview
This document defines the data structures and entities for Module 3 of the Docusaurus-based book on Physical AI & Humanoid Robotics. Since this is primarily a documentation project, the "data model" focuses on content organization and metadata structures specific to the NVIDIA Isaac module.

## Content Entities

### Module3
- **name**: String (required) - "Module 3: The AI-Robot Brain (NVIDIA Isaac™)"
- **number**: Integer (required) - 3
- **title**: String (required) - "The AI-Robot Brain (NVIDIA Isaac™)"
- **description**: String (required) - Brief overview of Isaac content
- **topics**: Array of Topic (required) - List of topics covered in Module 3
- **objectives**: Array of String (required) - Learning objectives for Module 3
- **prerequisites**: Array of String (optional) - Knowledge required before starting Module 3
- **word_count**: Integer (optional) - Target word count (1200-1500)
- **references**: Array of Reference (required) - Minimum 5 peer-reviewed sources
- **code_examples**: Array of CodeExample (optional) - Executable code snippets for Module 3
- **simulations**: Array of Simulation (optional) - Reproducible simulation examples for Module 3
- **figures**: Array of Figure (optional) - Diagrams and illustrations for Module 3
- **rag_instructions**: String (optional) - RAG chatbot instructions for Module 3

### Topic
- **title**: String (required) - Topic name
- **description**: String (required) - Brief description of the topic
- **content_path**: String (required) - Path to the markdown file
- **estimated_reading_time**: Integer (required) - In minutes
- **difficulty_level**: Enum (required) - "beginner", "intermediate", "advanced"
- **learning_outcomes**: Array of String (required) - What students will learn

### Reference
- **id**: String (required) - Unique identifier for the reference
- **type**: Enum (required) - "journal", "conference", "book", "web", "documentation"
- **title**: String (required) - Title of the reference
- **authors**: Array of String (required) - List of authors
- **year**: Integer (required) - Publication year
- **journal**: String (optional) - Journal name for journal articles
- **doi**: String (optional) - Digital Object Identifier
- **url**: String (optional) - Web URL
- **apa_citation**: String (required) - Properly formatted APA citation
- **peer_reviewed**: Boolean (required) - Whether it's peer-reviewed

### CodeExample
- **id**: String (required) - Unique identifier
- **title**: String (required) - Descriptive title
- **language**: String (required) - Programming language (python, yaml, etc.)
- **description**: String (required) - Explanation of what the code does
- **source_code**: String (required) - The actual code content
- **file_path**: String (required) - Path to the executable file
- **execution_instructions**: String (required) - How to run the example
- **expected_output**: String (optional) - What the output should look like
- **dependencies**: Array of String (optional) - Required packages/libraries
- **isaac_specific**: Boolean (optional) - Whether this is Isaac-specific code

### Simulation
- **id**: String (required) - Unique identifier
- **title**: String (required) - Simulation title
- **platform**: Enum (required) - "isaac_sim", "isaac_ros", "ros2", "custom"
- **description**: String (required) - What the simulation demonstrates
- **file_path**: String (required) - Path to the simulation file
- **setup_instructions**: String (required) - How to set up the simulation
- **execution_instructions**: String (required) - How to run the simulation
- **expected_behavior**: String (optional) - What should happen during simulation
- **hardware_requirements**: String (optional) - GPU or other hardware requirements

### Figure
- **id**: String (required) - Unique identifier
- **title**: String (required) - Figure title
- **description**: String (required) - Detailed description
- **file_path**: String (required) - Path to the image file
- **alt_text**: String (required) - Accessibility text
- **caption**: String (required) - Figure caption
- **type**: Enum (required) - "diagram", "screenshot", "graph", "photo", "pipeline"

### User
- **role**: Enum (required) - "student", "researcher", "developer", "educator"
- **background**: String (optional) - Technical background information
- **goals**: Array of String (optional) - What the user wants to achieve

## Validation Rules

### Module3 Validation
- Module number must be 3
- Title must be 5-100 characters
- Description must be 10-500 characters
- Must have 1-10 topics
- Must have 1-10 learning objectives
- Must have 5-20 references with at least 50% peer-reviewed
- Word count must be between 1200-1500 (excluding code and references)

### Reference Validation
- At least 5 references required for Module 3
- At least 50% must be peer-reviewed
- APA citation format must be valid
- Year must be between 2000 and current year
- Must have either DOI or URL for accessibility

### CodeExample Validation
- Must be executable and reproducible
- Language must be supported by syntax highlighter
- File path must exist in examples directory
- Dependencies must be documented
- Isaac-specific examples must include hardware requirements

### Simulation Validation
- Must be reproducible in Isaac environment
- Platform must match available tools (isaac_sim, isaac_ros)
- File path must exist in simulations directory
- Setup instructions must be complete and accurate
- Hardware requirements must be clearly specified

### Content Relationships
- Each Topic belongs to exactly one Module3
- Each Reference can be used by multiple Topics within Module 3
- Each CodeExample belongs to one or more Topics in Module 3
- Each Simulation belongs to one or more Topics in Module 3
- Each Figure belongs to one or more Topics in Module 3

## State Transitions
- Content starts as "draft" during writing
- Transitions to "review" when ready for technical verification
- Transitions to "approved" after successful review
- Transitions to "published" when included in the book

## Metadata Requirements
- Created date (ISO format)
- Last modified date (ISO format)
- Author information
- Review status
- Compliance with constitution principles (accuracy, reproducibility, etc.)
- RAG integration status for Module 3 chatbot functionality
- Isaac-specific requirements and dependencies