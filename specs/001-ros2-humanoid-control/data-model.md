# Data Model: Docusaurus Book for Modules 1-4 Physical AI & Humanoid Robotics

## Overview
This document defines the data structures and entities for the first four modules of the Docusaurus-based book on Physical AI & Humanoid Robotics. Since this is primarily a documentation project, the "data model" focuses on content organization and metadata structures for the foundational modules.

## Content Entities

### Module
- **name**: String (required) - The module title (e.g., "Module 1: The Robotic Nervous System")
- **number**: Integer (required) - Sequential module number (1-4 for this scope)
- **title**: String (required) - Descriptive title of the module
- **description**: String (required) - Brief overview of module content
- **topics**: Array of Topic (required) - List of topics covered in the module
- **objectives**: Array of String (required) - Learning objectives for the module
- **prerequisites**: Array of String (optional) - Knowledge required before starting
- **word_count**: Integer (optional) - Target word count (1200-1500 per module)
- **references**: Array of Reference (required) - Minimum 5 peer-reviewed sources
- **code_examples**: Array of CodeExample (optional) - Executable code snippets
- **simulations**: Array of Simulation (optional) - Reproducible simulation examples
- **figures**: Array of Figure (optional) - Diagrams and illustrations
- **rag_instructions**: String (optional) - RAG chatbot instructions for this module

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
- **language**: String (required) - Programming language (python, c++, etc.)
- **description**: String (required) - Explanation of what the code does
- **source_code**: String (required) - The actual code content
- **file_path**: String (required) - Path to the executable file
- **execution_instructions**: String (required) - How to run the example
- **expected_output**: String (optional) - What the output should look like
- **dependencies**: Array of String (optional) - Required packages/libraries

### Simulation
- **id**: String (required) - Unique identifier
- **title**: String (required) - Simulation title
- **platform**: Enum (required) - "gazebo", "unity", "isaac", "ros2", "custom"
- **description**: String (required) - What the simulation demonstrates
- **file_path**: String (required) - Path to the simulation file
- **setup_instructions**: String (required) - How to set up the simulation
- **execution_instructions**: String (required) - How to run the simulation
- **expected_behavior**: String (optional) - What should happen during simulation

### Figure
- **id**: String (required) - Unique identifier
- **title**: String (required) - Figure title
- **description**: String (required) - Detailed description
- **file_path**: String (required) - Path to the image file
- **alt_text**: String (required) - Accessibility text
- **caption**: String (required) - Figure caption
- **type**: Enum (required) - "diagram", "screenshot", "graph", "photo"

### User
- **role**: Enum (required) - "student", "researcher", "developer", "educator"
- **background**: String (optional) - Technical background information
- **goals**: Array of String (optional) - What the user wants to achieve

## Validation Rules

### Module Validation
- Module number must be between 1 and 4 (for this scope)
- Title must be 5-100 characters
- Description must be 10-500 characters
- Must have 1-10 topics
- Must have 1-10 learning objectives
- Must have 5-20 references with at least 50% peer-reviewed
- Word count must be between 1200-1500 (excluding code and references)

### Reference Validation
- At least 5 references required per module
- At least 50% must be peer-reviewed
- APA citation format must be valid
- Year must be between 2000 and current year
- Must have either DOI or URL for accessibility

### CodeExample Validation
- Must be executable and reproducible
- Language must be supported by syntax highlighter
- File path must exist in examples directory
- Dependencies must be documented

### Simulation Validation
- Must be reproducible in target environment
- Platform must match available tools (gazebo, unity, isaac, etc.)
- File path must exist in simulations directory
- Setup instructions must be complete and accurate

### Content Relationships
- Each Topic belongs to exactly one Module
- Each Reference can be used by multiple Topics within a Module
- Each CodeExample belongs to one or more Topics
- Each Simulation belongs to one or more Topics
- Each Figure belongs to one or more Topics

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
- RAG integration status for chatbot functionality