# Data Model: ROS 2 as Robotic Nervous System

## Documentation Entities

### Module
- **Name**: String (required) - The module name (e.g., "ROS 2 as Robotic Nervous System")
- **Description**: String (required) - Brief description of the module
- **Chapters**: Array[Chapter] - List of chapters in the module
- **Navigation**: Object - Navigation configuration for sidebar

### Chapter
- **Title**: String (required) - The chapter title
- **Slug**: String (required) - URL-friendly identifier
- **Content**: String (required) - Markdown content of the chapter
- **Prerequisites**: Array[String] - List of required knowledge
- **Objectives**: Array[String] - Learning objectives
- **Examples**: Array[CodeExample] - List of code examples in the chapter

### CodeExample
- **Language**: String (required) - Programming language (python, xml, etc.)
- **Code**: String (required) - The actual code content
- **Description**: String - Explanation of what the code does
- **Purpose**: String - Why this example is relevant

### NavigationItem
- **Type**: String (required) - Type of navigation item (doc, category, link)
- **Id**: String - Reference to the document
- **Label**: String - Display label in navigation
- **Items**: Array[NavigationItem] - Child navigation items (for categories)

## Relationships
- Module contains multiple Chapters
- Chapter contains multiple CodeExamples
- Module has one Navigation configuration
- Navigation contains multiple NavigationItems

## Validation Rules
- Module name must be unique within the documentation site
- Chapter slug must be URL-friendly (alphanumeric, hyphens only)
- Chapter titles must be unique within a module
- Code examples must be valid for the specified language
- Navigation items must reference existing documents

## State Transitions
- Draft → Review (when content is complete and ready for review)
- Review → Published (when content passes review and is ready for public consumption)
- Published → Archived (when content is outdated or deprecated)