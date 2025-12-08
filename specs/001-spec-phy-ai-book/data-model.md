# Data Model: Physical AI & Humanoid Robotics Book

This document outlines the high-level data structures for the content and projects in the textbook. As this is not a traditional software application with a database, the "data model" refers to the conceptual organization of the educational materials.

## 1. Core Concepts

The entire project is organized around a few core concepts.

### 1.1. Textbook
The top-level container for all content. It is built using Docusaurus.

-   **Attributes**:
    -   `title`: The title of the book.
    -   `modules`: A collection of `Module` objects.

### 1.2. Module
A `Module` represents a major section of the book, focusing on a specific area of robotics.

-   **Attributes**:
    -   `id`: A unique identifier (e.g., `module1-ros`).
    -   `title`: The title of the module (e.g., "The Robotic Nervous System (ROS 2)").
    -   `summary`: A brief description of the module's content.
    -   `chapters`: An ordered list of `Chapter` objects.
    -   `project`: A corresponding `Project` that contains the hands-on code for the module.

### 1.3. Chapter
A `Chapter` is a specific lesson within a `Module`.

-   **Attributes**:
    -   `id`: A unique identifier (e.g., `ros-nodes-and-topics`).
    -   `title`: The title of the chapter.
    -   `content`: The educational text, diagrams, and code snippets, written in Markdown.
    -   `exercises`: A set of small, focused tasks for the reader to complete.

### 1.4. Project
A `Project` is a self-contained software project, typically a ROS 2 workspace, that accompanies a `Module`.

-   **Attributes**:
    -   `id`: A unique identifier (e.g., `simple_ros2_pkg`).
    -   `path`: The location of the project files within the `projects/` directory.
    -   `description`: A summary of the project's goals.
    -   `dependencies`: A list of software dependencies (e.g., ROS 2 packages, Python libraries).

## 2. Relationships

The relationships between these concepts are hierarchical:

```
Textbook (1) --has--> (Many) Modules
Module (1)   --has--> (Many) Chapters
Module (1)   --has--> (1) Project
```

## 3. Example Instantiation

Here is an example of how these concepts map to the actual content:

-   **Textbook**: "Physical AI & Humanoid Robotics"
    -   **Module**: `module1-ros` ("The Robotic Nervous System (ROS 2)")
        -   **Chapters**:
            -   "Introduction to ROS 2"
            -   "ROS 2 Nodes and Topics"
            -   ...
        -   **Project**: `simple_ros2_pkg`
            -   Located at `projects/module1/simple_ros2_pkg/`
            -   Contains the ROS 2 nodes and launch files for the module's exercises.
