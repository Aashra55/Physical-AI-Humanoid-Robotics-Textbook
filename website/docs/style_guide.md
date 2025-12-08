# Markdown Style Guide for Physical AI & Humanoid Robotics Textbook

This style guide ensures consistency and readability across all book content written in Markdown.

## Headings

- Use ATX headings (prefixed with `#`) for all headings.
- Only one H1 (`#`) per page (the page title).
- Headings should be sequential (e.g., don't skip from H2 to H4).

```markdown
# Page Title

## Section Title

### Sub-section Title

#### Further Sub-section
```

## Paragraphs and Line Breaks

- Use blank lines to separate paragraphs.
- Keep sentences concise.
- Hard wrap lines at approximately 80-100 characters for readability in text editors.

## Emphasis

- Use `*single asterisks*` for *italics*.
- Use `**double asterisks**` for **bold**.
- Use `***triple asterisks***` for ***bold italics***.

## Lists

- Use hyphens (`-`) for unordered lists.
- Use `1.` for ordered lists. The actual number doesn't matter; Markdown will render it correctly.
- Indent nested lists with two spaces.

```markdown
- Item 1
  - Nested item 1.1
    - Nested item 1.1.1
- Item 2

1. Ordered item 1
  1. Nested ordered item 1.1
2. Ordered item 2
```

## Code

- Use backticks (`` `code` ``) for `inline code`.
- Use fenced code blocks with language specifiers for blocks of code.
- Always specify the language for syntax highlighting.

```python
```python
def hello_world():
    print("Hello, Physical AI!")
```

## Links

- Use inline links whenever possible.
- Provide descriptive link text.

```markdown
[Docusaurus website](https://docusaurus.io/)
```

## Images

- Store images in a `static/img` folder within the Docusaurus project.
- Use descriptive alt text.

```markdown
![Alt text for image](/img/my-diagram.png)
```

## Tables

- Use a consistent number of hyphens for separators.
- Align columns using colons.

```markdown
| Header 1 | Header 2 | Header 3 |
| :------- | :------- | :------- |
| Content  | Content  | Content  |
```

## Blockquotes

- Use `>` for blockquotes.

```markdown
> "The future of AI is physical." - Unattributed
```

## Admonitions (Callouts)

- Use Docusaurus admonition syntax for notes, warnings, etc.

```markdown
:::note
This is a note.
:::

:::warning
This is a warning.
:::
```

## Comments

- Use HTML comments for comments that should not be rendered.

```html
<!-- This is a comment -->
```
