---
sidebar_position: 1
---

# ایک صفحہ بنائیں

**اسٹینڈ لون صفحہ** بنانے کے لیے `src/pages` میں **مارک ڈاؤن یا React** فائلیں شامل کریں:

- `src/pages/index.js` → `localhost:3000/`
- `src/pages/foo.md` → `localhost:3000/foo`
- `src/pages/foo/bar.js` → `localhost:3000/foo/bar`

## اپنا پہلا React صفحہ بنائیں

`src/pages/my-react-page.js` پر ایک فائل بنائیں:

```jsx title="src/pages/my-react-page.js"
import React from 'react';
import Layout from '@theme/Layout';

export default function MyReactPage() {
  return (
    <Layout>
      <h1>میرا React صفحہ</h1>
      <p>یہ ایک React صفحہ ہے</p>
    </Layout>
  );
}
```

A new page is now available at [http://localhost:3000/my-react-page](http://localhost:3000/my-react-page).

## اپنا پہلا مارک ڈاؤن صفحہ بنائیں

`src/pages/my-markdown-page.md` پر ایک فائل بنائیں:

```mdx title="src/pages/my-markdown-page.md"
# میرا مارک ڈاؤن صفحہ

یہ ایک مارک ڈاؤن صفحہ ہے
```

A new page is now available at [http://localhost:3000/my-markdown-page](http://localhost:3000/my-markdown-page).
