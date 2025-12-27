---
sidebar_position: 2
---

# ایک دستاویز بنائیں

دستاویزات **صفحات کے گروپس** ہیں جو ان کے ذریعے منسلک ہیں:

- ایک **سائیڈبار**
- **پچھلی/اگلی نیویگیشن**
- **ورژننگ**

## اپنی پہلی دستاویز بنائیں

`docs/hello.md` پر ایک مارک ڈاؤن فائل بنائیں:

```md title="docs/hello.md"
# ہیلو

یہ میری **پہلی ڈوکوسارس دستاویز** ہے!
```

A new document is now available at [http://localhost:3000/docs/hello](http://localhost:3000/docs/hello).

## سائیڈبار کو کنفیگر کریں

ڈوکوسارس خود بخود `docs` فولڈر سے **ایک سائیڈبار بناتا ہے**۔

Add metadata to customize the sidebar label and position:

```md title="docs/hello.md" {1-4}
---
sidebar_label: 'ہیلو!'
sidebar_position: 3
---

# ہیلو

یہ میری **پہلی ڈوکوسارس دستاویز** ہے!
```

It is also possible to create your sidebar explicitly in `sidebars.js`:

```js title="sidebars.js"
export default {
  tutorialSidebar: [
    'intro',
    // highlight-next-line
    'hello',
    {
      type: 'category',
      label: 'ٹیوٹوریل',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
};
```
