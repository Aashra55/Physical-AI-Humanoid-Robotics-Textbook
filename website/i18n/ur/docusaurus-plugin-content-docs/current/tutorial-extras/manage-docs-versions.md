---
sidebar_position: 1
---

# دستاویزات کے ورژن کا انتظام کریں

ڈوکوسارس آپ کی دستاویزات کے متعدد ورژنز کا انتظام کر سکتا ہے۔

## دستاویزات کا ایک ورژن بنائیں

اپنے پروجیکٹ کا ورژن 1.0 جاری کریں:

```bash
npm run docusaurus docs:version 1.0
```

The `docs` folder is copied into `versioned_docs/version-1.0` and `versions.json` is created.

آپ کی دستاویزات کے اب 2 ورژن ہیں:

- `1.0` `http://localhost:3000/docs/` پر ورژن 1.0 دستاویزات کے لیے
- `current` `http://localhost:3000/docs/next/` پر **آنے والی، غیر جاری کردہ دستاویزات** کے لیے

## ورژن ڈراپ ڈاؤن شامل کریں

To navigate seamlessly across versions, add a version dropdown.

Modify the `docusaurus.config.js` file:

```js title="docusaurus.config.js"
export default {
  themeConfig: {
    navbar: {
      items: [
        // highlight-start
        {
          type: 'docsVersionDropdown',
        },
        // highlight-end
      ],
    },
  },
};
```

The docs version dropdown appears in your navbar:

![Docs Version Dropdown](./img/docsVersionDropdown.png)

## موجودہ ورژن کو اپ ڈیٹ کریں

It is possible to edit versioned docs in their respective folder:

- `versioned_docs/version-1.0/hello.md` `http://localhost:3000/docs/hello` کو اپ ڈیٹ کرتا ہے
- `docs/hello.md` `http://localhost:3000/docs/next/hello` کو اپ ڈیٹ کرتا ہے
