# اپنی سائٹ کا ترجمہ کریں

آئیے `docs/intro.md` کا اردو میں ترجمہ کریں۔

## i18n کو کنفیگر کریں

`ur` لوکیل کے لیے سپورٹ شامل کرنے کے لیے `docusaurus.config.js` میں ترمیم کریں:

```js title="docusaurus.config.js"
export default {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },
};
```

## ایک دستاویز کا ترجمہ کریں

`docs/intro.md` فائل کو `i18n/ur` فولڈر میں کاپی کریں:

```bash
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current/

cp docs/intro.md i18n/ur/docusaurus-plugin-content-docs/current/intro.md
```

`i18n/ur/docusaurus-plugin-content-docs/current/intro.md` کا اردو میں ترجمہ کریں۔

## اپنی لوکلائزڈ سائٹ شروع کریں

اردو لوکیل پر اپنی سائٹ شروع کریں:

```bash
npm run start -- --locale ur
```

آپ کی لوکلائزڈ سائٹ [http://localhost:3000/ur/](http://localhost:3000/ur/) پر قابل رسائی ہے اور `شروع کرنا` صفحہ کا ترجمہ ہو چکا ہے۔

:::caution

ترقی میں، آپ ایک وقت میں صرف ایک لوکیل استعمال کر سکتے ہیں۔

:::

## ایک لوکیل ڈراپ ڈاؤن شامل کریں

To navigate seamlessly across languages, add a locale dropdown.

Modify the `docusaurus.config.js` file:

```js title="docusaurus.config.js"
export default {
  themeConfig: {
    navbar: {
      items: [
        // highlight-start
        {
          type: 'localeDropdown',
        },
        // highlight-end
      ],
    },
  },
};
```

The locale dropdown now appears in your navbar:

![Locale Dropdown](./img/localeDropdown.png)

## اپنی لوکلائزڈ سائٹ بنائیں

Build your site for a specific locale:

```bash
npm run build -- --locale ur
```

Or build your site to include all the locales at once:

```bash
npm run build
```
