---
sidebar_position: 5
---

# اپنی سائٹ کو تعینات کریں

ڈوکوسارس ایک **اسٹیٹک-سائٹ-جنریٹر** ہے (جسے **[Jamstack](https://jamstack.org/)** بھی کہا جاتا ہے)۔

یہ آپ کی سائٹ کو سادہ **اسٹیٹک HTML، جاوا اسکرپٹ اور CSS فائلوں** کے طور پر بناتا ہے۔

## اپنی سائٹ بنائیں

**پروڈکشن کے لیے** اپنی سائٹ بنائیں:

```bash
npm run build
```

The static files are generated in the `build` folder.

## اپنی سائٹ تعینات کریں

Test your production build locally:

```bash
npm run serve
```

The `build` folder is now served at [http://localhost:3000/](http://localhost:3000/).

آپ اب `build` فولڈر کو **تقریباً کہیں بھی** آسانی سے، **مفت میں** یا بہت کم لاگت پر تعینات کر سکتے ہیں ([تعیناتی گائیڈ](https://docusaurus.io/docs/deployment) پڑھیں)۔
