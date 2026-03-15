# یونٹی کے ساتھ ایڈوانسڈ ویژولائزیشن

جبکہ گزیبو طبیعیات پر مبنی سیمولیشن میں بہترین ہے، یونٹی (ایک مقبول ریئل ٹائم 3D ترقیاتی پلیٹ فارم) جدید ویژولائزیشن، انسان-روبوٹ تعامل (HRI)، اور بھرپور، انٹرایکٹو ورچوئل ماحول بنانے کے لیے بے مثال صلاحیتیں پیش کرتا ہے۔ یہ باب یہ بتاتا ہے کہ یونٹی کو روبوٹکس کی ترقی کو بڑھانے کے لیے کیسے استعمال کیا جا سکتا ہے، یا تو ایک اسٹینڈ لون سیمولیشن ماحول کے طور پر یا ROS 2 کے ساتھ مل کر۔

## روبوٹکس ویژولائزیشن کے لیے یونٹی کیوں؟

یونٹی روبوٹکس ویژولائزیشن اور تعامل کے لیے کئی فوائد فراہم کرتا ہے:
-   **اعلیٰ مخلص رینڈرنگ**: فوٹو ریئلسٹک ماحول اور روبوٹ ماڈلز بنائیں۔
-   **بھرپور یوزر انٹرفیس**: یونٹی کے UI ٹول کٹ کے ساتھ پیچیدہ HRI انٹرفیس تیار کریں۔
-   **کراس پلیٹ فارم تعیناتی**: ڈیسک ٹاپ، ویب، یا یہاں تک کہ VR/AR کے لیے ویژولائزیشن بنائیں۔
-   **توسیع پذیری**: وسیع ایسٹ اسٹور اور C# اسکرپٹنگ کسٹم رویوں اور انضمام کی اجازت دیتی ہے۔
-   **انسان-روبوٹ تعامل (HRI)**: سیمولیشن میں انسانوں کے لیے روبوٹس کے ساتھ تعامل اور کنٹرول کرنے کے بدیہی طریقے ڈیزائن کریں۔

## یونٹی روبوٹکس ہب

یونٹی روبوٹکس ہب اوپن سورس ٹولز اور پیکجز کا ایک مجموعہ ہے جو یونٹی کے اندر روبوٹکس کی ترقی کو آسان بناتا ہے۔ اہم پیکجز میں شامل ہیں:
-   **ROS-TCP-Connector**: TCP کا استعمال کرتے ہوئے یونٹی اور ROS 2 کے درمیان براہ راست مواصلات کو فعال کرتا ہے۔
-   **URDF امپورٹر**: URDF ماڈلز کو براہ راست یونٹی میں درآمد کرتا ہے، انہیں یونٹی گیم آبجیکٹس میں تبدیل کرتا ہے۔
-   **Robotics-Visualizations**: یونٹی میں ROS 2 پیغامات (مثال کے طور پر، سینسر ڈیٹا، TF فریمز) کو دیکھنے کے لیے ٹولز فراہم کرتا ہے۔

## یونٹی کے ساتھ ROS 2 کو مربوط کرنا

`ROS-TCP-Connector` یونٹی کو آپ کے ROS 2 ایکو سسٹم کے ساتھ جوڑنے کے لیے مرکزی حیثیت رکھتا ہے۔ یہ یونٹی کو ROS 2 نوڈ کے طور پر کام کرنے کی اجازت دیتا ہے، سینسر ڈیٹا شائع کرتا ہے، کنٹرول کمانڈز کو سبسکرائب کرتا ہے، اور ROS 2 کے طاقتور مواصلاتی میکانزم کا فائدہ اٹھاتا ہے۔

### تصوراتی ورک فلو:

1.  **URDF درآمد کریں**: یونٹی میں اپنے روبوٹ ماڈل کو لانے کے لیے URDF امپورٹر کا استعمال کریں۔
2.  **سینسرز شامل کریں**: یونٹی میں اپنے روبوٹ ماڈل سے ورچوئل سینسرز (کیمرے، LIDARs) منسلک کریں۔
3.  **ROS-TCP-Connector کنفیگر کریں**: یونٹی کے لیے ROS 2 نیٹ ورک سے منسلک ہونے کے لیے مواصلاتی اینڈ پوائنٹس سیٹ اپ کریں۔
4.  **پبلشرز/سبسکرائبرز بنائیں**: یونٹی میں C# اسکرپٹس لکھیں تاکہ سیمولیٹڈ سینسر ڈیٹا کو ROS 2 موضوعات پر شائع کیا جا سکے اور ROS 2 سے کنٹرول کمانڈز کو سبسکرائب کیا جا سکے۔
5.  **HRI تیار کریں**: روبوٹ کے ساتھ تعامل کرنے کے لیے یونٹی میں انٹرایکٹو ڈیش بورڈز، کنٹرول پینلز، یا VR/AR انٹرفیس بنائیں۔

#### مثال: یونٹی سے ROS 2 پر کیمرہ ڈیٹا شائع کرنا

یونٹی میں یہ تصوراتی C# اسکرپٹ سیمولیٹڈ کیمرہ امیجز کو ROS 2 موضوع پر شائع کرے گا۔

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // Assuming Sensor messages are generated

public class UnityCameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/unity_camera/image_raw";
    public Camera renderCamera; // Assign your Unity camera here
    public int imageWidth = 640;
    public int imageHeight = 480;

    void Start()
    {
        ros = ROSConnection.Get = Instance();
        ros.RegisterPublisher<ImageMsg>(topicName);
    }

    void Update()
    {
        // RenderCamera سے تصویر کیپچر کریں
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderCamera.targetTexture;

        renderCamera.Render();

        Texture2D image = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        image.Apply();
        RenderTexture.active = currentRT;

        // Texture2D کو بائٹ سرنی میں تبدیل کریں
        byte[] rawImageData = image.EncodeToPNG(); // یا EncodeToJPG();

        // Create ROS 2 ImageMsg
        ImageMsg imageMsg = new ImageMsg();
        imageMsg.header.stamp = ros.rosTime.Now();
        imageMsg.header.frame_id = "unity_camera_frame";
        imageMsg.width = (uint)imageWidth;
        imageMsg.height = (uint)imageHeight;
        imageMsg.encoding = "rgb8"; // یا فارمیٹ کے لحاظ سے "rgba8"
        imageMsg.is_bigendian = 0;
        imageMsg.step = (uint)(imageWidth * 3); // rgb8 کے لیے فی پکسل 3 بائٹس
        imageMsg.data = rawImageData; // خام تصویری ڈیٹا تفویض کریں

        ros.Publish(topicName, imageMsg);

        Destroy(image); // Texture2D کو صاف کریں
    }
}
```

## انٹرایکٹو ماحول بنانا

یونٹی کا مضبوط ایڈیٹر بصری طور پر شاندار اور انتہائی انٹرایکٹو ماحول بنانے کی اجازت دیتا ہے۔ آپ آسانی سے اشیاء شامل کر سکتے ہیں، مواد کی تعریف کر سکتے ہیں، روشنی سیٹ اپ کر سکتے ہیں، اور یہاں تک کہ C# اسکرپٹنگ کا استعمال کرتے ہوئے پیچیدہ ماحولیاتی رویوں کو نافذ کر سکتے ہیں۔ یہ یونٹی کو ایسے منظرناموں کو تیار کرنے کے لیے ایک بہترین انتخاب بناتا ہے جہاں انسانی تعامل یا فوٹو ریئلسٹک رینڈرنگ سب سے اہم ہو۔

## مزید پڑھنا

- [گٹ ہب پر یونٹی روبوٹکس ہب](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector دستاویزات](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [یونٹی کے لیے URDF امپورٹر](https://github.com/Unity-Technologies/URDF-Importer)
