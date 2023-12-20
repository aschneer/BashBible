# How to Use `tinyxml2`

https://github.com/leethomason/tinyxml2

From Codium.

To create a new XML file, add elements to it, and write it to the disk using the TinyXML2 library, you can follow these steps:

1. Include the TinyXML2 header file in your code:

```cpp
#include "tinyxml2.h"
```

2. Create an instance of the `XMLDocument` class:

```cpp
tinyxml2::XMLDocument doc;
```

3. Create the root element of your XML document:

```cpp
tinyxml2::XMLElement* root = doc.NewElement("Root");
doc.InsertEndChild(root);
```

4. Add elements to the root element or any other element in your XML document:

```cpp
tinyxml2::XMLElement* element = doc.NewElement("Element");
element->SetAttribute("Attribute", "Value");
root->InsertEndChild(element);
```

5. Write the XML document to a file on the disk:

```cpp
doc.SaveFile("path/to/your/file.xml");
```

Make sure to replace `"path/to/your/file.xml"` with the desired path and filename for your XML file.

That's it! You have created a new XML file, added elements to it, and written it to the disk using TinyXML2.