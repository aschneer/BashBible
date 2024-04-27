# Namespaces

Namespace alias

```cpp
// Use type name "json" instead of "nlohmann::json"
using json = nlohmann::json;
```

A `::` with nothing in front of it means starting at the global namespace.

```cpp
// Google Test example:
class MyClassTest : public ::testing::Test {
};
```

A child namespace has access to everything in its parent/ancestor namespaces.

```cpp
my::parent {

class ParentNameClass {
};

}

my::parent::child {

ParentNameClass parent_instance;

}
```

