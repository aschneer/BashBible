# Classes

## Overview

```cpp
// Class definition
class Hobbit {
public:
    string name;
    float height; //in meters
    int age;

    Hobbit();  // Constructor
    ~Hobbit(); // Destructor
    
    void get_name() {
        cout << "This hobbit's name is " << name << endl;
    }
    void convert_height_to_cm();

private:
    //
protected:
    //
};

// Class method definitions
Hobbit::Hobbit() {
}
Hobbit::~Hobbit() {
}
void Hobbit::convert_height_to_cm() {
    height = height*100;
    cout << "Height of the hobbit is " << height << " centimeters" << endl;
}
```

## Inheritance

### Inheriting from Multiple Classes

```cpp
class DerivedClass : public BaseClass1, protected BaseClass2, private BaseClass3 {
};
```

### Inheriting from Classes Without a Default Constructor

The default constructor of each base class will be called when the derived class is instantiated. If the base class does not have a default constructor, there will be a compilation error.

To get around this, you can explicitly call whatever constructor exists for each base class in the initialization list of the derived class' constructor.

```cpp
class DerivedClass : public BaseClass1, protected BaseClass2 {
    DerivedClass() : BaseClass1(0, "hello"), BaseClass2(17, "world") {}
};
```

### Inheritance Keywords

#### Base Class

* `public`: accessible to anything outside or inside the class
* `protected`: members will be available to derived classes, but private otherwise.
* `private`: only accessible to methods within the class
* `friend`: friend class can access private and protected members of class in which friend in declared.

#### Derived Class Signature:

* `public` Inheritance ("as-is"):
  * `public` base -> `public` derived
  * `protected` base -> `protected` derived
  * `private` base -> not accessible in derived
* `protected` inheritance:
  * `public` base -> `protected` derived
  * `protected` base -> `protected` derived
  * `private` base -> not accessible in derived
* `private` inheritance:
  * `public` base -> `private` derived
  * `protected` base -> `private` derived
  * `private` base -> not accessible in derived

#### Friend Classes

Can declare under any of the `public`, `private`, or `protected` base class keywords; doesn't make any difference.

```cpp
class MyClass {
private:
    friend class OtherClassThatWantsPrivateAccess;
};
```

## Unit Testing Private/Protected Class Methods

The convention is to only test public methods of a class. If it is insufficient to only test public methods, it is said that your class needs restructuring.

However, there are some cases where you might want to test private/protected methods of a class. Perhaps an example would be if you are incrementally improving the robustness of that class and you want to do it in pieces, without having to refactor the entire thing at once.

Below are two strategies.

### 1. Use Inheritance (recommended)

* Have the test fixture class inherit publicly from the class being tested
* Any `public` and `protected` methods in the class under test will be available to the test fixture class, as long as the class under test is inherited publicly.
* `private` methods will not be accessible

### 2. Use a Friend Class (more complicated, not recommended)

* Declare the test fixture class as a `friend` class inside the class under test. It doesn't matter which privacy tag it sits under (`public`, `protected`, or `private`).
  * `friend class path::to::test::TestFixtureClass;`
* The test fixture class will then have access to all `public`, `protected`, and `private` members and methods of the class under test.
* Add a forward declaration of the test fixture class to the file containing the class under test. Otherwise the friend class declaration will fail because it doesn't have a definition of the test class.

```cpp
namespace path::to::test {
    class TestFixtureClass;
}
```

* If you're using Google Test for C++, only the test fixture class will have access to the private members. The test cases (`TEST`, `TEST_F`, `TEST_P`, etc.) will not. To get around this, you have to create helper functions in the test fixture class that the test cases will call. The helpers will simply call the private/protected methods being tested and pass the return values back.

{% code title="circle.h" %}
```cpp
namespace path::to::test {
    class TestFixtureClass;
}

class Circle {
private:
    double Circumference(double diam);
    
    friend class MyTestFixture;
};
```
{% endcode %}

<pre class="language-cpp" data-title="circle_test.cpp"><code class="lang-cpp"><strong>class MyTestFixture : public ::testing::Test {
</strong>    // CircleClass is the class whose private method
    // Circumference() is being tested.
    double CircumferenceHelper(const CircleClass&#x26; circle, const double&#x26; diam) {
        return circle.Circumference(diam);
    }
};
</code></pre>

* Finally, you call the helper function in the test case.

{% code title="circle_test.cpp" %}
```cpp
TEST_F(MyTestFixture, MyTestCase) {
    double diam = 10;
    EXPECT_EQ(CircumferenceHelper(diam), diam * 3.14) << "Circumference failed.";
}
```
{% endcode %}
