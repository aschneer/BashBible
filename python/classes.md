# Classes

## Reference Table

|                                             | No Instance / Class Level                                                                                                                                                       | With Instance / Instance Level                                                                                                                                                                                                                                                                                                                                                               |
| ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Prefix Convention                           | `cls`                                                                                                                                                                           | `self`                                                                                                                                                                                                                                                                                                                                                                                       |
| Define Variable                             | `cls.var1 = 0`                                                                                                                                                                  | `self.var1 = 0`                                                                                                                                                                                                                                                                                                                                                                              |
| Define Method                               | <pre class="language-python"><code class="lang-python">@classmethod
def cls_method(cls):
</code></pre>                                                                          | <pre class="language-python"><code class="lang-python">def inst_method(self):
</code></pre>                                                                                                                                                                                                                                                                                                  |
| Access Class Attribute from Outside Class   | <pre class="language-python"><code class="lang-python">MyClass.var1 = 3
</code></pre>                                                                                           | <pre class="language-python"><code class="lang-python">a = MyClass()
a.var1 = 3
</code></pre><p>or</p><pre class="language-python"><code class="lang-python">MyClass.var1 = 3
</code></pre>                                                                                                                                                                                                  |
| Access Class Attribute from Inside Class    | <pre class="language-python"><code class="lang-python">cls.var1 = 3
</code></pre><p>or</p><pre class="language-python"><code class="lang-python">MyClass.var1 = 3
</code></pre> | <pre class="language-python"><code class="lang-python"><strong>cls.var1 = 3
</strong></code></pre><p>or</p><pre class="language-python"><code class="lang-python">MyClass.var1 = 3
</code></pre><p>or</p><pre class="language-python"><code class="lang-python">type(self).var1 = 3
</code></pre><p>or</p><pre class="language-python"><code class="lang-python">self.var1 = 3
</code></pre> |
| Access Instance Variable from Outside Class | Not possible                                                                                                                                                                    | <pre class="language-python"><code class="lang-python">a = MyClass()
a.var1 = 3
</code></pre>                                                                                                                                                                                                                                                                                                |
| Access Instance Variable from Inside Class  | Not possible                                                                                                                                                                    | <pre class="language-python"><code class="lang-python">self.var1 = 3
</code></pre>                                                                                                                                                                                                                                                                                                           |
| Call Class Method from Outside Class        | <pre class="language-python"><code class="lang-python">MyClass.method()
</code></pre>                                                                                           | <pre class="language-python"><code class="lang-python">a = MyClass()
a.method()
</code></pre><p>or</p><pre class="language-python"><code class="lang-python">MyClass.method()
</code></pre>                                                                                                                                                                                                  |
| Call Class Method from Inside Class         | <pre class="language-python"><code class="lang-python">cls.method()
</code></pre><p>or</p><pre class="language-python"><code class="lang-python">MyClass.method()
</code></pre> | <pre class="language-python"><code class="lang-python">cls.method()
</code></pre><p>or</p><pre class="language-python"><code class="lang-python">MyClass.method()
</code></pre><p>or</p><pre class="language-python"><code class="lang-python">type(self).method()
</code></pre><p>or</p><pre class="language-python"><code class="lang-python">self.method()
</code></pre>                  |
| Call Regular Method from Outside Class      | Not possible                                                                                                                                                                    | <pre class="language-python"><code class="lang-python">a = MyClass()
a.method()
</code></pre>                                                                                                                                                                                                                                                                                                |
| Call Regular Method from Inside Class       | Not possible                                                                                                                                                                    | `self.method()`                                                                                                                                                                                                                                                                                                                                                                              |

From Codeium:

When you access an attribute with `self`, Python first looks for it in the instance's namespace. If it doesn't find it there, it then checks in the class namespace (and subsequently in the namespaces of its base classes, if any).

## Define a Class

```python
class MyClass:
	# class definition

class MyClass(ClassToInheritFrom):
	# class definition
```

## Member Variables

Define class attributes.

```python
class MyClass:
	class_attribute_1 = 0

	@classmethod
	def class_method(cls):
		# Change value of an existing class attribute.
		cls.class_attribute_1 = 2
		# Create a new class attribute.
		cls.class_attribute_2 = 4
```

Define instance variables.

```python
class MyClass:
	self.myvar1 = 2
	
	def __init__(self):
		self.myvar2 = 6
```

## Methods

Class methods, instance methods, and static methods:

```python
class MyClass:
	@classmethod
	def class_method(cls, arg1, arg2):
		# Must only pass `cls`.
		# You can't pass `self` (the instance) to
		# a class method because there's no instance.
		print("This is a class method")
	
	def regular_method(self, arg1, arg2):
		# Must only pass `self`.
		print("This is a regular method")
	
	@staticmethod
	def static_method(arg1, arg2):
		# Cannot pass `cls` or `self`.
		# Has no access to class or instance.
		print("This is a regular method")
```

## Accessing from Outside

Class attributes and methods can be accessed using either the class name or instance name.

The class name cannot be used to access instance member variables and methods.

```python
class MyClass:
	class_attribute = 0

	@classmethod
	def class_method(cls):
		print("hello world")

# Access using class name.
print(MyClass.class_attribute)
MyClass.class_method()

# Access using instance name.
instance = MyClass()
print(instance.class_attribute)
instance.class_method()
```

Instance member variables and regular methods can be accessed only using the instance name. They cannot be accessed using the class name.

```python
class MyClass:
	self.myvar1 = 2
	
	def __init__(self):
		self.myvar2 = 6

	def print_vals(self):
		print(self.myvar1)
		print(self.myvar2)

# Access using instance name.
instance = MyClass()
print(instance.myvar1)
instance.print_vals()
```

The values of class attributes will persist throughout the program, as instances of the class come and go. Instance member variables and methods live only within the instance.

## Accessing from Inside

```python
class MyClass:
	# Class attribute
	class_attribute = 140
	# Instance variable
	self.myvar1 = 0
	
	# Instance constructor
	def __init__(self):
		self.myvar2 = 6
	
	# Class method
	@classmethod
	def class_method(cls):
		# Access class attribute
		print(cls.class_attribute)
		# Access instance variable
		self.myvar1 = 2
		# Check and get class attribute
        if hasattr(MyClass, 'class_attribute'):
	        temp = getattr(MyClass, 'class_attribute')
	        setattr(MyClass, 'class_attribute', 17)
	
	# Instance method
	def print_vals(self):
		# Access instance variables
		print(self.myvar1, self.myvar2)
		# Access class attribute
		print(type(self).class_attribute)
		# Set instance variable
        setattr(self, 'myvar2', 43)
        # Set class attribute
        setattr(MyClass, 'class_attribute', 17)
	
	# Static method
	@staticmethod
	def print_text(text_to_print: str):
		print(text_to_print)
```

## Using Attribute Functions

```python
class MyClass:
	attr1 = 0
	self.var1 = 0
```

From either inside or outside class def:

```python
hasattr(MyClass, "attr1")
getattr(MyClass, "attr1")
setattr(MyClass, "attr1", 3)
delattr(MyClass, "attr1")
```

Only from inside class def:

```python
hasattr(self, "var1")
getattr(self, "var1")
setattr(self, "var1", 3)
delattr(self, "var1")
```

```python
def instance_method(self):
	hasattr(self, "var1")
	getattr(self, "var1")
	setattr(self, "var1", 3)
	delattr(self, "var1")
```

```python
@classmethod
def class_method(cls):
	hasattr(cls, "attr1")
	getattr(cls, "attr1")
	setattr(cls, "attr1", 3)
	delattr(cls, "attr1")
```

```python
hasattr(type(self), "attr1")
getattr(type(self), "attr1")
setattr(type(self), "attr1", 3)
delattr(type(self), "attr1")
```

Only from outside class def:

```python
intance = MyClass()
hasattr(instance, "var1")
getattr(instance, "var1")
setattr(instance, "var1", 3)
delattr(instance, "var1")
```

```python
intance = MyClass()
hasattr(type(instance), "attr1")
getattr(type(instance), "attr1")
setattr(type(instance), "attr1", 3)
delattr(type(instance), "attr1")
```
