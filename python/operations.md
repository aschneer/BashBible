# Operations

Object and list unpacking

```python
# Unpack list_a items into list_b
list_a = ['how', 'are', 'you']
list_b = ['hello', 'world', *list_a]

# Unpack dict_a into individual keys/values
dict_a = {
	'hello': 'world',
	'howare': 'you'
}
def func(hello: str, howare: str) -> None:
	print(hello)
	print(howare)
func(**dict_a)
# This will print the following:
world
you
```

List comprehension

```python
list_a = [1, 2, 3, 4, 5]
list_b = [str(item) for item in list_a]
```