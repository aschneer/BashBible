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
# Single loop
list_a = [1, 2, 3, 4, 5]
list_b = [str(item) for item in list_a]
# Result will be:
["1", "2", "3", "4", "5"]

# Single loop with conditional
list_a = [1, 2, 3, 4, 5]
list_b = [str(item) if item > 2 else str(item + 3) for item in list_a]
# Result (list_b) will be:
["4", "5", "3", "4", "5"]

# Nested loop
list_a = [[1, 2], [2, 3], [3, 4], [4, 5], [5, 6]]
list_b = [num + 1 for item in list_a for num in item]
# Result (list_b) will be:
[2, 3, 3, 4, 4, 5, 5, 6, 6, 7]

# Nuances about conditionals:
#
# If only "if" and no "else", it goes after the for loop(s).
list_b = [num + 1 for item in list_a for num in item if num > 2]
# If using "if" AND "else", it goes before the for loop(s).
list_b = [num + 1 if num > 2 else num for item in list_a for num in item]
```