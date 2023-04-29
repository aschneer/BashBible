# Data Types / Structures

Lists

```cpp
// Create list
list<int> numbers_list({1,10,100,1000});
list<string> vocals_list( {"a","e","i","o","u"} );

// Modify list
numbers_list.push_front(0);  //insert in the beginning
numbers_list.push_back(3000);  //insert in the end

// Concatenate lists
list<int> new_list({5,50,500});
numbers_list.insert(numbers_list.end(),new_list.begin(),new_list.end());
```

Dictionaries

```cpp
// Create dictionary (key/value pairs) where key is string and value is int
map<string,int> girls_dictionary;
// Assign values
girls_dictionary["Dolores"] = 30;
girls_dictionary["Maeve"] = 27;
girls_dictionary["Theresa"] = 6;
girls_dictionary["Clementine"] = 11;
// Print values
for (auto item : girls_dictionary)
	cout << item.first << " appears in " << item.second << " episodes\n";
```