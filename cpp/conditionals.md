# Conditionals

## Conditional (Ternary) Operator

Syntax

```cpp
condition ? value_if_true : value_if_false
```

How it's Used

```cpp
void PrintCowName(std::string name) {
    std::cout << "Cow's name is: " << name << std::endl;
}

bool is_a_cow = true;
PrintCowName(is_a_cow ? "tom" : "not_a_cow");
```
