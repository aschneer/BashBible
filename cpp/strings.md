# Raw String Literals

Don't need to escape any special characters within the raw string literal.

From Bard.

```cpp
// Use the following. The raw string literal goes inside
// the parentheses.
R"(my string with special chars)" // default delimiter
R"custom_delim(my string with special chars)custom_delim" // custom delimiters
    // Custom delimiters seem to have no practical
    // effect on the string output. They are simply
    // used for clarity and are optional. If you want to
    // include `()` or `"()"` in the string without escaping,
    // using custom delimiters can make it more readable.

// Standard encoding
std::string standard_string = R"(This is a raw string literal. No need to escape \n or \.)";

// UTF-8 encoding
std::string utf8_string = u8R"(\u00A9 Copyright 2024)";
```
