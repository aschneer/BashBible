# Switch Case

## From Gemini (Bard)

```cpp
switch (expression) {
    case value1:
        // code to execute if expression equals value1
        break;
    case value2:
        // code to execute if expression equals value2
        break;
    // ... more case labels
    default:
        // code to execute if expression doesn't match any case
        break;
}
```

## Break Statements

Normally, each `case` block needs a `break` at the end, otherwise it will "fall through" to the next block, meaning every `case` will be visited. `break` at the end of each `case` ensures that only one `case` is visited.

However, you only need a `break` if the body of the `case` does not otherwise exit the scope or exit the program. The following eliminate the need for a `break` statement in a `case` block:

* `return`
* `CHECK(false)` (an assertion or similar that is always false).
* `LOG(FATAL)`
* `exit()`

## Enum Class and Switch Statements

In newer versions of C++, the compiler can give errors (optional feature) if you run a switch statement and you don't have exactly one case for each `enum` value.

It's better to not include a `default` case if this feature is used, because then the compiler will catch such an issue early, whereas otherwise you wouldn't catch it until runtime.
