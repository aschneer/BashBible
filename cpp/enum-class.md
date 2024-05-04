# Enumerated Classes

```cpp
// Type explicitly declared.
enum class MyConstants : int {
    kStartTime = 0900,
    kEndTime = 1700
};

// Type not declared. Implicitly "int" by default.
enum class DayOfWeek {
    kMonday = 1,
    kTuesday = 2,
    kWednesday = 3,
    kThursday = 4,
    kFriday = 5,
    kSaturday = 6,
    kSunday = 7,
};

// Values not initialized. Will default
// to 0, 1, 2, etc., with or without type.
enum class MyConstants : int {
    kStartTime, // 0
    kEndTime, // 1
};
enum class MyConstants {
    kStartTime, // 0
    kEndTime, // 1
};
```
