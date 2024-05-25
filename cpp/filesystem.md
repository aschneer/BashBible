# Filesystem

## STL Filesystem

```cpp
#include <filesystem>
```

## Create a New Directory and All Parent Directories

```cpp
std::filesystem::path new_folder = "/home/my_username/new_folder";
std::filesystem::create_directories(new_folder); // equivalent of `mkdir -p`
```

## Join Paths

```cpp
std::filesystem::path path1 = "/home/my_username/new_folder";
std::filesystem::path path2 = "projects/cpp";
std::filesystem::path final_path = path1 / path2;
```
