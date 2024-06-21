# VSCode

## Settings

Where VSCode stores settings JSON files (on Windows):

User:

```bash
C:\Users\username\AppData\Roaming\Code\User\settings.json
```

Remote:

```bash
C:\Users\username\.vscode-server\data\Machine\settings.json
```

Workspace:

```bash
...repo\.vscode\settings.json
```

## Code Analysis Extensions

### C++

**clangd:**

* Pros
  * Faster, instant click to definition
* Cons
  * More complicated to set up
  * Have to manually build repo and refresh compile commands regularly, and then restart cland language server to keep go-to-definition up-to-date
  * It seems to not recognize a lot of symbols (classes, types, etc.) - there are a lot of red squiggles
  * When ctrl + clicking on a class type, it often doesn't go to the definition of that class, but rather to a variable of that type being declared, which is useless.

**Microsoft C++ Extension:**

* Pros
  * Easy to install and set up
  * Updates compile commands automatically in the background
  * Better at following ctrl + click to definitions of classes
  * Recognizes almost all symbols, very few are not recognized
* Cons
  * Slower
    * Often takes a few seconds to think when ctrl + clicking to go-to-definition, especially when you first open a file or ctrl + click a symbol for the first time in that session
    * Sometimes need to wait while automated refreshing compile commands runs in the background, symbols are not recognized in the meantime
