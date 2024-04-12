# Lesson 01 - Notes

Link: \
https://www.youtube.com/watch?v=7EmboKQH8lM&list=PLdpsE-GEhYVn_81kDPo1mwE73UgYCeMLu&index=1

## What is clean code?

"Clean code does **one thing well.**" - creator of C++

- Code should read like well-written prose.
- Good code is short
	- Short code is easier to read
- Use explanatory variables
	- Instead of putting a complicated conditional directly into an if statement, instead make an explanatory boolean variable that explains what condition the complex conditional represents, then have the if statement operate on that boolean variable. The explanatory variable gives the reader a higher-level understanding of what the code represents.
	```python
	# Instead of:
	if (complex == conditional and not (more or complexity)):
		run_operation()
	
	# Do this:
	data_pipe_open = (complex == conditional and not (more or complexity))
	if (data_pipe_open):
		run_operation()
	```
	
- Functions should be polite
	- Rules for writing an article/paper
		- Title
		- First paragraph is a synopsis
		- Following paragraphs are increasingly detailed
	- Allows reader to exit early
		- Start with headline. If interesting, keep reading.
		- Then read first paragraph. If still interested, keep reading.
		- Etc.
	- Good functions are readable at the level of detail desired by the reader
		- Allows them to exit early if they don't care or don't need to know more detail
- Functions should be small
	- Smaller is easier to understand
	- How small? As small as possible.
	- **Functions should only do one thing.**
		- **A function does one thing if you cannot meaningfully extract another function from it.**
			- IDE's like IntelliJ have "Extract Method" tool, which will extract a method from a block of code
		- Similar idea to prime factorization.
		- Keep extracting until you can't extract anymore.
		- **As you extract functions, name them appropriately, and move them into appropriate classes and source files to create a semantic tree.**
	- Functions should not be large enough to hold nested structures
		- Indent level of a function should not be more than one or two
	- **Typically should be 3-5 lines long**
	- Once full extracted, `if` statements should have a function call as the conditional, and a function call as the body:
	```python
	if (buffer_full()):
		empty_buffer()
	```
	- Arguments
		- 1-3 arguments is okay, no more than 3 ideally
		- Should have few enough that a human can remember their order
		- If there are more than 3 arguments you need to pass, they are cohesive enough that they should be a class or struct - just pass the object
		- Don't use boolean arguments
			- If you pass a boolean, that means there's a conditional in the function, which means it can be broken into two functions, one for the true case and one for the false case
			- Occasional exceptions - like pass bool into function that sets the state of the switch, pass either true or false bool (on or off)
- Avoid switch statements
	- They break often
	- Example:
		- Switch that switches on the type of a shape
		- You would need a switch statement for every action take on or with a shape (draw, erase, rotate, etc.)
		- **If you add a shape, you need to find every switch statement, understand it, and modify it.**
	- Instead, use **polymorphism** (class inheritance, function overloading/overriding)
	- Have a base class and sub-classes
		- If you add a shape later, you simply add a sub-class
	- Dependency tree issues
		- **Switch statements are a dependency magnet**
		- If cases in switch statement call out to other modules, it creates a big dependency tree
		- If you change any dependency, you need to recompile the module with the switch
		- Makes it difficult to independently deploy modules
		- **You want to independently deploy GUI, database, etc. so you can redeploy each one alone if you need to make a change**
- Open/close principle
	- **A module should be open for extension, but closed for modification**
	- Accomplished with base classes and sub-classes
- Avoid if/else statements
	- d
- Principle of least surprise
	- Code should not be surprising
- Side-effect functions
	- Change the state of the system
	- They come in pairs (open/close, new/delete, malloc/free)
	- Prone to problems with controlling pairs of functions
	- Pairs must be called in the right order
	- How to make side-effect functions safe:
		- Use lambdas
		```python
		def open(file_path, action_lambda):
			f = open(file_path, 'r')
			action_lambda()
			f.close()
		```
		- This seems to be the equivalent of a `with` block in Python.
		- This structure ensures that the other half of the side-effect pair (close function) gets called after the first half (open function) gets called.
- - Command and query separation (convention)
	- **Commands:** Change the state of the system, return `void`
		- A function that returns `void` must have a side-effect, otherwise it doesn't do anything
	- **Queries:** Return a value, do not change state of system
		- **A function that returns a value should not have a side-effect; it should only operate on the arguments passed to it**
		- This way when you see a function that returns a value, you know it's safe to call it. It leaves the system in the same state it was found in.
- Use exceptions instead of returning error codes
	- **When using a `try` block, write a function where the only thing in the function is the `try` block**
	- Under `try`, it calls another function which is the function that throws the exception
	- No prefix code, no suffix code in the function containing `try`
	- Error processing is "one thing," so it belongs in its own function
	- **Never nest `try` blocks**
- **DRY Principle:** Don't Repeat Yourself
	- Avoid duplication
	- If code is being duplicated, instead put it in a function
	- How to avoid duplicating loops
		- Say you have a complex loop that operates on a tree or database to find a node, then operates on the mode
		- You can write a function that contains the loop, and pass a lambda to the function which does the processing of the node once it's found
- Structured programming
	- d