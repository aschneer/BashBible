# Functions

```bash
# Arguments:
$0	# Name of script.
$1 $2 $3 $4...	# Arguments.
$#	# Number of arguments passed to script, starting with $1.
```

```bash
# Functions:
function_name () {
  commands
}
function_name () { commands; }
function function_name {
  commands
}
function function_name { commands; }
func_result="$(function_name)"	# Call function and save result to variable.
# Function arguments are not explicitly specified.
# Just feed them in after calling the function, separated by spaces.
function_name "Joe"
echo $0		# Prints name of function.
echo $1		# Prints first argument.
echo $2		# Prints second argument (none in this case).
# ...and so on.
```

```bash
# Functions don't return a value.
# They return an exit status.
echo $?		# Print exit status of function.
# Exit status is status of last line executed in function.
return 55	# Returns 55 as the exit status of the function ($?).
# Can return values like this:
my_function () {
  local func_result="some result"
  echo "$func_result"
}
func_result="$(my_function)"
echo $func_result
# Can also return by assigning value to global variable.
my_function () {
  func_result="some result"
}
my_function
echo $func_result
```

