# Text Operations

```bash
# String Concatenation:
x="hello"
y="world"
echo $x$y	# Prints "helloworld"
echo $y$x	# Prints "worldhello"
```

```bash
# Substrings:
x="hello"
echo ${x:0:3}	# Prints "hel" (3 chars starting at char 0)
echo ${x:3}		# Prints "lo" (char 3 through end)
```

```bash
# Find and replace within string:
${main_string/search_term/replace_term}	# First occurrence only.
${main_string//search_term/replace_term}	# All occurrences.
```

```bash
# Special characters:
n=$'\n'	# newline
t=$'\t'	# tab
r=$'\r'	# carriage return
```

```bash
# Get length of string:
str="hello"
echo ${#str}
```

```bash
# Grep:
grep -i "literal"	# return lines containing 'literal', case insensitive
grep -v "hello"		# return lines NOT containing 'hello', case sensitive
grep -E "hello|world|pizza"	# return lines containing 'hello' OR 'world' OR 'pizza'
grep "he..owo..d"	# return lines containing he, owo, d with anything in between (. is wildcard)
	# Examples = helloworld, heppowoeed, hekkowossd, etc.
grep "AB[CDE]HI[JKL]"	# returns lines containing any character in brackets in those positions
	# Examples = ABCHIJ, ABDHIJ, ABEHIL, etc.
```