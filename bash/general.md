# General

## Shebang
Include at the top of any bash script file.

```bash
#!/bin/bash
```

General Navigation / Interaction with Bash Shell

```bash
history  # bash command history
cat  # print file (all contents)
less  # print file (scrollable)
	/keyword  # search within less result
tail  # print last several lines of a file
watch  # watch file contents change in real time
echo [text]  # print specified text
printf [text]  # print specified text, highly customizable
pwd  # print full path of current working directory
```

```bash
### WARNING ### - be very careful with this
# Delete directory (includes all files contained within):
rm -rf
	# -r = recursive, -f = force
```

chmod:

```bash
sudo chmod <who to assign><permissions> <file/folder names>
sudo chmod u+x ./file.txt  # assigns the user execute permissions
sudo chmod +x ./file.txt  # assigns all users execute permissions
	sudo chmod a+x ./file.txt  # equivalent
sudo chmod g-x ./file.txt  # removes execute permissions for the group
# 'who' codes
o  # owner
g  # group
u  # user
a  # all users
# Binary permission codes. Add together to assign.
r = 4 = 100b
w = 2 = 010b
x = 1 = 001b
```

| binary code | decimal code | owner | group | user (all others) | 
| ----------- | ------------ | ----- | ----- | ----------------- |
| ooo ggg uuu |              |       |       |                   |
| 100 000 000 | 400          | r--   | ---   | ---               |
| 010 000 000 | 200          | -w-   | ---   | ---               |
| 001 000 000 | 100          | --x   | ---   | ---               |
| 000 100 000 | 040          | ---   | r--   | ---               |
| 000 010 000 | 020          | ---   | -w-   | ---               |
| 000 001 000 | 010          | ---   | --x   | ---               |
| 111 000 000 | 700          | rwx   | ---   | ---               |
| 110 000 000 | 600          | rw-   | ---   | ---               |
| 101 000 000 | 500          | r-x   | ---   | ---               |
| 011 000 000 | 300          | -wx   | ---   | ---               |
| 111 000 100 | 704          | rwx   | ---   | rw-               |

Environment Variables

```bash
export  # see all environment variables running
```

---

```bash
# Variables:
x=3		# No spaces between equal sign.
```

```bash
# Timing:
sleep 5	# Delay 5 seconds.
date +%s%3N # Get system time in milliseconds.
```

Printing:

```bash
# echo Command:
x="   hello   world   "
echo $x		# This automatically trims whitespaces from variable.
echo "$x"	# This prints the exact true string (incl. whitespaces), but no leading spaces.
echo $'\n'	# Print newline character.
echo -e "\n"	# Print newline character. -e enables interpretation of backslash escapes.
n=$'\n'; echo $n	# Store newline character to variable and use later.
```

```bash
# Printing:
echo hello; echo "hello" # Same result.
printf %s "hello"
printf '\r'%s "hello"	# Carriage return, goes back to beginning of same line and overwrites.
printf "\033[5A"	# Jump up 5 lines in the print screen.
printf "\033[5,4A"	# Jump up 5 lines and over 4 columns in the print screen. (I haven't tested this.)
```

Reading from standard input:

```bash
# Read from standard input (prompt for user input):
read
echo $REPLY; echo ${REPLY}	# Equivalent. Print what was last read in.
read -r	# -r = ignore backslash escapes
read line	# read user input and store to variable $line
read -u 3	# read from file descriptor 2 instead of standard input.
	# File descriptor must first be created with the following:
	exec 3< file	# Open $file to file descriptor 3. Don't include $ in front of 'file'.
	exec 3>&-	# Close file descriptor 3.
# Read returns 0 until end of file is encountered, it times out, or error occurs.
# IMPORTANT NOTE ABOUT FILE DESCRIPTORS:
#	File descriptors 0, 1 and 2 are for stdin, stdout and stderr respectively.
#	File descriptors 3, 4, .. 9 are for additional files.
```

Reading from file:

```bash
# Read text file line by line:
# Method 1:
file=$(cat temp.txt)
for line in $file
do
	echo -e "$line\n"
done
# Method 2:
#	File descriptor is created when file is opened, and stays open until closed.
# IMPORTANT NOTE ABOUT FILE DESCRIPTORS:
#	File descriptors 0, 1 and 2 are for stdin, stdout and stderr respectively.
#	File descriptors 3, 4, .. 9 are for additional files.
exec 3< file; # open file file as fd 3 (fd = file descriptor)
echo "hello" > file	# write to file
read -ur 3	# Reads 1 line and stores to $REPLY.
read -ur line	# Reads 1 line and stores to $line.
	# Keep running this, and it will read each line sequentially.
	# Close file and re-open to start at the beginning again.
	# -u = reads from file instead of stdin
	# -r = ignores backslash escapes
exec 3>&-	# Close file descriptor 3.
# Method 3:
file="temp.txt"
while read -r line
do
	echo -e "$line\n"
done <$file
```

Writing to a file:

```bash
# Write to a file; overwrite everything.
echo "hello" > ./file.txt
# Write to a file; append, don't overwrite.
echo "hello" >> ./file.txt
```

---

```bash
# Start/stop a process.
sudo systemctl start [process]
sudo systemctl stop [process]
```

Bash Configuration

```bash
# Add commands that execute every time Bash shell opens.
echo "[commands]" >> ~/.bashrc
```

Source Command

```bash
# Run file, and all variables created by it will be added to the current context.
source ./script.sh
# Add this path to the current shell's context.
# Allows you to execute files from this directory by name
# without providing the complete path.
source ~/andrew/robotics/scripts/
```
