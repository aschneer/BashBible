# Number Operations

```bash
# Converting between number bases:
x=$(printf %X 85)	# This works very well - it's the best way.
# From anything to decimal:
x=$((16#FC))		# Hex to decimal.
x=$((10#85))		# Decimal to decimal.
x=$((2#01001011))	# Binary to decimal.
hexNum=0xFF; echo "obase=10; ibase=16; $hexNum" | bc	# Input base = 16, output base = 10.
# Convert to Binary (0-255, 8 bits only):
D2B=({0..1}{0..1}{0..1}{0..1}{0..1}{0..1}{0..1}{0..1})
echo ${D2B[decNum]}
```

```bash
# Handling floating point numbers:
# Bash DOES NOT SUPPORT FLOATING POINT NUMBERS."
# Workarounds:
https://stackoverflow.com/questions/12722095/how-do-i-use-floating-point-arithmetic-in-bash
echo print 1/3. | python2
# Convert a floating point number that has been multiplied by 1000 to its
# floating point value.
floatVal=$((multVal / 1000))"."$((multVal % 1000))
	# floatVal = actual floating point number, like 173.45
	# multVal = integer version, floatVal multiplied by 1000, like 173450
```

```bash
# Floating point math:
a=$((12345 / 1000))"."$((12345 % 1000))
	# Result is "12.345".
```
