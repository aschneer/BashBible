# Loops

```bash
# Infinite While Loop:
while :
do
	# actions
	break	# optional
done
```

```bash
# Regular While Loop:
i=0
while [[ $i -lt 11 ]]
do
	echo $i
	((i++))
done
```

```bash
# For Loops:
# All the following are equivalent:
for i in 1 2 3 4 5
for i in {1..5}
for i in {1..5..1}	# {start..end..increment}
for (( c=1; c<=5; c++ ))
do  
  echo "Hello $c"
done
```

```bash

```