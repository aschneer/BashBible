# Data Structures

```bash
# Arrays:
myArray=("cat" "dog" "mouse" "frog")	# Declare.
myArray[3]="hello"	# Update fourth array element.
echo ${myArray[3]} # Access/print fourth array element.
# Declare is recommended over eval.
```

```bash
# Loop through array elements:
for str in ${myArray[@]}	# Loop through array indices.
do
  echo $str
done
```

```bash
# Loop through array indices:
for i in ${!myArray[@]}
do
  echo "element $i is ${myArray[$i]}"
done
# ! accesses array index, not element.
# @ returns entire array, not just first element.
```

```bash

```