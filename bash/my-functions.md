# My Functions

## Spinning Wheel

Simple spinning wheel to show something is loading, or to show data is being updated and not stale.

Arguments:
- $1 = index (required); if not provided, will reset to zero.

```bash
function wheel() {
	chars="|/-\\"
	index=$1
	if [[ index -lt 0 ]]
	then
		index=0
	elif [[ index -ge 3 ]]
	then
		index=0
	else
		index=$((index + 1))
	fi
	echo ${chars:$index:1}
}
```

