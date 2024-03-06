# File System

Dummy "null" file included in Linux distros:

```bash
/dev/null
```

Information about mounted disks:

```bash
# Get file system information
# for mounted disks only (disk space/usage, etc.):
df -h
	# -h is human readable
```

Listing files in directories (useless):

```bash
# File list:
ls -shla	# List files, include sizes, human readable.
ls -Shla	# Same as above, but sort by largest files first.
	# NOTE - -s and -S do not provide total size of directory contents,
	#		only the directory itself (which is useless).
	# -l = vertical list
	# -a = include all files/folders, including hidden
```

Disk usage (useful):

```bash
# Get total disk usage of files and folders.
du -shc ./* | sort -hr
	# Get total disk usage of everything in current directory.
	# -s = summary, don't list any subfolders or files.
	# -h = human readable file sizes
	# -c = return total size of each path
	# --threshold or -t = exclude files below this size if positive,
	#		exclude files above this size if negative.
	# sort -hr
	#		-h = human readable
	#		-r = reverse order (largest to smallest)
du -hca --threshold=100M ./* | sort -hr > result.txt
	# -a = all files and directories
		# Cannot use -s and -a at the same time
	# --threshold or -t = exclude files below this size if positive,
	#		exclude files above this size if negative.
	#		Use this if the list is too long
	# Save to text file result.txt if easier to look
	# 		through result in text editor.
```

Disk usage tree:

```bash
# Package to view file/directory sizes.
ncdu
	# Scroll through directories and
	# see disk usage in real time.
```

Recursive file list:

```bash
# List all files within directory, including subdirectories:
find . -type f
find ./ -type f
```

File search by substring in name, recursive:

```bash
# Search files recursively by name (substring).
	# "*substring*" asterisks are necessary to
	# search for a substring.
	# "-print" flag seems to be optional
find ./my_path/ -iname "*json*" <-print> # NOT case-sensitive
find ./my_path/ -name "*json*" <-print> # case-sensitive

# Search for multiple criteria.
find ./my_path/ -iname "*myfile*" -and -iname "*.json*" # both
find ./my_path/ -iname "*myfile*" -or -iname "*.json*" # either

# Search contents of files recursively
	# -r = recursive
	# -i = non-case-sensitive
	# -n = include line number of result within its file
grep -rin "hello-world" # current directory
grep -rin "hello-world" ./my_path
grep -rin "hello-world" ./my_path/ # equivalent
grep -rin "hello-world" ./my_path/* # NOT equivalent, not sure why

# Don't show any context. Just show the matched search terms.
grep -o "term" file.txt

# Show all context (entire text).
	# 9999 is an example.
	# It just needs to be a large number.
	# 0 is supposed to work, but it didn't for me.
grep -C 9999 "term" file.txt

# Search for multiple search terms.
# AND logic:
grep -e "term1" -e "term2" -e "term3" file.txt
# OR logic:
grep -E "term1|term  2|search term 3" file.txt

# Wildcards
grep -rin "hello.*" ./my_path
# returns any "hello_" ending in "()" with
# anything in between.
grep -rin "hello_.*()" ./my_path
```

```bash
# Get information for all connected disks, mounted or not:
lsblk -fT -o +SIZE
	# -a = all
	# -f = include filesystem info
	# -T = force tree output view
	# -o +[COLUMN] = adds additional info column by name
```

```bash
# Get UUID and filesystem type of disk:
blkid /dev/sda1 # replace sda1 with the desired partition.
```

```bash
# Get disk information:
sudo fdisk -l
```

```bash
# Check a disk or individual partition:
sudo fsck /dev/sda	# check disk
sudo fsck /dev/sda1	# check partition
sudo e2fsck /dev/xxxx # not sure what this does differently
```

```bash
# Mount/unmount partitions (must be listed in /etc/fstab?)
# (the mounts don't always work for me, but unmounting does).
sudo mount /dev/sdxx
sudo mount /dev/sdxx /media/oakdell/1TB_CH0	# second arg is mount point?
sudo mount -a	# mount all available disks
sudo umount /dev/sdxx
```

```bash
# Format partition:
mkfs.ext4 /dev/sda1	# Format partition to ext4. Replace sda1 with desired partition.
```

```bash
# Partition tools:
gdisk # GPT fdisk - best for advanced drive tools, partitioning
```