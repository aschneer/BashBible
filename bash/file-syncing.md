# File Syncing

```bash
# rsync over SSH.
# Sync a local folder to a remote folder.
rsync -a ~/code username@192.168.1.45:code/all/
```

```bash
# rsync to mirror 2 directories. Delete extraneous files in receiver.
rsync -a --delete ./a/ ./b
	# Syncs folder a with folder b.
	# Must use "./a/" or "a/", not "a/*" to delete extraneous files in receiver.
	# The wildcard syncs the files in the directory, not the directory.
	# -a = archive mode, combination of desirable settings, use it always.
rsync -a --delete a b				# Copies folder a into folder b.
rsync -a --delete a/ b				# Syncs directory b with directory a so they are mirrors of each other.
	rsync -a --delete ./a/ b		# Equivalent
	rsync -a --delete ./a/ b/		# Equivalent
	rsync -a --delete ./a/ ./b/		# Equivalent
```

```bash
# rsync: Exclude from transfer "update.sh" file in source folder.
# Delete all files in destination that are not in source.
# Delete all files in destination that are in source but excluded from sync.
rsync -az -e ssh --delete --filter="- update.sh" --delete-excluded ./ vpu0-0a:~/aschneer/
	# -z = compress data during transfer.
	# -e ssh = use ssh protocol
```

```bash
# Run a "diff" to compare two files.
sdiff
```