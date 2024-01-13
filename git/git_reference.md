# Git Reference

## Branches

You can have any branch structure you want. Branches are stored in a directory structure. The files of the repo exist in the folder for a given branch.

In this example, `my_fancy_branch` is a branch in a folder called `andy`, which is in a folder called `unreviewed`. This structure organizes branches by their purpose. Any branch can be merged into any other branch regardless of this folder structure; it's just a way of organizing branches.

```txt
my_repo/
├── unreviewed/
│   └── andy/
│      └── my_fancy_branch/
└── ...
```

Git has special namespaces for specific purposes. The following is for unreviewed code branches.

```bash
my_repo/refs/for/
```

Under this, you would put your branch according to whatever folder structure your team uses.

```bash
my_repo/refs/for/
├── unreviewed/andrew.schneer/my_fancy_branch
```

Working with branches:

```bash
# Current latest local commit.
HEAD
# Current latest remote commit.
origin/HEAD

# Push the HEAD to a branch called "my_fancy_branch"
# in the git namespace "refs/for/" in the branch
# directory "unreviewed/andy/".
git push origin HEAD:refs/for/unreviewed/andy/my_fancy_branch
# This will trigger code review
git push origin HEAD:refs/for/master

# Pull latest master and rebase your
# local master on top of it.
git pull --rebase

# Push from specific local branch
# to specific remote branch.
git push [remote] [local branch]:[remote branch]
git push origin local_branch:remote_branch

# List all branches.
git branch
# Create a new branch as a copy of the current branch.
git branch branch_name
# Move to a different branch.
git checkout branch_name
# Delete a branch.
git branch -d branch_name
# Rebase new feature branch onto master
git rebase master new_feature
	# Alternatively:
	git checkout new_feature
	git rebase master

# Get a clean origin/master in detached head state
git checkout origin/master
# Save it to a new branch
git switch -c new_branch_name

# Duplicate current branch
# to new branch and checkout the new branch.
git switch -c new_branch_name

# Duplicate an existing branch
# and checkout the new branch copy.
git switch -c feature/branch2 feature/branch1
git checkout -b feature/branch2 feature/branch1

# Get branch releases/2023.1.1 in detached head state
git checkout origin/releases/2023.1.1

# Get remote branch releases/2023.1.1, and create a
# matching local branch to track the remote branch
git checkout releases/2023.1.1
```

Alternate way to pull down a remote branch:

```bash
# Checkout the remote branch in detached HEAD state
git checkout origin/folder/branch2023.12
# Save the retreived branch to a new local branch
git switch -c new_branch_name
```

Delete all local UNCOMMITTED changes to a specific file:

```bash
git restore path/to/file
git checkout -- path/to/file
```

Delete all local changes to a specific file in a commit:

```bash
# Move all committed changes back to staging area
# (tracked, but not committed).
git reset --soft HEAD~1
# Unstage/untrack the specific file.
git reset HEAD <file>
# Revert all unstaged changes.
git restore . # Confirmed this works
# or...
git checkout . # Not tested
```

Working with commits:

```bash
# Edit current commit:
git commit --amend

# Edit current commit and message.
	# You can "git add ." and include new
	# changes in the amended commit as well.
git commit --amend -m "New and improved commit message"

# To edit a previous commit message, use
# interactive rebase with option 'r' (reword).

# Undo a `git commit --amend`
git reset --soft HEAD@{1}
```

Interactive Rebase:
- Squash commits
- Edit commit messages

```bash
# Squash (combine) multiple commits
# into one.
git rebase -i

# Squash 5 commits.
git rebase -i HEAD~5

# Setting up commits for squashing.
# First commit:
git commit -m "First commit message (to preserve)."
# Each subsequent commit.
git commit --fixup HEAD~1
# Squash 5 fixup commits into one.
git rebase -i --autosquash HEAD~5
```

Comparing changes line-by-line:

```bash
# List all commits and IDs
git log

# Check your changes line-by-line with diff:
	# Checks last commit against unstaged changes.
	# If you do `git add .`, diff will show no changes.
	git diff
	# ALWAYS specify older commit first,
		# otherwise colors will be reversed.
	# You can use commit IDs or branch names (which
		# will specify the latest commit in that branch).
	git diff [commit 1 (older)] [commit 2 (newer)]
	
	# Examples:
		# If your local HEAD (master) is
		# a few commits ahead of origin/master:
	git diff origin/master master
	git diff origin/master HEAD
	git diff origin/HEAD master
	# See diff between previous and current commit
	git diff HEAD^1
	git diff HEAD^1 HEAD # equivalent
```