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
# Rebase new feature branch onto master
git rebase master new_feature
	# Alternatively:
	git checkout new_feature
	git rebase master

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
git branch -D branch_name # Ignore warnings.

# Rename a branch.
git branch -m <old_branch_name> <new_branch_name>

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

# Create a new branch from the current branch
# starting at a specific commit.
git checkout -b new-branch-name <commit-hash>

# Get branch releases/2023.1.1 in detached head state
git checkout origin/releases/2023.1.1

# Get remote branch releases/2023.1.1, and create a
# matching local branch to track the remote branch
# (i.e. NOT detached HEAD state).
git checkout releases/2023.1.1 # leave out "origin" (remote name)
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

Discard all untracked files and changes, including any cherry-picked commits:

This is how you can erase a cherry-pick you added to your branch for testing before the cherry-picked commit gets merged.

* **Untracked:** Unknown to git, never been added with `git add`
* **Unstaged:** Known to git, previously added with `git add`, but changed since then and changes not yet staged for next commit

```bash
git clean -d -f
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

# Show current commit info, with graph of
# number of lines changed for each file.
git show --stat
```

Interactive Rebase:

* Squash commits
* Edit commit messages

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

# Find a specific commit
git log 3a5ceg01 # abbreviated hash
git log 4c5g1g91d891f2d691ee13a9d9096c3ecfc0fb0f # full hash

# See git log history between two commits,
# exclusive of the older one, inclusive of
# the newer one.
git log [older commit]..[newer commit]
git log 3a5ceg01..a5c36e7g

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

Visualizing commit tree:

```bash
git log --graph --oneline --all

# tig is the best tool for this.
sudo apt install tig
tig
```

Git Bisect for triaging commits to find the root cause of a test failure:

```bash
# Binary search through commits between
# a known pass and known failure, running
# a test on each one to find the root cause.
git bisect [help|start|bad|good|new|old|terms|skip|next|reset|visualize|view|replay|log|run]

git bisect start # starts the process
# Start by marking the first known bad commit.
git bisect bad [commit hash]
# Next mark the last known good commit.
git bisect good [commit hash]
# Then bisect will start the search and
# checkout the first commit. Test the code, then:
git bisect [good|bad] # then it will move to the next commit
# When the process is complete, it will show
# the first bad commit. Then to finish, run:
git bisect reset
```
