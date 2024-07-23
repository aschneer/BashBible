---
layout:
  title:
    visible: true
  description:
    visible: true
  tableOfContents:
    visible: true
  outline:
    visible: true
  pagination:
    visible: true
---

# Git Reference

## Branches

You can have any branch structure you want. Branches are stored in a directory structure. The files of the repo exist in the folder for a given branch.

In this example, `my_fancy_branch` is a branch in a folder called `andy`, which is in a folder called `experimental`. This structure organizes branches by their purpose. Any branch can be merged into any other branch regardless of this folder structure; it's just a way of organizing branches.

```txt
my_repo/
├── experimental/
│   └── andy/
│      └── my_fancy_branch/
└── ...
```

Gerrit has special namespaces for specific purposes. The following is for pushing branches for code review.

```bash
my_repo/refs/for/
```

Under this, you would put your branch according to whatever folder structure your team uses.

```bash
my_repo/refs/for/
├── experimental/my.name/my_fancy_branch
```

### Working with Branches

```bash
# Current latest local commit.
HEAD
# Current latest remote commit.
origin/HEAD
```

List all branches:

```bash
git branch
```

Create a new branch as a copy of the current branch:

```bash
# Just create it
git branch branch_name

# Switch to it at the same time
git switch -c new_branch_name
```

Move to a different branch:

```bash
git checkout branch_name
```

Delete a branch:

```bash
git branch -d branch_name
git branch -D branch_name # Ignore warnings.
```

Rename a branch:

```bash
git branch -m <old_branch_name> <new_branch_name>
```

Get local copy of a remote branch:

```bash
# Detached head state
git checkout origin/master
git checkout origin/releases/2024.7.19

# Tracking remote branch
git checkout master
git checkout releases/2024.7.19
```

Duplicate an existing branch and checkout the new branch copy:

```bash
git switch -c feature/branch2 feature/branch1
git checkout -b feature/branch2 feature/branch1
```

Create a new branch from the current branch starting at a specific commit:

```bash
git checkout -b new-branch-name <commit-hash>
```

### Pushing to Remote Branches

```bash
# Push the HEAD to a branch called "my_fancy_branch"
# in the git namespace "my/namespace/" in the branch
# directory "experimental/andy/".
git push origin HEAD:my/namespace/experimental/andy/my_fancy_branch
# Or push to master branch in "my/namespace/"
git push origin HEAD:my/namespace/master
```

Push from specific local branch to specific remote branch:

```bash
git push [remote] [local branch]:[remote branch]
git push origin local_branch:remote_branch
```

### Rebasing

Pull latest master and rebase your local master on top of it.

```bash
git pull --rebase
```

Rebase a new feature branch onto master:

```bash
git checkout new_feature
git rebase master

# or

git rebase master new_feature
```

Rebase onto a remote branch:

```bash
git checkout new_feature
git rebase origin/master
```

### Pull

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

Move all committed changes back to staging area (keep them staged and tracked, but not committed):

```bash
git reset --soft HEAD~1
```

Unstage files, but keep them tracked:

```bash
git reset --soft HEAD~

# Then untrack them if desired...

git reset
```

Unstage and untrack files at once:

```bash
# All files
git reset HEAD
git restore --staged . # equivalent
git reset <commit_hash> # equivalent

# Specific file
git reset HEAD <file>
git restore --staged <file> # equivalent
```

Revert all unstaged changes:

```bash
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

## Working with Commits

```bash
# Edit current commit:
git commit --amend

# Edit current commit and message.
	# You can "git add ." and include new
	# changes in the amended commit as well.
git commit --amend -m "New and improved commit message"

# To edit a previous commit message, use
# interactive rebase with option 'r' (reword).

# Show current commit info, with graph of
# number of lines changed for each file.
git show --stat
```

### Interactive Rebase

Useful for:

* Squash commits
* Edit commit messages
* Edit individual commits in a chain of commits

It is important to understand how an interactive rebase works.

The way it works is it goes back in the commit history to where you tell it to start. From that point, it rebuilds each commit in the chain one at a time with the staged changes at each step.

It follows the basic workflow:

**Stage Changes -> Commit -> Move to next commit**

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

#### Edit a Specific Commit in a Chain

To edit a specific commit in a chain without affecting the other commits, do as follows:

```bash
tig # look at commit history, decide where to start rebase
git rebase -i HEAD~x # "x" is number of commits back to start
```

Make changes to the commit chosen. When changes are made:

```bash
git add .
git commit --amend
git rebase --continue
```

**IMPORTANT:** If merge conflicts are found:

1. Resolve them
2. Run `git rebase --continue`

{% hint style="warning" %}
**DO NOT** `git commit --amend` after resolving merge conflicts. Resolving (in VSCode) will automatically stage the conflict fixes. Keep the fixes in the staging area.

Amending the conflict fixes will combine them with the current commit in the rebase process, not the commit that caused the conflict. The effect will be that commits will be squashed and will disappear from the chain.

Simply fix each merge conflict, then run `git rebase --continue`.
{% endhint %}

## Comparing changes line-by-line

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

### Split Large Commit into Smaller Commits

#### Commit with Changes to Existing Files

This method uses `git add -p` to selectively stage hunks of each file and commit them separately.

```bash
git checkout branch_with_large_commit
git reset --soft HEAD~ # or HEAD~1, HEAD^, HEAD^1

# Changes will now be staged but uncommitted

git reset # sends back to tracked but unstaged
git add -p <file> # and go hunk by hunk
# Use "s" command in `git add -p` to split up hunks further
```

{% hint style="info" %}
Another command that is similar to `git add -p` is `git add -i` (interactive).
{% endhint %}

If the large commit contains new files, the above method won't work. When you reset changes from the staging area, instead of going to tracked but unstaged, they will go to fully untracked.

There is no way to get them to be tracked but unstaged, so git never sees them as "changes" and will not let you do `git add -p`. It will just say "no changes" when you try.

#### Commit with New Files

Instead, we create a new branch for our new commit chain. Then we reach over to the branch with the large commit and pull in hunks of changes one at a time.

```bash
git checkout master
git switch -c new_branch
# Grab files one at a time from large commit.
git checkout large_commit_branch -p -- path/to/file
```

Select `y` to add the hunk, or `e` to edit it in place (break it up further).

Instead of using `e`, you can also use `y` to add the whole thing, then go into the file manually, delete parts you don't want, and `git add .` to stage those deletions.

You don't really need to use `checkout -p` at all in this case. You can just do all the hunk selection from the text editor directly.

```bash
git checkout large_commit_branch -- path/to/file
# edit files in text editor.
git add .
git commit -m "message"
```

Another alternate method is to try going in the other direction; unstaging hunks of changes that you want to remove from the large commit.

```bash
git checkout branch_with_large_commit
git reset -p <file>
```

## Reflog

This is git's undo history.

```bash
git reflog
```

Command history will be listed in reverse chronological order.

To go back to a previous state, find the HEAD pointer number corresponding to the desired state. The form will be `HEAD@{1}`, `HEAD@{2}`, etc. chronologically through the history.

Use git reset to go back to that state.

```bash
git reset HEAD@{x} # x = place in reflog to revert to
```

Example: undo the last `git commit --amend`:

```bash
git reset --soft HEAD@{1}
```
