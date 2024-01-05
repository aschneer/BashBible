# Code Commit Process 2 - Simple

Process of making code changes, committing them, getting them reviewed, and getting them merged into master, using Gerrit and Bazel.

**Big picture:** \
You want all your changes for a project/feature to be in a single commit. Each commit gets code-reviewed and merged separately. So if you need to make changes after initially committing, you use `git commit --amend` and push the same commit again to Gerrit, which creates a new patchset in the same code review.

Here's a simple workflow to follow:

1. Clone the repo, or pull the latest version
	```bash
	git clone [repo URL]
	# or...
	git pull <origin> <master>
	```
2. Make code changes
3. Stage changed files
	```bash
	git add .
	# If you made a mistake and want to unstage...
	git restore --staged .
	```
4. Commit changes (locally)
	```bash
	git commit -m "message"
	```
5. If you need to make additional changes after committing, make the changes and then amend the initial commit.
	```bash
	git commit --amend
	
	# Do not make additional commits on top
	# of the first one, otherwise you'll need
	# to squash them later before pushing
	# to Gerrit. Just keep amending the
	# same commit.
	```
6. Test your changes
	```bash
	# Go to a directory with test modules
	cd [directory with test modules]
	# Run all tests in the current directory (recursive)
	bazel test -c opt ...
	# Or, run a specific test. Example:
	bazel test -c opt //my/folder/directory:test_1
	```
7. When your changes are done, run formatting checks. This will edit files automatically.
	```bash
	bazel run //:buildifier
	bazel run //:clang_format
	./any_other_format_checker.sh
	```
8. If any changes were made by the format checkers, you will need to do a final commit amend to capture them.
	```bash
	# Check if any changes were made
	# (they will be listed in red).
	git status
	# If there are no changes, skip
	# the rest of this step.
	
	# Otherwise continue:
	
	# If any changes exist, stage them
	git add .
	# Amend initial commit
	git commit --amend
	
	# You can amend the current commit
	# message again afterward if needed.
	git commit --amend # it will prompt you
	git commit --amend -m "New message"
	```
9. Push your commit to Gerrit for code review
	```bash
	git push origin HEAD:refs/for/master
	
	# This will push your commit into a staging
	# area with other un-merged commits.
	# It will only merge with master once approved.
	
	# You can also push to an unreviewed branch
	# if you're not ready for code review and
	# just want to back up your commit to the cloud.
	git push origin HEAD:unreviewed/your-name/my_branch
	# You can then go into Gerrit to delete
	# this branch when you no longer need it.
	#
	# To pull down an unreviewed branch:
	# Creates detached head state.
	git checkout origin/unreviewed/your-name/my_remote_branch
	# Copies detached head state into new branch with good state.
	git switch -c new_branch_name
	
	# Worse way to do the same thing; requires rebasing.
	git pull origin refs/heads/unreviewed/your-name/my_remote_branch
	git pull origin unreviewed/your-name/my_remote_branch
	```
10. Add patches to existing commit in Gerrit
	```bash
	# Make more changes, then...
	git add .
	git commit --amend # update commit message when prompted
	git push origin HEAD:refs/for/master # same commit ID will become patchset
	
	# If you moved away from the commit in your
	# local repo and need to go back to it
	# (or you just want to be safe):
		# In Gerrit, open the commit, click 'download' and
		# run the provided git cherry-pick command. This will
		# pull down your commit for further editing.
		# Example:
		git fetch ssh://address && git cherry-pick FETCH_HEAD
		# Make changes, then...
		git add .
		git commit --amend
		git push origin HEAD:refs/for/master
	
	# As long as you continue pushing the same commit ID,
	# it will get added to the existing code review
	# as a new patchset. Pushing a different commit ID
	# will create a new code-review in Gerrit.
	```
11. Once code review is approved and merged, wipe local changes and reset local master to match origin/master (start fresh for next project).
	```bash
	git reset --hard origin/master
	```