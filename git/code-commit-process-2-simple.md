# Code Commit Process 2 - Simple

Process of making code changes, committing them, getting them reviewed, and getting them merged into master.

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
5. Commit additional changes (locally)
	```bash
	git commit --amend
	
	# Do not make additional commits on top
	# of the first one, otherwise you'll need
	# to squash them. Just keep amending the
	# same commit.
	```
6. Test your changes
	```bash
	cd [directory with test modules]
	# Run all tests in this directory
	bazel test -c opt ...
	# Or, run a specific test
	bazel test -c opt //brt/ingest/directory:test_name
	```
7. When your changes are done, run a formatting check and fix (automatic)
	```bash
	bazel run //:buildifier
	bazel run //:clang_format
	```
8. Do a final commit amend to capture any formatting changes
	```bash
	git commit --amend
	
	# You can amend the current commit
	# message again afterward if needed.
	git commit --amend
	git commit --amend -m "New message"
	```
9. Push your commit to the cloud for code review
	```bash
	git push origin HEAD:refs/for/master
	
	# This will push your commit into a staging
	# area with other un-merged commits.
	# It will only merge with master when approved.
	```
10. Add patches to existing commit
	```bash
	# Make more changes, then...
	git commit --amend # update commit message when prompted
	git push origin HEAD:refs/for/master # same commit ID will become patch
	
	# If you moved away from the commit in your
	# local repo and need to go back to it
	# (or you just want to be safe):
		# In cloud app, open the commit, click 'download' and
		# run the provided git cherry-pick command. This will
		# pull down your commit for further editing.
		# Example:
		git fetch ssh://blah_blah_blah && git cherry-pick FETCH_HEAD
		# Make changes, then...
		git commit --amend
		git push origin HEAD:refs/for/master
	```