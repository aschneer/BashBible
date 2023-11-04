# Code Commit Process 1

Process of making code changes, committing them, getting them reviewed, and getting them merged into master for a Bazel project.

1. Clone the repo
2. Optionally create a new branch for your changes, or just edit master
	```bash
	git branch my_changes
	git checkout my_changes
	```
3. Make code changes
4. Add changed files
	```bash
	git add .
	```
5. Make first commit (locally)
	```bash
	git commit -m "message"
	```
6. Make as many subsequent commits as you like
	```bash
	# For each one, use:
	git commit --fixup HEAD~1
	```
7. Test your changes
	```bash
	cd [directory with test modules]
	bazel test -c opt ...
	```
8. When your changes are done, run a formatting check and fix
	```bash
	bazel run //:buildifier
	bazel run //:clang_format
	```
9. You are now ready to merge. Use interactive rebase to squash them all into one commit with the correct commit message.
	```bash
	# 'n' is the number of commits to squash
	git rebase -i --autosquash HEAD~n
	
	# You can amend the current commit
	# message afterward if needed.
	git commit --amend
	git commit --amend -m "New message"
	
	# To amend a previous commit message
	git rebase -i
	# Use option 'r' (reword) in the instruction file.
	```
10. Fetch and pull origin/master
	```bash
	git checkout master
	git fetch <origin> <master>
	git pull <origin> <master>
	```
11. Rebase your new branch onto local master (if you have one), and rebase local master onto origin/master
	```bash
	git rebase master my_changes # if relevant
	git rebase origin/master master
	# Resolve any conflicts that arise.
	
	# You can also do it in the opposite order;
	# Rebase your new branch onto local master,
	# then rebase local master onto origin/master
	# as you pull.
	git rebase master my_changes
	git fetch
	git pull --rebase
	# Resolve any conflicts that arise.
	```
12. Push your commit for code review
	```bash
	git push origin HEAD:refs/for/master
	```
