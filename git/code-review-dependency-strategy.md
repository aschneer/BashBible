# Code Review Dependency Strategy

From Bard.

**Question:**

Let's say I want to work on new code that depends on a commit I just sent in for code review. What's the best way to do this so that if the code review results in changes or is not approved, I can easily move the new code back to master, removing the dependency on the rejected commit?

**Answer:**

**Here's a strategy to manage code dependencies in your scenario, ensuring flexibility and easy removal if needed:**

1. **Create a Feature Branch:**
	- Branch off `master` to isolate your new code:
		- `git checkout -b my-new-feature master`
	- This creates a separate branch for development, independent of the code review process.
2. **Develop on the Feature Branch:**
	- Implement your new code on `my-new-feature`.
    - Commit changes as needed.
3. **Address Dependencies:**
	- If your code relies on the unapproved commit:
		- **Option 1:** Temporarily cherry-pick the necessary changes onto `my-new-feature`:
			- `git cherry-pick <commit-hash>`
		- **Option 2:** Use a feature flag to conditionally enable the dependent code, waiting for approval before fully integrating it.
4. **Review and Approval:**
	- Submit `my-new-feature` for code review.
	- If approved, merge it into `master`.
5. **Handling Rejection:**
	- If the unapproved commit is rejected or requires changes:
		- Revert or modify the cherry-pick on `my-new-feature`.
		- If using feature flags, disable the dependent code.
		- Update your new code accordingly.
		- Resubmit for review.
6. **Rebase for Updates:**
	- If the unapproved commit is updated:
		- Rebase `my-new-feature` onto the updated `master`:
			- `git rebase master my-new-feature`
		- Resolve any conflicts.
		- Resubmit for review.

**Key Benefits:**

- **Isolation:** Feature branches prevent merge conflicts and protect `master`.
- **Flexibility:** Cherry-picking or feature flags manage dependencies effectively.
- **Control:** Rebase keeps `my-new-feature` up-to-date with `master`.
- **Maintainability:** Clear separation of concerns for easier code management.

**Additional Tips:**

- Communicate with your team about dependencies and strategies.
- Consider using feature flags for long-term management of experimental or risky features.
- Keep branches up-to-date with `master` to minimize merge conflicts.
- Use clear and descriptive commit messages to track changes and dependencies.

By following these practices, you can confidently develop code that depends on unapproved commits while ensuring easy management and adaptability based on review outcomes.