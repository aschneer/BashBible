# WSL - Windows Subsystem for Linux

From Bard.

**WSL Management Commands:**

**Installation and Setup:**

- **`wsl --install`**: Installs the default Linux distribution (Ubuntu).
- **`wsl --install -d <Distribution Name>`**: Installs a specific distribution from the list of available options.
- **`wsl --list --online`**: Displays a list of available Linux distributions for installation.
- **`wsl --set-default <Distribution Name>`**: Sets the default WSL distribution to use when launching WSL without specifying a distribution.
- **`wsl --set-version <Distribution Name> <Version Number>`**: Sets the WSL version (1 or 2) for a specific distribution.

**Managing Distributions:**

- **`wsl -l -v`**: Lists installed distributions and their WSL versions.
- **`wsl --unregister <Distribution Name>`**: Unregisters and removes a distribution.
- **`wsl --export <Distribution Name> <FileName>`**: Exports a distribution as a tar file.
- **`wsl --import <Distribution Name> <InstallLocation> <FileName>`**: Imports a distribution from a tar file.

**Running and Interacting:**

- **`wsl`**: Launches the default WSL distribution.
- **`wsl <Distribution Name>`**: Launches a specific WSL distribution.
- **`wsl -u <Username>`**: Launches a distribution as a specific user.
- **`wsl --exec <Command>`**: Executes a Linux command without launching a distribution shell.

**Additional Options:**

- **`wsl --shutdown`**: Shuts down all running WSL distributions.
- **`wsl --status`**: Shows the status of running WSL distributions.
- **`wsl --help`**: Displays help information for WSL commands.