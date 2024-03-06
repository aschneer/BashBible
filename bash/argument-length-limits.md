Various configuration settings for Bash.

# Command Line Limit

## References

- [Why am I getting [Errno 7] Argument list too long and OSError: [Errno 24] Too many open files when using mrjob v0.4.4?](https://stackoverflow.com/questions/30650841/why-am-i-getting-errno-7-argument-list-too-long-and-oserror-errno-24-too-ma)
- ["Argument list too long": How do I deal with it, without changing my command?](https://unix.stackexchange.com/questions/45583/argument-list-too-long-how-do-i-deal-with-it-without-changing-my-command)

## Command Argument List Length Limit

Get the current command argument list limit.

```bash
getconf ARG_MAX
```

This command queries the value of `MAX_ARG_STRLEN` in `/usr/include/linux/binfmts.h` kernel source code.

The linux kernel source code is usually located at `/usr/include/linux` but if not, you can install it with:

```bash
sudo apt-get install linux-headers-$(uname -r)
```

Get current stack space limit (space available for the stack).

```bash
ulimit -s # result in KB
	# Default is 8192 KB

# Change it.
ulimit -s 65536
```

## From ChatGPT

<u>**Is there a way to configure the max arg length for execve() system call on the fly in linux without recompliing the kernel?**</u>

The maximum argument length for the `execve()` system call in Linux is determined by the `MAX_ARG_STRLEN` constant, which is defined in the Linux kernel source code. This value is typically set to 32 pages (or 128 KB), and it is used to limit the length of the command line arguments passed to an executable.

As of my last knowledge update in September 2021, adjusting `MAX_ARG_STRLEN` on the fly without recompiling the kernel is not directly possible. This constant is part of the kernel configuration, and changes to it would require a recompilation of the kernel.

However, there are alternative approaches to dealing with long command lines without modifying the kernel source code:

1. **Environment Variables**: Instead of passing lengthy information via command-line arguments, consider using environment variables. The environment can hold more data, and it doesn't have a predefined length limit like command-line arguments.
2. **Input from Files or Other IPC Mechanisms**: If you need to pass a large amount of data, consider reading it from a file or using inter-process communication (IPC) mechanisms like pipes or sockets.
3. **Use of Configuration Files**: If you have a configuration that is too large for command-line arguments, consider using configuration files. Your program can read the required information from the file.
4. **Dynamic Memory Allocation**: If you are dealing with dynamically changing data sizes, you can allocate memory dynamically and manage it during the execution of your program.

Remember, these alternatives might require changes to your program's code or behavior. The choice depends on the specific requirements and design of your application. Always consider the security implications and design your system accordingly.

---

<u>**What does this do?**</u> `getconf ARG_MAX`

The `getconf ARG_MAX` command in Linux is used to retrieve the maximum length (in bytes) of the arguments that can be passed to a new process when using the `exec` family of functions, such as `execve()`.

When you run `getconf ARG_MAX`, it queries the system configuration and prints the maximum allowed argument length in bytes. This value represents the maximum size of the command line that can be passed to a new process.

Here's an example of how you might use it:

```bash
$ getconf ARG_MAX
```

The output will be a numerical value representing the maximum allowed size of command-line arguments in bytes.

Note that this value is specific to the system's configuration and kernel settings. It is derived from the `MAX_ARG_STRLEN` constant in the Linux kernel source code and is typically set to 32 pages (128 KB), but it can vary between systems. The value returned by `getconf ARG_MAX` provides a way to query this limit without needing to inspect kernel source code or kernel configuration files.

---

<u>**What is the relationship between `ARG_MAX`/`MAX_ARG_STRLEN` and `ulimit -s`?**</u>

`ARG_MAX` (or `MAX_ARG_STRLEN`) and `ulimit -s` are related to process resource limits in a Unix-like operating system, such as Linux. However, they are associated with different aspects of process execution.

1. **`ARG_MAX` (or `MAX_ARG_STRLEN`)**:
   - `ARG_MAX` (or `MAX_ARG_STRLEN`) represents the maximum length (in bytes) of the arguments that can be passed to a new process when using the `exec` family of functions, such as `execve()`.
   - It is a system-wide constant defined in the Linux kernel source code.
   - You can query it using the `getconf ARG_MAX` command.
   - It affects the maximum length of the command line that can be passed to a new process.
2. **`ulimit -s`**:
   - `ulimit -s` sets or displays the stack size limit for a shell and its child processes.
   - The stack size limit represents the maximum size of the call stack for a process.
   - It is expressed in kilobytes (KB).
   - The `ulimit -s` command is used to query or modify the stack size limit.

While these two parameters are related to process limits, they serve different purposes:
- `ARG_MAX` is about the maximum size of command-line arguments passed to a new process, affecting the `exec` system calls.
- `ulimit -s` is about the stack size limit for a process, affecting the size of the call stack for functions and variables.

Adjusting the stack size limit with `ulimit -s` doesn't directly impact the maximum length of command-line arguments (`ARG_MAX`). However, both parameters are important in understanding and managing the resource limits of a process. Keep in mind that changing these limits may have implications for system performance and application behavior, and adjustments should be made judiciously based on the requirements of your applications.