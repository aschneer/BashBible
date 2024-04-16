# Resource Management

Monitor system resources.

```bash
top
htop
```

If a process is killed, it returns a negative number. The number corresponds to the type of signal used to kill it.

POSIX signals:

```bash
man signal
man 7 signal
```

Signal numbers from `man 7 signal`:

```bash
   Standard signals
       Linux  supports  the standard signals listed below.  The second column of the table indi‐
       cates which standard (if any) specified the signal: "P1990" indicates that the signal  is
       described  in  the  original POSIX.1-1990 standard; "P2001" indicates that the signal was
       added in SUSv2 and POSIX.1-2001.

       Signal      Standard   Action   Comment
       ────────────────────────────────────────────────────────────────────────
       SIGABRT      P1990      Core    Abort signal from abort(3)
       SIGALRM      P1990      Term    Timer signal from alarm(2)
       SIGBUS       P2001      Core    Bus error (bad memory access)
       SIGCHLD      P1990      Ign     Child stopped or terminated
       SIGCLD         -        Ign     A synonym for SIGCHLD
       SIGCONT      P1990      Cont    Continue if stopped
       SIGEMT         -        Term    Emulator trap
       SIGFPE       P1990      Core    Floating-point exception
       SIGHUP       P1990      Term    Hangup detected on controlling terminal
                                       or death of controlling process
       SIGILL       P1990      Core    Illegal Instruction
       SIGINFO        -                A synonym for SIGPWR
       SIGINT       P1990      Term    Interrupt from keyboard
       SIGIO          -        Term    I/O now possible (4.2BSD)
       SIGIOT         -        Core    IOT trap. A synonym for SIGABRT
       SIGKILL      P1990      Term    Kill signal
       SIGLOST        -        Term    File lock lost (unused)
       SIGPIPE      P1990      Term    Broken pipe: write to pipe with no
                                       readers; see pipe(7)
       SIGPOLL      P2001      Term    Pollable event (Sys V);
                                       synonym for SIGIO
       SIGPROF      P2001      Term    Profiling timer expired
       SIGPWR         -        Term    Power failure (System V)
       SIGQUIT      P1990      Core    Quit from keyboard
       SIGSEGV      P1990      Core    Invalid memory reference
       SIGSTKFLT      -        Term    Stack fault on coprocessor (unused)
       SIGSTOP      P1990      Stop    Stop process
       SIGTSTP      P1990      Stop    Stop typed at terminal
       SIGSYS       P2001      Core    Bad system call (SVr4);
                                       see also seccomp(2)
       SIGTERM      P1990      Term    Termination signal
       SIGTRAP      P2001      Core    Trace/breakpoint trap
       SIGTTIN      P1990      Stop    Terminal input for background process
       SIGTTOU      P1990      Stop    Terminal output for background process
       SIGUNUSED      -        Core    Synonymous with SIGSYS
       SIGURG       P2001      Ign     Urgent condition on socket (4.2BSD)
       SIGUSR1      P1990      Term    User-defined signal 1
       SIGUSR2      P1990      Term    User-defined signal 2
       SIGVTALRM    P2001      Term    Virtual alarm clock (4.2BSD)
       SIGXCPU      P2001      Core    CPU time limit exceeded (4.2BSD);
                                       see setrlimit(2)
       SIGXFSZ      P2001      Core    File size limit exceeded (4.2BSD);
                                       see setrlimit(2)
       SIGWINCH       -        Ign     Window resize signal (4.3BSD, Sun)

       The signals SIGKILL and SIGSTOP cannot be caught, blocked, or ignored.

       Up to and including Linux 2.2, the default behavior for SIGSYS, SIGXCPU, SIGXFSZ, and (on
       architectures  other  than SPARC and MIPS) SIGBUS was to terminate the process (without a
       core dump).  (On some other UNIX systems the default action for SIGXCPU and SIGXFSZ is to
       terminate  the  process without a core dump.)  Linux 2.4 conforms to the POSIX.1-2001 re‐
       quirements for these signals, terminating the process with a core dump.

       SIGEMT is not specified in POSIX.1-2001, but nevertheless appears on most other UNIX sys‐
       tems, where its default action is typically to terminate the process with a core dump.

       SIGPWR  (which is not specified in POSIX.1-2001) is typically ignored by default on those
       other UNIX systems where it appears.

       SIGIO (which is not specified in POSIX.1-2001) is ignored by  default  on  several  other
       UNIX systems.
```

Signal numbering from `man 7 signal`:

```bash
   Signal numbering for standard signals
       The numeric value for each signal is given in the table below.  As shown  in  the  table,
       many signals have different numeric values on different architectures.  The first numeric
       value in each table row shows the signal number on x86, ARM,  and  most  other  architec‐
       tures;  the  second  value is for Alpha and SPARC; the third is for MIPS; and the last is
       for PARISC.  A dash (-) denotes that a signal is absent on  the  corresponding  architec‐
       ture.

       Signal        x86/ARM     Alpha/   MIPS   PARISC   Notes
                   most others   SPARC
       ─────────────────────────────────────────────────────────────────
       SIGHUP           1           1       1       1
       SIGINT           2           2       2       2
       SIGQUIT          3           3       3       3
       SIGILL           4           4       4       4
       SIGTRAP          5           5       5       5
       SIGABRT          6           6       6       6
       SIGIOT           6           6       6       6
       SIGBUS           7          10      10      10
       SIGEMT           -           7       7      -
       SIGFPE           8           8       8       8
       SIGKILL          9           9       9       9
       SIGUSR1         10          30      16      16
       SIGSEGV         11          11      11      11
       SIGUSR2         12          31      17      17
       SIGPIPE         13          13      13      13
       SIGALRM         14          14      14      14
       SIGTERM         15          15      15      15
       SIGSTKFLT       16          -       -        7
       SIGCHLD         17          20      18      18
       SIGCLD           -          -       18      -
       SIGCONT         18          19      25      26
       SIGSTOP         19          17      23      24
       SIGTSTP         20          18      24      25
       SIGTTIN         21          21      26      27
       SIGTTOU         22          22      27      28
       SIGURG          23          16      21      29
       SIGXCPU         24          24      30      12
       SIGXFSZ         25          25      31      30
       SIGVTALRM       26          26      28      20
       SIGPROF         27          27      29      21
       SIGWINCH        28          28      20      23
       SIGIO           29          23      22      22
       SIGPOLL                                            Same as SIGIO
       SIGPWR          30         29/-     19      19
       SIGINFO          -         29/-     -       -
       SIGLOST          -         -/29     -       -
       SIGSYS          31          12      12      31
       SIGUNUSED       31          -       -       31

       Note the following:

       *  Where defined, SIGUNUSED is synonymous with SIGSYS.  Since glibc 2.26, SIGUNUSED is no
          longer defined on any architecture.

       *  Signal 29 is SIGINFO/SIGPWR (synonyms for the same value)  on  Alpha  but  SIGLOST  on
          SPARC.
```