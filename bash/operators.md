# Operators

## Logical Operators

```bash
# Logical Operators:
==            # Test for exact equality
-lt           # Less than
-ge           # Greater than or equal to.
-ne [or] !=   # Not equal to.
-z $string    # Checks if $string is a null string.
```

## Boolean Operators

```bash
# Boolean Operators:
||    # OR
&&    # AND
```

## Command Operators

> The `||` command operator in Bash is called the "logical OR" operator. It is used to execute a command only if the preceding command fails or returns a non-zero exit status.

<pre class="language-bash"><code class="lang-bash"><strong>command1 || command2    # Command error handling
</strong></code></pre>

From Groq:

> Here, `command1` is the command that is executed first. If `command1` fails (i.e., returns a non-zero exit status), then `command2` is executed. If `command1` succeeds (i.e., returns a zero exit status), then `command2` is not executed.
