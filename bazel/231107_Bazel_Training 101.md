# 231107 Bazel Training 101

Alex Eagle from Aspect.

## High level stuff

- Interoperable medium is files (compared to UNIX where bash commands interface with text streams, i.e. `cat | grep`)
- Continually updated transformation from source tree to `bazel-out` tree
- Bazel build files and extensions are written in Skylark
	- Stripped down version of Python
- Using `git` command in Bazel build file:
	- Dependencies downloaded don't get cached, so wind up downloading every time if using this
- All dependencies are pinned - you always pull the version of a dependency you want unless you explicitly change it
	- Integrity hashes are used to ensure correct dependency versions are pulled

## Bazelisk

- Download binary
- Rename it to "bazel"
- Move it into one of the binary folders (/usr/bin, ~/bin, etc.)
- Add that location to the PATH `export PATH=$PATH:/path/to/your/directory`

## 5 Phases of a Build

1. Configure
	1. Compile and run "Gazelle" binary
2. Fetching
	1. Download external resources required by requested targets
3. Loading
	1. Find build files and understand graph of them
	2. Pre-processor macros run here
	3. Result is dependency graph
	4. If you run loading phase on its own, it will do fetching because it needs that to do loading phase
4. Analysis
5. Execution
	1. This is part of `bazel build`
	2. `bazel run` builds, and then runs the binaries that result
	3. `bazel test` also builds, then runs tests

**To trigger all steps, just run `bazel build`**.

## Bazel Project Structure

```md
root/             # This is the "//" package
├── BUILD.bazel
├── ...
├── animations    # Here is the "//animations" package
│   ├── BUILD.bazel
│   ├── browser   # Here is "//animations/browser"
│   │   ├── BUILD.bazel
│   │   └── ...
|   |   ...
│   ├── src
│   │   └── index.ts  # This file is in the `animations` package, since there's no BUILD here
│   ├── test
│   │   ├── BUILD.bazel
│   │   └── ...
```

## Labels

```md
          ┌ package name ┐
          v              v
@angular//animations/utils:draw_circle
   ^                           ^
   |                           └ target
   └ repository name (optional)
```

## Label Shorthand

If the working directory is in the same workspace, `//animations/util:draw_circle`

- `//` means the root of that workspace.
- On the command line, labels can be relative to the working directory
- Each package has a default label, named after the package

You can usually use this shorthand to save typing. For example you could just `cd backend; bazel run devserver` rather than `bazel run //backend/devserver:devserver`.

Every package should have a nice default target, to save typing and make an ergonomic experience for developers interacting with Bazel in your project. You can use `alias` to introduce an indirection, for example if you'd like users to be able to `bazel run backend` from the repository root, then you'd add an `alias`:

backend/BUILD.bazel

```bazel
alias(
	name = "backend",   # the default target for the backend package
	actual = "//backend/devserver",
)
```

NOTE

Run `bazel help target-syntax`

## Bazel Rules

Anyone can create Bazel rules. They are not part of Bazel core.

It's basically an app-store / marketplace of rules that anyone can write, and you need to find the ones you need, load them in with `load()`, and use them.

`data` = runtime deps
`deps` = compile time deps

Use `run_binary` instead of `genrule`.

## Rules vs. Macros

Macros and rules both can't distinguish what they are pointing to (a single file, or a package built by Bazel).

Macros are like pre-processor / compile-time directives.

Macros not private. Once built, users can see inside macro, run specific parts of it, and will get error messages from inside macro.

### When to use a macro

- Wrap rule in macro to provide default values to a rule

`bazel query --output=build`

## Providers

Alternative way of passing information along action graph.

## Tests

Nothing can depend on a test.

