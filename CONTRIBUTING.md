# Contributing

Issues can be submitted in the GitHub issue tracker. For bugs and enhancements, PRs are welcome.

## Creating a Pull Request

- Fork the repository.
- Start writing code and committing.
- Create a pull request from the fork into the `main` branch.
- In the pull request, please reference the issue that the PR is addressing.
- Make sure all checks pass.

## Testing

- Create a build folder and `cd` into it.
- Run

```text
$ cmake .. -DIMUNANO33_BUILD_TEST=ON
```

- Run `make`.
- Run `ctest` to run the test suite.

## Documentation

To build documentation, you need doxygen and sphinx.

- Create a build folder and `cd` into it.
- Run

```text
$ cmake .. -DIMUNANO33_BUILD_DOC=ON
```

- Run `make doxygen`.
- The index file should be in `./doc/html/index.html`

## Linting and Formatting

To lint your code, you need `clang-tidy` and `clang-format`, meaning that you need to install LLVM onto your system.

- Create a build folder and `cd` into it.
- Run

```text
$ cmake .. -DSVECTOR_BUILD_EXAMPLE=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra"
```

- Run `make`.
- Go to the root directory of the repository.

```text
$ cd ..
```

- Run

```text
$ make -C scripts/ lint
```

- To format your C++ code (do not run the command for any other type of file), run

```text
$ clang-format -i path/to/file/you/just/changed.hpp
```
