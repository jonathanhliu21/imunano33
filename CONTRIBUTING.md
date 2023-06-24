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

