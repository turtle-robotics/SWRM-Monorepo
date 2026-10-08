# SWRM-Monorepo
This is a monorepository for the SWRM project. As a monorepo, it contains 
multiple subfolders that represent different components of the project, but the
git history is shared across all of them. This allows for an easy way to find 
the SWRM code in the Turtle organization while not cluttering the main Turtle
GitHub organization at the same time.

## Creating a new component
When a new component of the SWRM project needs to be created, a new subfolder
for that component should be created in the root of this repository. The 
subfolder should be named after the component. The subfolder should also be 
independent of other components, meaning that it should not share code or files
with other components; each component should be able to be built and ran 
independently of the other.

While the structure of each component is up to the developer, two files are
required to be present in each component's subfolder:
- `README.md`: It should contain a description of the component, how to build 
  it, how to run it, any dependencies it requires, and any other relevant 
  information.
- `.gitignore`: Each component should manage its own gitignore file to avoid
  accidentally ignoring files and patterns that may be needed by another 
  component.

Ideally, each component should also include a `docs` folder that contains any
generated documentation (use tools like Doxygen for C/C++ or Sphinx for Python)
and a `tests` folder that contains any unit tests for the component (use tools
like Google Test for C/C++ or PyTest for Python).

## Contributing
At the moment, for the sake of simplicity, CI pipelines are not set up to 
automatically test code on pushes and merges. However, new features and 
contributions should be made as a separate branch and should be tested locally
before submitting a Pull Request for merge. If multiple people are working on
the same component, request a review from another developer before merging to
ensure component quality and functionality.
