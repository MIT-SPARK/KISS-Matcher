# Contributing to KISS-Matcher

Thank you for your interest in contributing to KISS-Matcher! This document provides guidelines and information for contributors.

## Development Setup

### Prerequisites

- **C++**: CMake 3.24+, C++17 compiler, Ninja (recommended)
- **Python**: Python 3.8+ for Python bindings
- **ROS2**: Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04) for ROS integration
- **System Dependencies**: Eigen3, TBB, lz4, PCL (for examples), GTSAM (for ROS)

### Local Development

1. **Clone the repository:**

   ```bash
   git clone https://github.com/MIT-SPARK/KISS-Matcher.git
   cd KISS-Matcher
   ```

1. **Install pre-commit hooks:**

   ```bash
   pip install pre-commit
   pre-commit install
   ```

1. **Build C++ core:**

   ```bash
   cd cpp/kiss_matcher
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
   cmake --build build --parallel
   ```

1. **Build Python bindings (optional):**

   ```bash
   cd python
   pip install -e . -v
   ```

1. **Build ROS2 wrapper (optional):**

   ```bash
   # Setup ROS2 workspace
   mkdir -p ~/ros2_ws/src
   cp -r . ~/ros2_ws/src/kiss_matcher
   cd ~/ros2_ws
   colcon build --packages-select kiss_matcher kiss_matcher_ros
   ```

## How CI Runs

Our Continuous Integration (CI) system runs comprehensive tests across multiple platforms and components:

### Workflow Overview

#### 1. **C++ Core Build** (`.github/workflows/ci-cpp.yml`)

- **Platforms**: Ubuntu 22.04/24.04, macOS-14, Windows-latest
- **Compilers**: GCC-11/13, Clang, MSVC
- **Features**:
  - Uses ccache for build acceleration
  - Builds both core library and examples
  - Exports `compile_commands.json` for development
  - Uploads build artifacts and test reports

**Reproduce locally:**

```bash
# Ubuntu/macOS
cmake -S cpp/kiss_matcher -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache \
  -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
cmake --build build --parallel

# Windows
cmake -S cpp/kiss_matcher -B build -G "Visual Studio 17 2022" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
```

#### 2. **Python Wheels** (`.github/workflows/ci-python.yml`)

- **Platforms**: Linux, macOS, Windows
- **Python versions**: 3.8-3.13
- **Features**:
  - Uses `cibuildwheel` for cross-platform wheel building
  - Tests import and basic functionality
  - Publishes to TestPyPI on main branch

**Reproduce locally:**

```bash
# Install build dependencies
pip install scikit-build-core pybind11 numpy open3d

# Build wheel
cd python
pip wheel . -v

# Test installation
pip install dist/*.whl
python -c "import kiss_matcher; print('Success!')"
```

#### 3. **ROS2 Integration** (`.github/workflows/ci-ros2.yml`)

- **ROS Distros**: Humble (22.04), Jazzy (24.04)
- **Features**:
  - Uses official ROS Docker containers
  - Builds core library first, then ROS wrapper
  - Runs colcon tests and basic integration checks

**Reproduce locally:**

```bash
# Setup ROS2 environment
source /opt/ros/humble/setup.bash  # or jazzy

# Create workspace
mkdir -p ~/ros2_ws/src
cp -r . ~/ros2_ws/src/kiss_matcher
cd ~/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -y

# Build
colcon build --packages-select kiss_matcher kiss_matcher_ros
source install/setup.bash

# Test
colcon test --packages-select kiss_matcher kiss_matcher_ros
```

#### 4. **Linting and Formatting** (`.github/workflows/lint.yml`)

- **Tools**: pre-commit hooks, clang-format, cmake-format, black, isort, flake8, cpplint
- **Coverage**: C++, Python, CMake, YAML, Markdown files
- **Security**: Trivy vulnerability scanning

**Reproduce locally:**

```bash
# Run all pre-commit hooks
pre-commit run --all-files

# Individual tools
clang-format --dry-run --Werror cpp/**/*.{cpp,hpp}
black --check python/
cmake-format --check CMakeLists.txt
```

### Caching Strategy

CI uses several caching mechanisms for speed:

- **ccache**: C++ compilation cache (Linux/macOS)
- **pip cache**: Python package cache
- **pre-commit cache**: Pre-commit hook environments
- **Actions cache**: GitHub Actions cache for dependencies

### Artifacts Generated

- **C++ builds**: Binaries, libraries, compile_commands.json
- **Python wheels**: Cross-platform .whl files
- **ROS2 builds**: Built packages and test results
- **Test reports**: Coverage and test result files

## Code Style and Standards

### C++

- **Standard**: C++17
- **Formatting**: clang-format (see `.clang-format`)
- **Linting**: cpplint with custom filters
- **Naming**: Use descriptive names, follow existing patterns

### Python

- **Formatting**: black (line length 88)
- **Import sorting**: isort
- **Linting**: flake8 (E501 ignored for black compatibility)
- **Type hints**: Encouraged for new code

### CMake

- **Formatting**: cmake-format
- **Style**: Use modern CMake (3.24+) patterns

### General

- **Documentation**: Update relevant docs for new features
- **Tests**: Add tests for new functionality when possible
- **Commit messages**: Clear, descriptive messages
- **Pre-commit**: All hooks must pass before committing

## Testing

### C++ Tests

Currently manual testing is performed. Future work may include:

- Unit tests with Google Test
- Integration tests with test data
- Performance benchmarks

### Python Tests

- Import tests verify bindings work correctly
- Example scripts serve as integration tests

### ROS2 Tests

- colcon test framework
- Basic functionality verification
- Launch file validation

## Submitting Changes

1. **Fork** the repository
1. **Create** a feature branch: `git checkout -b feature/amazing-feature`
1. **Commit** your changes: `git commit -m 'Add amazing feature'`
1. **Push** to the branch: `git push origin feature/amazing-feature`
1. **Open** a Pull Request

### Pull Request Guidelines

- Ensure all CI checks pass
- Include tests for new features
- Update documentation as needed
- Link to related issues
- Provide clear description of changes

### CI Troubleshooting

**Common CI failures:**

1. **Formatting issues**: Run `pre-commit run --all-files` locally
1. **Build failures**: Check compiler compatibility and dependencies
1. **Test failures**: Verify tests pass locally first
1. **Windows-specific issues**: Path separators, case sensitivity
1. **ROS2 dependency issues**: Check package.xml dependencies

**Getting help:**

- Check CI logs for specific error messages
- Compare with recent successful builds
- Ask for help in issue comments

## Development Tips

1. **Use ccache** for faster local C++ builds
1. **Install pre-commit** to catch issues early
1. **Test locally** on your platform before pushing
1. **Check CI status** before requesting review
1. **Keep PRs focused** on single features/fixes

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
