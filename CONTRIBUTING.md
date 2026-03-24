# Contributing to MOLA

Thank you for your interest in contributing to MOLA. This document outlines the process and expectations for contributing to this project. Please read it carefully before submitting any contribution.

MOLA is a modular SLAM and localization framework written in C++ with ROS 2 support. We welcome contributions from the community, provided they meet the quality and legal requirements described below.

---

## Table of Contents

1. [Contributor License Agreement](#1-contributor-license-agreement)
2. [Code of Conduct](#2-code-of-conduct)
3. [Types of Contributions](#3-types-of-contributions)
4. [Getting Started](#4-getting-started)
5. [Development Workflow](#5-development-workflow)
6. [Commit and Pull Request Guidelines](#6-commit-and-pull-request-guidelines)
7. [Coding Standards](#7-coding-standards)
8. [Testing Requirements](#8-testing-requirements)
9. [Documentation](#9-documentation)
10. [Review Process](#10-review-process)
11. [Reporting Issues](#11-reporting-issues)

---

## 1. Contributor License Agreement

All contributions to MOLA require acceptance of the [Contributor License Agreement (CLA)](./individual-cla.md) prior to being merged.

MOLA is distributed under a dual licensing model:

- **Open source:** GNU General Public License v3 (GPL v3)
- **Commercial:** A separate commercial license is available for industry users who cannot comply with the terms of the GPL v3

The CLA ensures that the project maintainers hold the necessary rights to distribute your contribution under both licensing models. It does not transfer copyright ownership — you retain full ownership of your work.

When you open a Pull Request, an automated CLA assistant will verify whether you have previously accepted the agreement. If you have not, you will be prompted to do so before the PR can be reviewed or merged. Contributions submitted without CLA acceptance will not be considered.

---

## 2. Code of Conduct

All contributors are expected to adhere to the project's [Code of Conduct](./CODE_OF_CONDUCT.md). We are committed to maintaining a welcoming and professional environment for everyone, regardless of background or experience level. Violations may result in removal from the project.

---

## 3. Types of Contributions

We welcome the following types of contributions:

- **Bug fixes** — corrections to incorrect or unexpected behavior
- **Performance improvements** — optimizations with measurable, documented impact
- **New modules or algorithms** — extensions to MOLA's modular architecture
- **Documentation improvements** — corrections, clarifications, and additions to existing documentation
- **Dataset integrations** — support for additional SLAM benchmark or real-world datasets
- **ROS 2 integrations** — improvements to the ROS 2 interface, launch files, or message definitions
- **Tests** — new or improved unit, integration, or regression tests

If you are considering a large or architectural contribution, please open an issue for discussion before beginning work. This avoids duplication of effort and ensures alignment with the project roadmap.

---

## 4. Getting Started

Full setup instructions, dependencies, and build guides are available in the official documentation:

**https://docs.mola-slam.org/**

Ensure your development environment is fully configured and that all existing tests pass before making any changes.

---

## 5. Development Workflow

1. **Fork** the repository and create a new branch from `main` (or the relevant base branch).
2. Name your branch descriptively, e.g. `fix/imu-integration-drift` or `feat/lidar-odometry-module`.
3. Make your changes in focused, logically grouped commits.
4. Ensure all tests pass locally before opening a Pull Request.
5. Open a Pull Request against the `main` branch with a clear description of your changes.

Do not submit Pull Requests that bundle unrelated changes. Each PR should address a single concern.

---

## 6. Commit and Pull Request Guidelines

**Commits**

- Write commit messages in the imperative mood: `Fix timestamp alignment in IMU handler`, not `Fixed` or `Fixes`.
- Keep the subject line under 72 characters.
- Reference relevant issues in the commit body where applicable, e.g. `Closes #123`.

**Pull Requests**

- Provide a clear summary of the problem being solved and the approach taken.
- Include references to any related issues, discussions, or prior art.
- If the PR introduces a user-visible change, update the relevant documentation and changelog.
- Mark the PR as a draft if it is not yet ready for review.

---

## 7. Coding Standards

All C++ code must conform to the project's established style guidelines, which follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with project-specific modifications documented in the repository. A `.clang-format` configuration file is provided at the root of the repository. All submissions must pass automated formatting checks.

Key requirements:

- C++17 or later
- No use of deprecated ROS 2 APIs
- Headers must be self-contained and include guards or `#pragma once`
- Avoid unnecessary dependencies; additions to `CMakeLists.txt` must be justified in the PR description

---

## 8. Testing Requirements

All contributions that modify functional code must include appropriate tests. The project uses [Google Test (gtest)](https://github.com/google/googletest) for unit testing.

- New functionality must be accompanied by unit tests covering expected behavior and relevant edge cases.
- Bug fixes must include a regression test that fails without the fix and passes with it.
- Tests must pass on all supported platforms prior to merge.

CI checks are run automatically on all Pull Requests. PRs that fail CI will not be merged until all issues are resolved.

---

## 9. Documentation

Public APIs, new modules, and non-trivial algorithms must be documented. Documentation contributions are subject to the same review process as code. Where applicable:

- Use Doxygen-compatible docstrings for C++ API documentation.
- Update or create Markdown documentation under the `docs/` directory for higher-level guides.
- For algorithmic contributions, include a reference to the source paper or method in a comment or docstring.

---

## 10. Review Process

All Pull Requests are reviewed by at least one project maintainer. The review process evaluates correctness, code quality, test coverage, documentation, and alignment with the project's architecture and goals.

Reviewers may request changes before a PR is approved. Requested changes should be addressed promptly; PRs that remain inactive for an extended period may be closed at the maintainers' discretion and can be reopened when work resumes.

Approval and merge are at the sole discretion of the project maintainers.

---

## 11. Reporting Issues

To report a bug or request a feature, please open an issue on the GitHub repository. Before doing so, search existing issues to avoid duplicates.

A good bug report includes:

- A concise description of the problem
- Steps to reproduce the issue
- Expected vs. actual behavior
- Relevant environment details (OS, compiler version, ROS 2 distribution, MOLA version)
- Any relevant logs, stack traces, or screenshots

---

*For questions about the contribution process, open a discussion on the GitHub repository.*
