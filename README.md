# codingForDummies

[![tokei](https://tokei.rs/b1/github/duken72/codingForDummies)](https://github.com/duken72/codingForDummies)

Important things about coding that I have learnt by myself and from the professionals. Overall, this is more like a collection of notes from different sources. Others have already done an amazing job of explaining stuffs. Let's not re-invent the wheel. It's just that .. everything by themselves is scatterred everywhere .. Perhaps this would make your search a bit more directed.

- This README contains general terms or problems.
- Other folders contain guides and samples for specific coding languages, tools and possible terminal commands.

## Table of Contents

- [Abbreviations & Definitions](#abbreviations--definitions)
- [Package and Package Management System](#package-and-package-management-system)
- [License](#license)
- [Docker](#docker)
- [Unit Testing](#unit-testing)
- [DevOps](#devops)
  - [CI](#ci---continuous-integration)
  - [CD](#cd---continuous-deliverydeployment)
  - [Build Server](#build-server)
  - [Test Server](#test-server)
- [Agile Project Management](#agile-project-management)
- [LICENSE](#license)

-------

## Abbreviations & Definitions

- CLI - Command-line Interface.
- API - Application Programming Interface: [What is an API?](https://youtu.be/s7wmiS2mSXY).
- Compiling: translates source code (human-readeable, eg., in Python, C++) to low-level machine language (eg., Assembly). [How do computers read code?
](https://youtu.be/QXjU9qTsYCc)
- Building: could include generating source code, documentation, compiling of source code, packaging complied code, installing, etc.

-------

## Pull Requests

Yes :)) I made a section just for Pull Requests.

- Remember to merge the main/develop branch and fix merge conflicts:

  ```bash
  git fetch -a && git merge origin/develop
  ```

- Invite issue reporter and colleagues to review your code and contribute changes
- Write decent PRs when you submit, and look out for these when you review one:
  - **What does the PR do?**: eg. screenshots, videos  
  Does the PR do what it's supposed to do? Does it do it correctly?
  - **Is the code readable and clean?** too long functions doing too many things, coding-style, indentation, etc.)
  - **Can the code be shorter, smarter?** duplicated of codes, suggest efficency optimization)
  - **Are changes covered by tests?** "fairly complex" functions should have unit tests)
  - **Is it documented?** README updated, functions commented in Doxygen style

-------

## Package and Package Management System

- .iso -
- .exe in Windows - .elf in Linux (kind of, not precisely)
- apt - Advanced Package Tool
- .deb -
- dpkg - Debian package management system

-------

## License

Goto [licenseFD.md](licenseFD/licenseFD.md)

-------

## Docker

Check [Docker.md](dockerFD/README.md)

-------

## Unit Testing

- GTEST: also known as Google test, for C++ [1](https://youtu.be/nbFXI9SDfbk), [2](https://youtu.be/BwO07hUzFNQ).
- pytest [1](https://youtu.be/DhUpxWjOhME)
- unittest [1](https://youtu.be/6tNS--WetLI), [2](https://youtu.be/1Lfv5tUGsn8), [3](https://youtu.be/uCxL7NGEohI).

-------

## DevOps

Set of practices to build, test and release your code in small frequent steps [1](https://youtu.be/scEDHsr3APg).

### CI - Continuous Integration

Autotomate test, build and deploy.

- Nightly build vs Continuous Integration
- [Jenkins For Beginners](https://youtu.be/LFDrDnKPOTg)
- Other open-source tools: Bamboo, Buildbot, Apache Gump, Travis CI.
- Github action

### CD - Continuous Delivery/Deployment

### Build server

- Maven: [1](https://youtu.be/bSaBmXFym30), [2](https://youtu.be/JXXdipKFLQg)

### Test server

- Selenium

### Example Build Pipeline

- Clone target repo
- Configure GBP pipeline
- Clone dependency repos
- Setup docker build
- Setup build environment
- Prepare workspace
- Build dependencies
- Lint target
- Build target
- Test target
- Run custom smoke test
- Parse logs

-------

## Agile Project Management

This's a bit actually off to project management than just coding.

- Jira: [YouTube](https://youtu.be/xrCJv0fTyR8)
- Atlassian: [YouTube](https://youtu.be/hWXNmcSN4bE)
- Confluence: [YouTube](https://youtu.be/uhWCMlcY_Zw)

-------

## TODO

- Artifactory - JFrog
- Azure Storage, KeyVault, Container Registry, etc.
- Jenkins
- Gliffy diagram

-------

## LICENSE

See the [LICENSE](LICENSE.md) file for license rights and limitations (MIT).
