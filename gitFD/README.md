<!-- omit in toc -->
# GIT

<!-- omit in toc -->
## Table of Contents

- [CLI commands](#cli-commands)
- [Syncing](#syncing)
- [Guides](#guides)

-------

## CLI commands

```bash
alias gitg='less $(fd gitFD.bash ~)'
alias gitgg='less $(fd gitFD.bash ~) | grep'
```

Check [gitFD.bash](gitFD.bash) for commands.

Some guides:

- [Version control git](https://missing.csail.mit.edu/2020/version-control/)
- [Oh Shit, Git!?!](https://ohshitgit.com/#accidental-commit-master)
- [Creating a personal access token](https://docs.github.com/en/github/authenticating-to-github/keeping-your-account-and-data-secure/creating-a-personal-access-token)

-------

## Syncing

There is a difference between a remote branch and a branch existing in a remote repository (github, bitbucket, gitlab ..).  
Remote branch is a branch in your local machine, mapping to a branch in remote repo.

To sync (push/pull), use either PAT (Personal Access Token) or SSH. With PAT, one could specify different level of permissions, while with SSH, there can only be read-only or read-write.

To stop asking for PAT when pushing to origin, save the key in `.git-credentials`:

```bash
git config --global credential.helper store
```

Use `.gitignore` to intentionally specify untracked files.

-------

## Guides

- [GitHub Flow](https://docs.github.com/en/get-started/quickstart/github-flow)
- [Git branching model](https://nvie.com/posts/a-successful-git-branching-model/)
- [git - the simple guide](https://rogerdudler.github.io/git-guide/)
- [Learn Git Branching](https://learngitbranching.js.org/)
- [Git Tutorial for Beginners](https://www.youtube.com/watch?v=HVsySz-h9r4)
