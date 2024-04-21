<!-- omit in toc -->

# GIT

<!-- omit in toc -->

## Table of Contents

- [Guides](#guides)
- [CLI commands](#cli-commands)
- [Tips and Tricks](#tips-and-tricks)
- [Advanced Techniques](#advanced-techniques)
- [Syncing](#syncing)
- [Pull Requests](#pull-requests)
- [README Badges](#readme-badges)

---

## Guides

More advanced:

- [Git branching model](https://nvie.com/posts/a-successful-git-branching-model/)
- [GitHub Flow](https://docs.github.com/en/get-started/quickstart/github-flow)
- [Learn Git Branching](https://learngitbranching.js.org/)
- [13 Advanced (but useful) Git Techniques and Shortcuts](https://youtu.be/ecK3EnyGD8o)
- [Creating a personal access token](https://docs.github.com/en/github/authenticating-to-github/keeping-your-account-and-data-secure/creating-a-personal-access-token)
- [Oh Shit, Git!?!](https://ohshitgit.com/#accidental-commit-master): What to do when you fucked things up

Basics:

- [Version control git](https://missing.csail.mit.edu/2020/version-control/)
- [git - the simple guide](https://rogerdudler.github.io/git-guide/)
- [Git Tutorial for Beginners](https://www.youtube.com/watch?v=HVsySz-h9r4)

---

## CLI commands

```bash
alias gitg='less $(fd gitFD.bash ~)'
alias gitgg='less $(fd gitFD.bash ~) | grep'
```

- Check [gitFD.bash](gitFD.bash) for commands.
- Alternatively, just go with `git help <verb>`

---

## Tips and Tricks

```bash
git commit -am "Message"                    # combine add and commit
git config --global alias.ac "commit -am"   # setting git alias
```

---

## Advanced Techniques

Source: [13 Advanced (but useful) Git Techniques and Shortcuts](https://youtu.be/ecK3EnyGD8o)

- `git commit --amend`

  - can be used to change previous commit message, if you haven't pushed it to the remote repo

  ```bash
  git commit --amend -m "New message"
  ```

  - can also be used to add more changes to previous commit

  ```bash
  git add .
  git commit --amend --no-edit
  ```

- `git revert`: undo a commit with a new commit, but the previous commit is still in history
- `git stash`: remove the changes, but save them for later use

  ```bash
  git stash       # remove the changes, save it
  git stash save <something>
  git stash list
  git stash pop   # apply the saved changes
  git stash apply <something>
  ```

- `git bisect`: TODO
- Squash multiple commits:
- Git hook: TODO

---

## Syncing

There is a difference between a remote branch and a branch existing in a remote repository (GitHub, Bitbucket, GitLab ..).  
Remote branch is a branch in your local machine, mapping to a branch in remote repo.

To sync (push/pull), use either PAT (Personal Access Token) or SSH. With PAT, one could specify different level of permissions, while with SSH, there can only be read-only or read-write.

To stop asking for PAT when pushing to origin, save the key in `.git-credentials`:

```bash
git config --global credential.helper store
```

Use `.gitignore` to intentionally specify untracked files.

---

## Pull Requests

Yes :)) I made a section just for Pull Requests.

- Remember to merge the main/develop branch and fix merge conflicts:

  ```bash
  git fetch -a && git merge origin/develop
  ```

- Invite issue reporter and colleagues to review your code and contribute changes
- Write decent PRs when you submit, and look out for these when you review one:
  - **What does the PR do?**: e.g. screenshots, videos  
    Does the PR do what it's supposed to do? Does it do it correctly?
  - **Is the code readable and clean?** Too long functions doing too many things, coding-style, indentation, etc.
  - **Can the code be shorter, smarter?** Duplicated of codes, suggest efficiency optimization
  - **Are changes covered by tests?** "Fairly complex" functions should have unit tests
  - **Is it documented?** README updated, functions commented in Doxygen style

---

## README Badges

Check <https://shields.io/>
