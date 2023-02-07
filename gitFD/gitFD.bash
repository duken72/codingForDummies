# HELP
git help <verb>     # help
git <verb> --help   # help

# -------------------------------------------------
# INIT CURRENT FOLDER
git init [folder]       # init folder
git add .               # init folder - staging
git commit -m "bla bla" # init folder - staging
git branch -M main      # init folder - naming current local branch
git remote add origin <LINK_to_git_repo> # init folder - declare remote repo
git push -u origin main # init folder - push to branch main of remotes/origin

# -------------------------------------------------
# CONFIGS
git remote set-url origin git@github.com:username/repo.git # config
git config --global credential.helper 'cache --timeout 7200' # config - enable credential caching time
# CREDENTIALS & SIGNING
git config --global credential.helper store # config - save the key in .git-credentials
# To change passphrase for private SSH key
ssh-keygen -f ~/.ssh/id_rsa -p
# Signing commit, tag with GPG key
gpg --list-secret-keys --keyid-format=long # check current keys

# -------------------------------------------------
# BASICS
git status
git rm                              # remove from the working tree
git restore                         # unchange
git add [files/dirs]                # staging   / stage
git reset [files/dirs]              # unstaging / unstage
git commit -m "Message"             # Commit with a message
git commit -am "Message"            # add and commit
git commit --admend "New message"   # change commit message
git add . && git commit --admend --no-edit  # add files to previous commit, same message
git pull                            # pull from remote repo
git push [origin] [branch_name]     # push to remote repo, branch [branch_name]

# -------------------------------------------------
# CLONE / SUBMODULE
git clone <source.git> [dir]
git submodule add <source.git> [dir]
git remote -v                   # list info of the repo

# -------------------------------------------------
# EXAMINE
git diff <revision> <filename>  # examine - see current changes
git diff <source_branch> <target_branch>
git show                        # examine - see last changes
git blame                       # examine - who edited which lines
git checkout [commitID]         # branch - checkout specified commit

# LOGGING
git log --author=duke               # log
git log --pretty=oneline            # log
git log --all --graph --decorate    # log
git log -p, shortlog                # log

# -------------------------------------------------
# WHEN YOU FUCKED THINGS UP
git checkout -- <filename>      # fucked - replace local changes to last content in HEAD
git fetch origin                # fucked - update local remote branches from server
git reset --hard origin/master  # fucked - unstage and remove all changes

# -------------------------------------------------
# BRANCHING
git fetch               # branch - update local remote branches to remote branches
git fetch --prune       # branch - prune branches that no longer has remote repo counterpart
git branch              # branch - show branches, a-all, r-remote(only), d-delete
git branch [name]       # branch - create a branch
git branch -d [name]    # branch - delete a branch locally
git checkout [branch_name/hash_of_snapshot] # branch - switch to branch
git checkout -b feature_x   # branch - create then switch branch
git checkout -          # branch - checkout previous branch
git push origin --delete remoteBranchName   # branch - delete branch remotely
# If you have made changes, you have to either stash or commit before checkout other branches

# -------------------------------------------------
# MERGING
# git checkout master
git merge [name]    # merge current branch with another
git merge --allow-unrelated-histories [name] # merge - use with care, or just don't use it

# -------------------------------------------------
# TAGGING
# For software release
# <commitID> only needs first 10 characters, eg., 1b2e1d63ff
git tag 1.0.0 <commitID>

# -------------------------------------------------
# ADVANCE STUFFS (MAYBE JUST TO ME)
git stash                   # stash - remove the changes, save it
git stash pop               # stash - apply the saved changes
git stash save <name>       # stash - but with specified name
git stash list              # stash - list the saved changes
git stash apply <name>      # stash - apply specified change

git revert <commitID>   # undo a commit with a new commit, ID from `git log --oneline`
git rebase              # TODO
git bisect              # TODO
