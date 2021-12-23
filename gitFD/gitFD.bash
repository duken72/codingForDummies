git fetch # update local remote branches to branches on remote repo
git fetch --prune # prune branches that no longer has remote repo counterpart

# To upload current project to github from computer
# create repo on git
git init [folder]
git add . # staging
git commit -m "bla bla" # create snapshot
git branch -M main # name the current local branch as "main"
git remote add origin <LINK_to_git_repo> # declare the corresponding remote repo (on github ..)
git push -u origin main # push from current local branch "main", to remotes/origin, branch main

# -------------------------------------------------
git remote set-url origin git@github.com:username/repo.git
git config --global credential.helper store # this save the key in .git-credentials
git config --global credential.helper 'cache --timeout 7200' # Also enabling credential caching to 2hrs
# To change passphrase for private SSH key
ssh-keygen -f ~/.ssh/id_rsa -p
# Signing commit, tag with GPG key
gpg --list-secret-keys --keyid-format=long # check current keys

# -------------------------------------------------
git pull
git add [files/dirs]
git commit --admend --no-edit #not the commit contains no change!
git status # show if we behind, ahead on staging and commit
git push [origin] [branch_name] # push to remote repo, branch [branch_name]

# -------------------------------------------------
git log --all --graph --decorate
git log -p, shortlog
git revert, rebase
git show
git diff <revision> <filename>
git blame # show who edited which line, too details, git show is better

# -------------------------------------------------
git branch [-a] # show branches, -a to show also remote branches, -r show only local remote branches, run fetch first to get all branches at remote repo
git branch [name] # create a branch
git checkout [branch_name/hash_of_snapshot] # switch to a local branch, -b create a branch first then switch
git branch -d [name] # delete a branch locally
git push origin --delete remoteBranchName # delete branch remotely

git merge [name] # merge current branch with another, should be in master/main branch first, git checkout master
git merge --allow-unrelated-histories [name] # use with care, or just don't use it
