# -------------------------------------------------
# bashForDummies


# Navigation
# exa > ls > tree
exa -la --git --tree
ls -l -a [folder] #-l (long) for details, -a (all) include hidden files
ls -lhrt
tree --du -h
tree -dh
broot # install with brew
# result of ll:
# ---------- 10 character
# drwxrwxrwx : d - implies directory, rwx - read-write-execute
# 3 groups for owner, owning group, and everyone else permission on this file/dir
chown USER[:GROUP] [file] # change ownership of a file
chmod +x [file] # change permission of a file, +/- rwx

pwd # current working directories
cd [dir] / cd .. / cd # or cd ~ = root
cd -  #previous dir
# pushd & popd

file [file_name] # description of file
du -ahd 1 [dir/file] # size of a dir/file
du -hsc * [dir/file]

# Search for file
# fzf > fdfind > find
fzf -e .sh$ !str # e-exact, $ include str, ! exclude str
fdfind #tags: u-unrestricted, t-type (f,d,x,l), e-extension, E-exclude, x-exec
# ex: fdfind -tf -tx -e py -x vim
find <in-folder> -type f -name <file_name> -exec rm {} \;
# ex: find . -name abc -type d; type d for dir, f for file
locate [file_name] # locate file in the entire system

# rg > grep
rg [pattern] # t-type
grep <what> <file_name> # search for a string in a file
# to search string in a dir
{grep -R, rgrep, rg, ag} <what> <dir>

# -------------------------------------------------
# Guidance
man [cmd] # manual guide for [cmd]
which [cmd] # or whatis [cmd]
apropos [something] # every command relate with [_]
type [cmd] # also show guide on that command, whether it's an alias
history # Ctrl + R

# -------------------------------------------------
# Working with files and directories
mkdir [dir]
rmdir [dir] # remove all empty/unnecessary directories
touch [file_name] # create or update
cp -r [files] [dir] # r-recursive
mv [files] [dir]
mmv 'name_pattern' 'new_name_pattern' # rename multiple files with desired pattern
for file in *.txt; do mv "$file" "${file%.txt}.sh"; done
rm (-r) (-f)
ln -s [OPTIONS] ORIGINAL_FILE LINKED_FILE #(soft) Symbolic link 

# -------------------------------------------------
# Placeholders
ls *.pdf      # * - any SET of characterS
ls ?uck*      # ? - any SINGLE character
ls [c-e]uck*  # [a-f] - characters in [abcdef]
ls duc[^a-c]* # [^a-c] - any character not in [abc]

# -------------------------------------------------
# View and edit files
# bat > less > more > cat
less [file_name] || more [file_name] || cat [file_name]
# vim > gedit ~ nano
gedit [file] || nano [file] || vim [file]

# sd > sed
sd [prev_expression] [new_expression] [file]
sd [prev_expression] [new_expression] [file] -p #p-preview
sed 's/[prev_expression]/[new_expression]/g' [file.txt] # just print out the results
sed -i 's/[prev_expression]/[new_expression]/g' [file.txt] # replace a string with another
sed '/searchStr/c\newLine' [file.txt] # replace a line contain a string

awk '/pattern/ {print $2}' file.txt
awk -f awkFD.awk file.txt

# counting lines of code
# tokei > cloc

# -------------------------------------------------
# Chaining and redirection commands
./[file.name] 1> output_file.txt 2> error_log.txt # use >> to append instead of overwrite
cmd1; cmd2; cmd3 # call cmds one after another, cmd2 runs even if cmd1 fails
cmd1 && cmd2 && cmd3 # same, but fails if any of the cmds return an error code
cmd1 || cmd2 # If cmd1 fails, run cmd2
cmd1 | cmd2 | cmd3 # pipe stdout of cmd1 to stdin of cmd 2, ..
# ex: ls | grep something

# -------------------------------------------------
sudo su # root user, be careful :)
exit

xdg-open # open stuff with default app
xargs # ??

# Time benchmarking
# hyperfine > time

# httpie > curl

# -------------------------------------------------
# Archive
# zip tags: r-recursive, e-encrypt, v-verbose, 9-compress better
zip -er9 output.zip file1 file2 # zip with encryption
# unzip tags: x *.h - exclude files .h, o-overwriting target_dir
# unzip tags: n-not overwrite, just unzip files not in target_dir
unzip -o input.zip -d target_dir # unzip to a dir
upzip -l input.zip | less # looking inside zip file
zipcloack file.zip # add password, in case u forgot it before
zipsplit -n [size_in_bytes] file.zip # split to size restriction
# tar tags: v-verbose, f-files, c-create, z-gunzip, x-extract, t-list
tar cvf [target_file.tar] [files/dirs] # create .tar
tar zcvf [target_file.tar.gz] [files/dirs] # create .tar.gz
tar tvf [file.tar] # list out files in file.tar
tar xvf [file.tar] -C [dirs] # extract tar file
tar xvfz [file.tar.gz] [dirs] # extract tar.gz file
gzip [file.tar] # compress the tar file -> .tar.gz file
gunzip [file.tar.gz] # uncompress the .tar.gz file -> .tar file
unrar x [file.rar]

# -------------------------------------------------
# Symmetric cryptography
openssl aes-256-cbc -salt -pbkdf2 -in <file.name> -out <file.enc.name>
openssl aes-256-cbc -d -pbkdf2 -in <file.enc.name> -out <file.dec.name>
cmp <file1> <file2> | echo $? #compare 2 file, 0 if same, 1 if different

# -------------------------------------------------
# Monitor your system
htop
neofetch # kute :3

sensible-browser $WEBSITE
sensible-editor
sensible-pager

# -------------------------------------------------
# Killing / resuming process
# Ctrl-C - SIGINT
# Ctrl-\ - SIGQUIT
fg || bg # resume task in foreground, background
jobs
kill -l
kill -9
