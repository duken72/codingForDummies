# https://missing.csail.mit.edu/2020/shell-tools/

# shebang
# Execute the file using which shell / interpreter
#!/bin/sh
#!/bin/bash
#!/usr/bin/env python3

shellcheck file.sh
# Turn on/off tracing with
set -x
set +x

foo=bar # 'foo = bar' will not run, it interprete foo as command, "=" as 1st argument, "bar" as 2nd argument
echo "$foo" # prints bar
echo '$foo' # prints $foo
read #Keyboard Input, tag -p, -t, -s

mcd()
{
  mkdir -p "$1"
  cd "$1"
}

# $0 - Name of the script
# $1 to $9 - Arguments to the script. $1 is the first argument and so on.
# $@ - All the arguments
# $# - Number of arguments
# $? - Return code of the previous command
# $$ - Process identification number (PID) for the current script
# !! - Entire last command, including arguments, ex: sudo !! if last command fail due to permission
# $_ - Last argument from the last command.
# If you are in an interactive shell, you can also quickly get this value by typing Esc followed by .

# Logical
# true -> 0, false -> 1
false || echo "Oops, fail" # Oops, fail
true || echo "Will not be printed"
true && echo "Things went well" # Things went well
false && echo "Will not be printed"
true ; echo "This will always run"
false ; echo "This will always run"

# {} as comment substring
convert image.{png,jpg}
# Will expand to
convert image.png image.jpg

# Flow Control
# if - elif - else
if conditions; then
  commands
[elif conditions; then
  commands]
[else conditions; then
  commands]
fi
if ([condition1] || [condition2]) && [condition3]; then
  commands
fi
# for
for i in word1 word2 word3; do
  echo "$i"
done
for i in "$@"; do
  echo $i
done

# Test
# Check file/directory existence
if [ -d file ]; then echo "file is a directory" fi
if [ -e file ]; then echo "file exists" fi
if [ -f file ]; then echo "file exists and is a regular file" fi
if [ -L file ]; then echo "file is a symbolic link" fi
if [ -r file ]; then echo "file is a file readable by you" fi
if [ -w file ]; then echo "file is a file writable by you" fi
if [ -x file ]; then echo "file is a file executable by you" fi
if type cmd; then echo "cmd exists" fi
#according to modification time
if [ file1 -nt file2 ]; then echo "file1 is newer than file2" fi
if [ file1 -ot file2 ]; then echo "file1 is older than file2" fi
#string
if [ -z string ]; then echo "string is empty" fi
if [ -n string ]; then echo "string is not empty" fi
if [ string1 = string2 ]; then echo "string1 equals string2" fi
if [ string1 != string2 ]; then echo "string1 does not equal string2" fi

## Comment block
: <<'END'
echo "I hope that there is no error"
END
