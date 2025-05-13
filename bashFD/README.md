# BASH

Bash stands for Bourne Again SHell. It's the command language to interact with the OS.  
All the commands are actually from folders in `PATH`

```bash
echo $PATH
```

## Table of Contents

<!-- vim-markdown-toc GFM -->

* [Commands](#commands)
* [Style Guide for Shell scripting](#style-guide-for-shell-scripting)

<!-- vim-markdown-toc -->

---

## Commands

- Check [bashFD.bash](bashFD.bash) for commands.
- Browse commands with

  ```bash
  alias bashfd='cd $(fd -t d bashFD ~) && ls'
  alias bashg='less $(fd bashFD.bash ~)'
  alias bashgg='less $(fd bashFD.bash ~) | grep'
  ```

- Check [CLI_hotkeys.md](CLI_hotkeys.md) for hotkeys working in Linux terminal.

---

## Style Guide for Shell scripting

Style guide to keep this neat:

- leave two blank lines before a new function
- leave one blank line before completion function, e.g `\_function-complete`, which associates to a `function`
- leave no blank line before complete function, e.g. `complete -F function`, which associates to a `\_function-complete`
- all `print`s, `echo`s, `log`s etc. should start with lower-case

Coloring scripts:

```bash
COLOR_RED='\033[1;31m'
COLOR_BLUE='\033[0;34m'
NO_COLOR='\033[0m'

echo -e "Normal color ${COLOR_RED}red ${COLOR_BLUE}blue${NO_COLOR} normal"
```

Check source [stackoverflow](https://stackoverflow.com/questions/5947742/how-to-change-the-output-color-of-echo-in-linux):

- Color mode:
  - `1;` lighter than normal
  - `2;` darker than normal
- Text mode:
  - `3;` italic
  - `4;` underline
  - `5;`/`6;` blinking slow or fast
  - `7;` reverse
  - `8;` hide
  - `9;` cross-out

```bash
echo -e "\033[4mtext here\033[m"
echo -e "\033[4m \033[9m text here \033[m"
```
