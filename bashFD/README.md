<!-- omit in toc -->
# BASH

<!-- omit in toc -->
## Table of Contents

- [CLI commands](#cli-commands)
- [Overview](#overview)
- [Commands](#commands)

-----

## CLI commands

```bash
alias bashfd='cd $(fd -t d bashFD ~) && ls'
alias bashg='less $(fd bashFD.bash ~)'
alias bashgg='less $(fd bashFD.bash ~) | grep'
```

-----

## Overview

To interact with the OS.  
All the commands are actually from folders in {PATH}

```bash
echo $PATH
```

-----

## Commands

Check [bashFD.bash](bashFD.bash) for commands.
