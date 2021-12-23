# BASH

CLI commands:

```bash
alias bashfd='cd $(fd -t d bashFD ~) && ls'
alias bashg='less $(fd bashFD.bash ~)'
alias bashgg='less $(fd bashFD.bash ~) | grep'
```

-----

To interact with the OS.  
All the commands are actually from folders in {PATH}

```bash
echo $PATH
```

Check [bashFD.bash](bashFD.bash) for commands.
