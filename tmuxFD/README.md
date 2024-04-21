# TMUX

## Table of Contents

<!-- vim-markdown-toc GFM -->

* [Hotkeys](#hotkeys)
  * [Panes](#panes)
  * [Windows](#windows)
  * [Sessions](#sessions)
  * [Others](#others)

<!-- vim-markdown-toc -->

## Hotkeys

- Detach from server: Ctrl+B+D

### Panes

- Split panes: Ctrl+B+% or Ctrl+B+", like the way Terminator split into many sides, in the same window
- Move around panes with Ctrl+B+navigation key

### Windows

- Split windows: Ctrl+B, 'C'
- Move around windows: Ctrl+B, number
- Rename window: Ctrl+B, ','
- Focus on one window: Ctrl+B, 'Z'
- Resize: Ctrl+B, 'Ctrl+navigation key'

### Sessions

- View session: `tmux ls`
- Re-attach to session: `tmux attach -t session_name`
- Kill session: `tmux kill-session -t session_name`

### Others

- Scroll around: Ctrl+B, '[', then UpArrow, PgDown, j, k, Ctrl+D, Ctrl+U
- Show hotkeys: Ctrl+B, '?'
- Show clock: Ctrl+B, 'T'
