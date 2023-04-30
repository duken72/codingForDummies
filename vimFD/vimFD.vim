# -------------------------------------------------
:set number - line number

# -------------------------------------------------
## Modes
ESC             # command mode
i               # insert mode
r               # replace mode, then enter insert mode
v               # visual mode

# -------------------------------------------------
## Basics for kids
#### In and out
:q              # close a window, check vim window
:qa             # quit all window
:q!             # quit without writing
:w              # write
:wq             # save and exit

# -------------------------------------------------
#### Navigating / aka Motion
hjkl            # simple movement (single char)
wbe             # words: next, beginning, end of word
WBE             # words, but seperated by space
0$		# start/end of line
^		# start of line (non-blank)
f{char}/F{char}	# go to next/previous char in current line
t{char}/T{char}	# move before next/previous char
{}		# previous/next empty line
%		# go to matching char '()', '[]', '{}'
Shift-HML       # screen top, middle, bottom
Ctrl-UD         # scroll up/down
Ctrl-FB         # scroll faster up/down
zz		# center current line
gg              # move to start of the file
G               # move to bottom of the file
Ctrl-G          # tell where you are
:number         # go to line number


# -------------------------------------------------
#### Editing
i               # insert mode
r               # replace mode, then enter insert mode
s               # xi, delete char, then enter insert mode
d{motion}       # delete motion, eg: dw, de, dd, d$, d0
c{motion}       # change motion, eg: cw, ce, cc, c$, c0
y{motion}       # copy motion, eg: yw, yy, y$, y0
D               # delete the remaining of line, = d$
C               # change the remaining of line, = c$
A               # append text at the end of line
x / X           # delete/backspace character
o / O           # insert line below/above and enter insert mode
p / P           # paste after/before the cursor
u               # undo last command
U               # undo all cmds in the current line
Ctrl-r          # redo command
.               # repeat the last command, could be somewhere else
q<char>         # start record keystrokes into register <char>
q               # stop record
@<char>         # play recorded keystrokes into register <char>
@@              # repeat last recording
ci(             # change inside ( bracket
da{             # delete around { bracket, so include { and }
##### Replace
:s/searchStr/substituteStr/         # replace first searchStr with substituteStr in current line
:s/searchStr/substituteStr/g        # replace all searchStr with substituteStr in current line
:%s/searchStr/substituteStr/g       # replace all searchStr with substituteStr in whole file
:2,7s/searchStr/substituteStr/g     # replace all searchStr with substituteStr in lines 2-7 
##### Search only
/[word]+Enter   # search for word
n / shift-N     # go to next or previous searched word

#### Visual mode: copy, paste, delete at an .. shit level
v               # visual mode
Shift-v         # visual line mode
Ctrl-v          # visual block mode

#### Counts
3w              # 3 words forward
5j              # 5 lines down
7dw             # delete 7 words

# -------------------------------------------------
#### Vim window, not tab, but synced
:sp             # split horizontal
Ctrl+w, s       # split horizontal
:vsp            # split vertical
Ctrl+w, v       # split vertical
Ctrl+w, q       # close splitted window
Ctrl+w, w       # switch around window
Ctrl+w, hjkl    # switch around adjacent window

# -------------------------------------------------
#### Advanced stuffs?
Ctrl+]		# jump to tag
Ctrl+O, Ctrl+T	# jump back
Shift+j		# join two lines togeter
Shift+Z+Z	# write and exit (same as :wq)
:help CMD	# show helps relating to cmd, or some feature

> + {motion}	# indent right
< + {motion}	# indent left
