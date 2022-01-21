# VS-Code

## Hotkeys

- Ctrl+Space        Invoke IntelliSense suggestion
- Ctrl+C            Copy entire current line without text selection
- Alt+UpArrow       Move entire selected line(s) up/down
- Ctrl+Shift+K      Delete the entire line
- F2                Rename Refactoring
- Ctrl+Shift+I      Formating
- Ctrl+Shift+[ or ] Code folding
- F8                Errors and Warning
- Open terminal
- Find files Ctrl+P
- Extensions: Ctrl+Shift+X

-------

## Snippets

Go to File -> Preferences -> User snippets

```json
"Add include guard": {
  "prefix": "guard",
  "description": "Adds an ifndef include guard to a C/C++ header",
  "body": [
    "#ifndef ${TM_DIRECTORY/(.*[\\/](\\w+$))/${2:/upcase}/}__${TM_FILENAME/^([^\\.]*)\\..*$/${1:/upcase}/}_${TM_FILENAME/^.*\\.([^\\.]*)$/${1:/upcase}/}__",
    "#define ${TM_DIRECTORY/(.*[\\/](\\w+$))/${2:/upcase}/}__${TM_FILENAME/^([^\\.]*)\\..*$/${1:/upcase}/}_${TM_FILENAME/^.*\\.([^\\.]*)$/${1:/upcase}/}__",
    "",
    "$0",
    "",
    "#endif ${LINE_COMMENT} ${TM_DIRECTORY/(.*[\\/](\\w+$))/${2:/upcase}/}__${TM_FILENAME/^([^\\.]*)\\..*$/${1:/upcase}/}_${TM_FILENAME/^.*\\.([^\\.]*)$/${1:/upcase}/}__",
    ""
  ]
}
```