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

## Code Style Support

### Code completion and Liting for Python

Set in workspace configuration `<path-to-ws>/.vscode/` a [setting.json](settings.json)

### Code completion for C++

In similar manner, maintain in the `<path-to-ws>/.vscode/` a [c_cpp_properties.json](c_cpp_properties.json)

### Format code C++

Using the extension [RunOnSave](https://marketplace.visualstudio.com/items?itemName=emeraldwalk.RunOnSave).

Add to `settings.json` of VSCode (via `Ctrl+P`, `>Preferences: Open Settings (JSON)`):

```json
"emeraldwalk.runonsave": {
  "commands": [
  {
    "match": ".(h|hpp|c|cpp)$",
    "isAsync": true,
    "cmd": "ament_uncrustify ${file} --reformat"
  },
  ]
}
```

-------

## Snippets

Go to `File >Preferences >User snippets`.

Check [global.code-snippets](global.code-snippets)

```json
"Snippet purpose": {
  "prefix": "prefix_name",
  "description": "Do something",
  "body": [
    "line 1",
    "line 2",
    "",
    "line 3",
    ""
  ]
}
```
