# VS-Code

## Hotkeys

|             Command              |                Action                 |
|:-------------------------------: |:------------------------------------: |
|              Ctrl+P              |              Find files               |
|           Ctrl+Shift+E           |            File Explorers             |
|           Ctrl+Shift+P           |           Show all commands           |
|            Ctrl+Space            |    Invoke IntelliSense suggestion     |
| Ctrl+C (without text selection)  |       Copy entire current line        |
|           Ctrl+Shift+K           |        Delete the entire line         |
|          Alt+MoveArrow           | Move entire selected line(s) up/down  |
|              Ctrl+F              |          Find words in file           |
|              Ctrl+H              |     Find & replace words in file      |
|           Ctrl+Shift+F           |        Find words in workspace        |
|           Ctrl+Shift+H           |   Find & replace words in workspace   |
|                F2                |          Rename Refactoring           |
|                F8                |          Errors and Warning           |
|           Ctrl+Shift+I           |               Formatting              |
|        Ctrl+Shift+[ or ]         |             Code folding              |
|             Ctrl + `             |             Open Terminal             |
|           Ctrl+Shift+M           |             Open Problems             |
|           Ctrl+Shift+X           |              Extensions               |
|           Ctrl+Shift+G           |                  Git                  |
|              Ctrl+,              |               Settings                |
|              Ctrl+.              |             Code actions              |
|              Ctrl+B              |           Close side panels           |
|              Ctrl+J              |          Code bottom panels           |
|              Ctrl+\              |             Split editor              |
|              Ctrl+2              |               Extra tab               |

-------

## Code Style Support

### Code completion and Linting for Python

Set in workspace configuration `<path-to-ws>/.vscode/` a [setting.JSON](settings.JSON)

### Code completion for C++

In similar manner, maintain in the `<path-to-ws>/.vscode/` a [c_cpp_properties.JSON](c_cpp_properties.JSON)

### Format code C++

Using the extension [RunOnSave](https://marketplace.visualstudio.com/items?itemName=emeraldwalk.RunOnSave).

Add to `settings.JSON` of VSCode (via `Ctrl+P`, `>Preferences: Open Settings (JSON)`):

```JSON
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

```JSON
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
