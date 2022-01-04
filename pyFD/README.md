# Basics

## Table of Contents

- [CLI commands](#cli-commands)
- [Tips and Tricks](#tips-and-tricks)
  - [Naming Convention](#naming-convention)
  - [Docstrings](#docstrings)
  - [Function annotations](#function-annotations)
  - [PEP - Python Enhancement Proposal](#pep---python-enhancement-proposal)
- [Directory / package structure](#directory--package-structure)

-------

## CLI commands

```bash
alias pyg='less $(fd pyFD.py ~)'
alias pygg='less $(fd pyFD.py ~) | grep'
```

Check pyFD.py for python syntax.

-------

## Tips and Tricks

### Naming Convention

According to the Google Python Style Guide:

```python
module_name, package_name,
function_name, function_parameter_name,
ClassName, method_name
GLOBAL_CONSTANT_NAME
ExceptionName
global_var_name, instance_var_name, local_var_name
```

### Docstrings

```python
def func(abc, fds=4.6):
 # One-line Docstrings, note the "." at the end
 '''Do sth and return sth.'''
    
    # Multi-line Docstrings, Google Style (check also Sphinx, Numpy style)
    '''Summary line.
    
    Args:
        a (int): the dog
        b (float): the duck
    
    Raises:
        RuntimeError: Out of dog.
        
    Returns:
        dick (float): the dick
    '''
    
    pass

print(func.__doc__)
help(func)
```

### Function annotations

```python
def func(abc:'int', fds:'float'=4.6) -> 'list':
    pass
print(func.__annotations__)
```

### PEP - Python Enhancement Proposal

- Type hinting: [docs](https://docs.python.org/3/library/typing.html)

-------

## Directory / package structure

```bash
root_folder
├── asdf.py
├── asdf1.py
├── packageA
│  ├── __init__.py
│  ├── a1.py
│  ├── a2.py
│  ├── sub_packageA
│  │  ├── __init__.py
│  │  ├── duck.py
│  │  └── duke.py
│  └── sub_packageB
│     └── __init__.py
└── packageB
   ├── b1.py
   └── b2.py
```

```import``` search through list of paths in ```sys.path```  
Importing package basically run package's ```__init__.py```  
Can import packageA, can't import packageB because there is no ```__init__.py``` in packageB (but ```__init__.py``` is no longer needed for Python 3.3+)

```import <package>
import <module>
from <package> import <module / subpackage / object>
from <module> import <object>
import packageA.subpackageA.duck
from . import sub_packageA
from .sub_packageA import duck
```

Accessible names

```python
dir(package_name / module_name)
```

File paths on Windows

```python
'C:\Users\jdoe'    # Wrong!
'C:\\Users\\jdoe'
r'C:\Users\duck' # raw string
```

-------
