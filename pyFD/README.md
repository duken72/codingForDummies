# Basics

[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/duken72/codingForDummies.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/duken72/codingForDummies/context:python)

## Table of Contents

- [CLI commands](#cli-commands)
- [Directory / package structure](#directory--package-structure)
- [Style Guide](#style-guide)
  - [Naming Convention](#naming-convention)
  - [Docstrings](#docstrings)
  - [Function annotations](#function-annotations)
- [Best practices, tips and tricks](#best-practices-tips-and-tricks)
  - [PEP - Python Enhancement Proposal](#pep---python-enhancement-proposal)
  - [Iterable, iterator and generator](#iterable-iterator-and-generator)

-------

## CLI commands

```bash
alias pyg='less $(fd pyFD.py ~)'
alias pygg='less $(fd pyFD.py ~) | grep'
```

Check pyFD.py for python syntax.

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

```python
import <package>
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

## Style Guide

### Naming Convention

As usual when it comes to coding style, stick with what your company is using. If it's a green field project, define yourself that stick with one.

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

Check [type hints](https://docs.python.org/3/library/typing.html) for more (PEP483, PEP 484).

```python
def func(abc:'int', fds:'float'=4.6) -> 'list':
    pass
print(func.__annotations__)
```

-------

## Best practices, tips and tricks

### PEP - Python Enhancement Proposal

- Type hinting: [docs](https://docs.python.org/3/library/typing.html)

### Iterable, iterator and generator

Source: [stackoverflow1](https://stackoverflow.com/questions/231767/what-does-the-yield-keyword-do), [stackoverflow2](https://stackoverflow.com/questions/2776829/difference-between-pythons-generators-and-iterators).

Everything you can use `for... in...` on is an `iterable`; lists, strings, files, etc. Reading its items one by one is called `iteration`. Any object, whose class is `iterable`, is an `interator`, which has a `__next__` method.

`generator`s are `iterator`s, but not vice versa. You can only iterate over once. They do not store all the values in memory, **they generate the values on the fly**.

`yield` is the keywork used to replace `return` in `generator`.

```python
# An iterable from list comprehension
mylist = [x*x for x in range(3)]
# A generator, simply use () instead of []
mygenerator = (x*x for x in range(3))

def create_generator():
    mylist = range(3)
    for i in mylist:
        # Use yield instead of return
        yield i*i
```

Read the above source for more in depth explanation.

In many cases, generators are memory efficient and faster. In other cases, you get stuck in an infinite loop.

-------
