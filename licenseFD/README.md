# License For Dummies

## Table of Content

<!-- vim-markdown-toc GFM -->

* [Basics](#basics)
* [Documentations](#documentations)
* [ROS2](#ros2)
* [Corporate Stuffs](#corporate-stuffs)

<!-- vim-markdown-toc -->

---

## Basics

What is it, and what are different types?

- [Open Source Licenses](https://gist.github.com/nicolasdao/a7adda51f2f185e8d2700e1573d8a633)
- [MIT vs. Apache vs. GPL](https://exygy.com/blog/which-license-should-i-use-mit-vs-apache-vs-gpl/)

---

## Documentations

Keep an updated [`3rd-party-licenses.txt`](3rd-party-licenses.txt) around.

---

## ROS2

```bash
ament_copyright -h
ament_copyright --add-missing "<name copyright holder>" bsd_3clause --verbose .
ament_copyright --list-licenses
```

---

## Corporate Stuffs

\#not_so_basic_any_more

There are 3 classes of licenses, in simple words:

- **Permissive OSS Licenses**: can be used for commercial purposes with ease
  > “do whatever you want with this, just don’t sue me.”
- **Weak Copyleft licenses**: be careful when using, should ask someone more experienced about it
- **Strong Copyleft licenses**: is a turnoff for corporations
  > “I open sourced my code, so you should too”

Go to [tldrlegal.com](https://tldrlegal.com/) to check the license information.

1. Permissive OSS Licenses:

   - MIT License (Expat)
   - Apache License 2.0 (Apache-2.0)
   - BSD 3-Clause License (Revised)
   - BSD 2-Clause License (FreeBSD/Simplified)
   - ISC License
   - BSD 0-Clause License (0BSD)
   - Universal Permissive License 1.0 (UPL-1.0)
   - The JSON License
   - Free Public License 1.0.0
   - The Don't Ask Me About It License
   - Very Simple Public License (VSPL)
   - Simple non code license (SNCL)
   - The Spice Software License Version 1.1 (Spice-1.1)
   - IBM PowerPC Initialization and Boot Software (IBM-pibs)
   - etc.

2. Weak Copyleft licenses: Depend on usage, be careful though

   - GNU Lesser General Public License v3 (LGPL-3.0)
   - GNU Lesser General Public License v2.1 (LGPL-2.1)
   - etc.

3. Strong Copyleft licenses:
   - GNU General Public License v3 (GPL-3)
   - GNU General Public License v2.0 (GPL-2.0)
   - Microsoft Reciprocal License (Ms-RL)
   - etc.
