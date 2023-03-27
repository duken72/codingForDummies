# awk & gawk

`awk` and `gawk` are tools for data search and manipulation

You can do simple tasks like:

- Print content, just like with `cat`
- Search for content, just like with `grep`
- Modify content like `sed`

More advance functionalities of `awk` and `gawk`:

- Search for specific part of the content and print out /
    E.g.: search for line with the first element is/is not something, print out specific other part

    ```bash
    gawk '$1 == "string" { print $2 } ' input.txt
    gawk '$1 ~ "string" { print $2 } ' input.txt
    gawk '$1 != "string" { print $2 } ' input.txt
    ```

- Take in output from other cmd

    ```bash
    ps | gawk '{print $2 "\t" $4}'
    ```

- Pipe output to other cmd
  
    ```bash
    pacman -S $(gawk '{print}' input.txt)
    ```

- Consider different field seperator

    ```bash
    gawk -F ":" '{print $1, $6}' /etc/passwd
    ```

- Find lines containing specified substring:

    ```bash
    gawk '/substring/ {print}' input.txt
    ```

- Replace substring

    ```bash
    gawk '{gsub("oldSubstring","newSubstring"); print}' input.txt
    ```
