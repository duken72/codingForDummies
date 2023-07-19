# ROS msg

- `msg`: `msg` files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.
- Common data types:
    - `int8`, `int16`, `int32`, `int64`
    - `float32`, `float64`
    - `string`
    - time, duration
    - other `msg` files
    - variable-length array[], fixed-length array[C]

## Creating a `msg`

1. Create `.msg` file in `/msg` folder: e.g. `packageName/msg/Name.msg`
    
    ```txt
    Header header
    string first_name
    string last_name
    uint8 age
    uint32 score
    ```
    
2. Add dependencies to `package.xml` in the package root `src/packageName`

    ```xml
    <build_depend>message_generation</build_depend>
    <exec_depend>message_runtime</exec_depend>
    ```
3. Add dependencies to `CMakeLists.txt` in package root `src/packageName`
    
    ```xml
    find_package(catkin REQUIRED COMPONENTS
        ...
        message_generation
        ...
    )

    catkin_package(
        ...
        CATKIN_DEPENDS message_runtime
        ...
    )

    add_message_files(
        FILES
        Name.msg
    )

    generate_messages(
        DEPENDENCIES
        std_msgs
    )
    ```

4. Rebuild package: `catkin_make` at `/catkin_ws`

## Using `rosmsg`

- Show message details:

```bash
rosmsg show [msgType] 
# msgType = packageName/nameOfMessage or just nameOfMessage
# ex: rosmsg show beginner_tutorials/Num -> int64 num
```

- List messages in a package:

```bash
rosmsg package [packageName]
```
