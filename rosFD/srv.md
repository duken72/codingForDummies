# ROS srv

- `srv`: an `srv` file describes a service. It is composed of two parts: a request and a response.

## Creating a `srv`

1. Create `.srv` file in `/srv` folder: e.g. `packageName/srv/Name.srv`
    
    ```txt
    # Request
    int64 a
    int64 b
    ---
    # Response
    int64 sum
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

    add_service_files(
        FILES
        Name.srv
    )
    ```

4. Rebuild package: `catkin_make` at `/catkin_ws`

## Using `rossrv`

- Show service details:

```bash
rossrv show <service type> 
```

- List services in a package:

```bash
rossrv package [packageName]
```
