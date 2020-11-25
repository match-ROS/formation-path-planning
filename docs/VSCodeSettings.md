# c_cpp_properties.json in .vscode folder
{
    "configurations": [
        {
            "browse": {
                "databaseFilename": "",
                "limitSymbolsToIncludedHeaders": true
            },
            "includePath": [
                "/home/hlurz/catkin_ws/devel/include/**",
                "/opt/ros/melodic/include/**",
                "/home/hlurz/catkin_ws/src/formation-path-planning/relaxed_a_star/include/**",
                "/home/hlurz/catkin_ws/src/cooperative-object-pick-up/**",
                "/usr/include/**"
            ],
            "name": "ROS",
            "intelliSenseMode": "gcc-x64",
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++14",
            "configurationProvider": "b2.catkin_tools"
        }
    ],
    "version": 4
}

# launch.json in .vscode folder
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS: Attach",
            "type": "ros",
            "request": "attach"
        }
    ]
}

# settings.json in .vscode folder
{
    "python.autoComplete.extraPaths": [
        "/home/hlurz/catkin_ws/devel/lib/python2.7/dist-packages",
        "/opt/ros/melodic/lib/python2.7/dist-packages",
        "./geometry_info"
    ],
    "cmake.configureOnOpen": false,
    "python.analysis.extraPaths": [
        "/home/hlurz/catkin_ws/devel/lib/python2.7/dist-packages",
        "/opt/ros/melodic/lib/python2.7/dist-packages",
        "./src/cooperative-object-pick-up/optimal_position_calculator/geometry_info"
    ],
    "python.linting.enabled": true //
}

# tasks.json in .vscode folder
{
	"version": "2.0.0",
	"tasks": [
		{
            "label": "catkin_make",
            "type": "shell",
            "command": "catkin_make",
            "args": [
                "-j4",
                "-DCMAKE_BUILD_TYPE=Debug",
                "-DCMAKE_EXPORT_COMPILE_COMMANDS=1",
                "-DCMAKE_CXX_STANDARD=14"
            ],
            "problemMatcher": [],
            "group": {
                "kind": "build",
                "isDefault": true
            }
        }
	]
}