#   brief about project - ROS2 action mux
-- A Ros2 project based on preemption of lower priority goals when we receive higher priority ones by using action client and action server

This project implements a ROS 2 Action Server that selects between multiple publishers based on the frequency of messages received. It uses a custom action definition and supports goal preemption for high-frequency message handling.

Below is the complete project structure of action_mux implemented with action server , action client , 3 publishers and a generic subscriber

ros2_ws2/
|--README.md
├-- src/
|   |-- action_mux_client/
│       |-- CMakeLists.txt/
│       |-- include/
|               |--action_mux_client/
│       |-- package.xml/
│       |-- src/
|               |--action_client.cpp/
|               |--publisher_1.cpp/
|               |--publisher_2.cpp/
|               |--publisher_3.cpp/
|   |-- action_mux_interfaces/
│       |-- CMakeLists.txt/
│       |-- include/
|               |--action_mux_interfaces
│       |-- package.xml/
|       |--msg/
|               |--StampedMessage.cpp/
|       |--action/
|               |--MuxAction.cpp
│       |-- src/
|    |-- action_mux_server/
│       |-- CMakeLists.txt/
│       |-- include/
|               |--action_mux_server/
│       |-- package.xml/
│       |-- src/
|               |--action_server.cpp/
|               |--generic_subscriber.cpp/   
|     |--install
|     |--build
|     |--log 
|-- build/       # Ignored in Git
|-- install/     # Ignored in Git
|-- log/         # Ignored in Git

