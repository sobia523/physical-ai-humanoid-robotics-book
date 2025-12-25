# Glossary of ROS 2 Terms

## A

### Action
A communication pattern in ROS 2 for goal-oriented tasks that may take a long time to complete. Actions include feedback during execution and result reporting upon completion.

### Architecture
The structural design of a robotic system, including the organization of nodes, topics, services, and their relationships.

## B

### Build System
Tools and processes used to compile and link ROS 2 packages, typically using colcon for building workspaces.

## C

### Client
A node that sends requests to a service and waits for a response.

### colcon
The build tool used in ROS 2 for building packages in a workspace.

### Communication
The exchange of data between nodes using topics, services, or actions.

## D

### DDS (Data Distribution Service)
The middleware technology that underlies ROS 2's communication system, providing publish-subscribe and request-reply patterns.

### Debugging
The process of identifying and resolving errors or issues in software or robotic systems.

## E

### Environment Variables
System variables that configure the ROS 2 environment, such as ROS_DOMAIN_ID or RMW_IMPLEMENTATION.

## F

### Framework
A platform for developing robotic applications, providing libraries, tools, and conventions.

## G

### Gazebo
A robotics simulation environment that provides high-fidelity physics simulation and 3D rendering.

## H

### Hardware Abstraction
Software layer that provides a consistent interface to different hardware components.

## I

### Interface
In ROS 2, a definition of messages, services, or actions that can be used for communication.

### Infrastructure
The underlying systems and tools that support ROS 2 development and deployment.

## L

### Launch File
A configuration file that specifies which nodes and parameters to start in a ROS 2 system.

## M

### Message
A data structure used to exchange information between ROS 2 nodes through topics.

### Middleware
Software that provides common services and capabilities to applications beyond what's offered by the operating system, specifically ROS 2 in this context.

### Node
A process that performs computation in ROS. Nodes are the fundamental building blocks of a ROS program.

## P

### Package
A modular unit of software in ROS that contains libraries, nodes, and other resources.

### Publisher
A node that sends messages to a topic in the publish-subscribe communication pattern.

## Q

### QoS (Quality of Service)
A set of policies that define the behavior of message delivery in terms of reliability, durability, liveliness, and deadline.

## R

### rclpy
The Python client library for ROS 2, allowing Python programs to interact with ROS 2 systems.

### Robot Operating System (ROS)
A flexible framework for writing robotic software that provides services designed for a heterogeneous computer cluster.

### ROS 2
The second generation of the Robot Operating System, designed for production environments with improved security and real-time capabilities.

### Runtime
The execution environment where ROS 2 nodes operate and communicate.

## S

### Service
A communication pattern in ROS 2 that provides a request-response interaction between nodes.

### Simulation
The use of software to model the behavior of a real-world robotic system.

### Subscriber
A node that receives messages from a topic in the publish-subscribe communication pattern.

## T

### Topic
A communication channel through which ROS 2 nodes exchange messages of a specific type.

### Tool
A software application that assists in the development, debugging, or operation of ROS 2 systems.

## U

### URDF (Unified Robot Description Format)
An XML format for representing robot descriptions, including links, joints, and visual properties.

## V

### Visualization
Tools and techniques for displaying ROS 2 data and robot states in a human-readable format.

## W

### Workspace
A directory containing ROS 2 packages that are built together using colcon.