# ROS 2 Communication

These notes explain how ROS 2 works compared to ROS 1, focusing on communication, discovery, and writing code.

In ROS 1, we used TCP(Transmission Control Protocol) which didn’t help in critical and real time system. To solve this, ROS 2 uses UDP and DDS.

---

## Communication in ROS 2

### DDS – Data Distribution Service
- ROS 2 uses DDS as its middleware for communication.
- DDS is built on the RTPS protocol (Real-Time Publish-Subscribe).
- It uses a data-centric approach, meaning communication focuses on sharing data.
- DDS provides end-to-end communication with built-in discovery and Quality of Service (QoS).

### Transport & Protocols
- DDS mainly uses UDP (User Datagram Protocol) for communication.
- It supports multicast, which sends messages efficiently to multiple nodes.

### Discovery in ROS 2
- ROS 1 uses a centralized ROS Master to let nodes find each other.
- ROS 2 replaces this with decentralized peer-to-peer discovery through DDS.
- Nodes discover each other automatically using multicast.
- For large systems, ROS 2 can use a Discovery Server (a centralized contact book) to speed up discovery.
- A ROS 2 daemon caches discovered participants to make command line tools faster. Restart it to refresh discovery data.

### Quality of Service (QoS)
For communication to be possible, QoS has to be compatible.
QoS allows us to have a higher control over performance and how we want our messages to be transported by customising settings.
- QoS settings control how messages are transmitted, e.g., reliability and message lifespan.
- Both publishers and subscribers must agree on QoS policies.
- This helps handle different data rates, reliability needs, and latency in robotics.

### Why ROS 2 Dropped the ROS Master
- ROS 1's master node was a single point of failure—if it crashed, the whole system stopped.
- The centralized model limited dynamic discovery and fault tolerance.
- ROS 2 uses DDS to allow decentralized, fault-tolerant communication without a master.

---

## Writing Code in ROS 2 vs ROS 1

### Client Libraries
- ROS 1: Separate C++ (`roscpp`) and Python (`rospy`) APIs, different from each other.
- ROS 2: One core library (`rcl`) with client libraries `rclcpp` (C++) and `rclpy` (Python), sharing similar APIs.

### Node Structure
- ROS 1: Nodes implemented independently without enforced structure.
- ROS 2: Nodes are classes that inherit from a `Node` base class, keeping functionality organized.

### Components
- ROS 1: Nodelets allowed multiple nodes in one executable but were separate tools.
- ROS 2: Multiple nodes (called components) can directly run in a single executable by default.

### Lifecycle Nodes (ROS 2 only)
- Nodes have managed states: unconfigured, inactive, active, and finalized.
- Useful for controlling when nodes start or stop working, improving safety and resource use.

### Launch Files
- ROS 1: Launching nodes uses XML `.launch` files.
- ROS 2: Python `.py` launch files are preferred, though XML is still supported.

---

## Communication Features: ROS 1 vs ROS 2

- **Master Node:** ROS 1 needs a central master; ROS 2 has none.
- **Parameters:** ROS 1 uses a central parameter server; ROS 2 has node-specific parameters with dynamic callbacks.
- **Services:** ROS 1 services are synchronous; ROS 2 services are asynchronous by default but can be synchronous.
- **Actions:** ROS 1 added actions later; ROS 2 has them as core features.
- **Messages:** Both use `.msg`, `.srv`, `.action` files, but ROS 2 adds namespaces.
- **QoS:** Supported only in ROS 2, allowing flexible communication handling.
- **Build System:** ROS 1 uses `catkin_make`; ROS 2 uses `colcon build`.

---

## ROS 2 QoS Settings (Brief Overview)

### 1. Reliability

- `RELIABLE`: Guarantees message delivery (resends if needed).
- `BEST_EFFORT`: Sends messages once without guarantees (may lose messages if the network is unstable).

We use RELIABLE when data is important.

We use BEST_EFFORT when speed matters more than accuracy.

### 2. History

- `KEEP_LAST`: Only keep the last `N` messages (default).
- `KEEP_ALL`: Keep all messages (use with caution; may use lots of memory).

KEEP_LAST is efficient. 

KEEP_ALL can be heavy on memory.

### 3. Depth

- Number of messages to store in the queue if they can’t be sent immediately.
- Only works with `KEEP_LAST`.

We increase if our system is dropping messages under load.

### 4. Durability

- `VOLATILE`: Messages are only available while the subscriber is active.
- `TRANSIENT_LOCAL`: Stores the last message so late subscribers can still receive it.

We use TRANSIENT_LOCAL when late subscribers need to catch up.

We use VOLATILE is default and fine for most real-time data.

### 5. Liveliness

- Determines how to detect if a publisher is still alive.
- Common types:
  - `AUTOMATIC`: ROS handles it (default).
  - `MANUAL_BY_TOPIC`: Publisher manually asserts it's alive.

We use default unless we need precise liveliness detection.

### 6. Deadline (Optional)

- Time allowed between messages before considered a failure.
- Helps detect dropped or delayed data.

### 7. Lifespan (Optional)

- How long a message remains valid for subscribers.
- After this duration, the message is discarded.

---

## Modifying nodes to add QoS profile

### QoS Code Used
I used the same following QoS settings for both the publisher and subscriber nodes.

```python
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
```

### Why These Settings?

- **RELIABLE**: Ensures messages are delivered. Needed because we don't want to miss any numbers.
- **KEEP_LAST**: Only the last few messages are stored — we care about recent data, not old.
- **depth=10**: Keeps up to 10 messages in the queue, allowing minor delays.


I didn't add any other QoS settings because:

**Durability (VOLATILE)** is default : it means the subscriber receives messages only while active, which is fine since both nodes run together.

**Liveliness** and **deadline** are useful for advanced use cases like monitoring node health or time-critical systems : we do not need it here.

The settings written ensure all numbers are received and processed correctly without wasting memory. So, the minimal QoS settings used are enough and appropriate for this example.

---

## Writing launch files 
In ROS 2, launch files are python scripts which are used to start multiple nodes using a single command. Instead of manually opening multiple terminals and running each node one by one, a launch file saves time by doing it at once.

- The launch file uses imports like `from launch import LaunchDescription` and `from launch_ros.actions import Node` to define which nodes to start.
- Then, a function called `generate_launch_description()` returns a list of nodes to launch.

I used it to launch both my publisher (the one publishing number to 'number'(topic)) and subscriber(the one which subscribes to 'number' and returns square of that number) together using one command, which helped me quickly test things like QoS.

For `ros2 launch` to find the launch file, we add the following line in our `setup.py` under `data_files`:
```
(os.path.join('share', package_name, 'launch'), ['launch/node_launch.py']),
```

---

## ROS 2 QoS Testing – Issues Faced & Observations

Issues that I encountered while testing QoS settings in ROS 2 using Python nodes, including commands used, expected behavior, and actual observations.

### 1. QoS Mismatch Doesn't Show Warnings

- **Command Used:**
  ```bash
  ros2 launch my_node node_launch.py
  ```

- **What It Usually Does:** 
  Starts the publisher and subscriber nodes. If QoS settings between them are mismatched (e.g., `RELIABLE` vs `BEST_EFFORT`), ROS 2 may print a warning or fail to deliver messages depending on severity.

- **What Happened for Me:** 
  Both nodes ran without any warning, even when `RELIABLE` publisher was connected to a `BEST_EFFORT` subscriber. No indication of QoS incompatibility appeared.

### 2. `ros2 topic info` Shows `UNKNOWN` 

- **Command Used:**
  ```bash
  ros2 topic info number --verbose
  ```
- **What It Usually Does:** 
  Displays detailed info about a topic, including the QoS profiles of publishers and subscribers (e.g., reliability, history, durability).
- **What Happened for Me:** 
  Some fields such as `History` showed `UNKNOWN` though I kept the setting as `KEEP_LAST`.

### 3. `ros2 doctor` Shows Compatibility as OK Even When QoS Mismatched

- **Command Used:**
  ```bash
  ros2 doctor --report
  ```
- **What It Usually Does:** 
  Checks for system-level issues including node connectivity, environment configuration, and basic QoS compatibility.
- **What Happened for Me:** 
  Even when QoS settings were intentionally mismatched between publisher and subscriber, the doctor report still said "Compatibility: OK".

Then after asking ChatGPT, I understood that these issues are limitations in current ROS 2 CLI tooling and introspection, especially when using Python (`rclpy`). Therefore, the absence of warnings or full QoS visibility does not mean that the QoS settings are incorrect or missing — they are active as long as they are correctly set in your node code.

---
## Summary

- ROS 2 removed the ROS Master to create a scalable, modular, and robust system. Using DDS provides decentralized discovery, flexible QoS, and better fault tolerance.
- I studied the basic differences between ROS1 and ROS2 in various aspects.
- I studied the basics of ROS2 Communication including concepts like UDP, DDS, peer to peer discovery, QoS, how to write launch files.
- I modified the publisher and the subscriber nodes - that I wrote before for publishing number where subscriber would take it and square it - by adding a suitable QoS profile to it.

---
## Resources that I used


