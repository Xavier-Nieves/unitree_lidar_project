"""Common ROS 2 QoS profiles, node utilities, and helpers."""

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


# Sensor data QoS — best effort, volatile, small depth
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

# Reliable QoS — for commands and important messages
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Default QoS — keep last 10
DEFAULT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def wait_for_topic(node, topic_name: str, msg_type, timeout_sec: float = 10.0) -> bool:
    """Wait until a topic has at least one publisher.

    Args:
        node: ROS 2 node instance.
        topic_name: Name of the topic to wait for.
        msg_type: Message type (unused but kept for API clarity).
        timeout_sec: Maximum seconds to wait.

    Returns:
        True if topic found, False if timed out.
    """
    import time

    start = time.time()
    while time.time() - start < timeout_sec:
        topics = node.get_topic_names_and_types()
        for name, _ in topics:
            if name == topic_name:
                pubs = node.count_publishers(topic_name)
                if pubs > 0:
                    return True
        time.sleep(0.5)
    return False


def check_node_alive(node, target_node_name: str) -> bool:
    """Check if a specific ROS 2 node is currently running.

    Args:
        node: ROS 2 node to query from.
        target_node_name: Name of the node to look for.

    Returns:
        True if the node is found in the node graph.
    """
    node_names = node.get_node_names()
    return target_node_name in node_names
