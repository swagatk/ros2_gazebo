#!/usr/bin/env python3
import subprocess
import time
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('graph_exporter')
    
    print("[INFO] Waiting for ROS 2 DDS discovery to detect active nodes...")
    
    # Discovery loop: wait up to 5 seconds for other nodes to register
    discovered_nodes = []
    start_time = time.time()
    while time.time() - start_time < 5.0:
        rclpy.spin_once(node, timeout_sec=0.3)
        raw_nodes = node.get_node_names_and_namespaces()
        discovered_nodes = [
            (name, ns) for name, ns in raw_nodes if 'graph_exporter' not in name
        ]
        if discovered_nodes:
            # Nodes found; give DDS an extra moment to populate all topic endpoints
            time.sleep(1.0)
            break

    node_names = node.get_node_names_and_namespaces()
    topics = node.get_topic_names_and_types()

    print(f"[INFO] Discovered {len(discovered_nodes)} node(s) and {len(topics)} topic(s).")
    
    if not discovered_nodes:
        print("[WARN] No active ROS 2 nodes found in this environment.")
        print("[WARN] Please ensure your simulation launch is active and check `ros2 node list`.")

    dot_lines = [
        'digraph ROS2Graph {',
        '  rankdir=LR;',
        '  node [shape=box, style="filled,rounded", fontname="Helvetica"];',
        '  edge [fontname="Helvetica", fontsize=10];'
    ]

    # Add active nodes
    for name, ns in node_names:
        full_name = f"{ns}/{name}".replace('//', '/')
        if 'graph_exporter' in full_name:
            continue
        dot_lines.append(f'  "{full_name}" [fillcolor="#D4E6F1", color="#2980B9"];')

    # Add active topics and edges
    for topic, _ in topics:
        # Ignore high-frequency internal topics to keep the diagram clean
        if topic in ['/clock', '/parameter_events', '/rosout']:
            continue

        dot_lines.append(f'  "{topic}" [shape=ellipse, fillcolor="#FCF3CF", color="#F39C12"];')

        # Publishers -> Topic
        publishers = node.get_publishers_info_by_topic(topic)
        for pub in publishers:
            pub_node = f"{pub.node_namespace}/{pub.node_name}".replace('//', '/')
            dot_lines.append(f'  "{pub_node}" -> "{topic}";')

        # Topic -> Subscribers
        subscribers = node.get_subscriptions_info_by_topic(topic)
        for sub in subscribers:
            sub_node = f"{sub.node_namespace}/{sub.node_name}".replace('//', '/')
            dot_lines.append(f'  "{topic}" -> "{sub_node}";')

    dot_lines.append('}')
    dot_output = '\n'.join(dot_lines)

    dot_filepath = '/tmp/ros2_graph.dot'
    with open(dot_filepath, 'w') as f:
        f.write(dot_output)

    node.destroy_node()
    rclpy.shutdown()

    # Convert DOT to PDF using system graphviz
    try:
        subprocess.run(['dot', '-Tpdf', dot_filepath, '-o', 'ros2_graph.pdf'], check=True)
        print("Successfully generated: ros2_graph.pdf")
    except FileNotFoundError:
        print(f"Graph DOT written to {dot_filepath}. Install graphviz (sudo apt install graphviz) to build PDF.")
    except Exception as e:
        print(f"Failed to generate PDF: {e}")

if __name__ == '__main__':
    main()