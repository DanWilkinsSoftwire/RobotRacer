import json
import os

class MazeMap:
    """Manages the topological graph of the maze, consisting of:
    - nodes: dict mapping node_id -> {"label": label, "x": x, "y": y}
    - edges: list of dicts mapping path details between nodes.
    """
    def __init__(self):
        self.nodes = {}
        self.edges = []

    def get_next_node_id(self):
        """Generates a unique node identifier (e.g., node_0, node_1)."""
        idx = 0
        while f"node_{idx}" in self.nodes:
            idx += 1
        return f"node_{idx}"

    def add_node(self, label, x, y, node_id=None):
        """Adds a node to the graph at (x, y) with a label. Generates a new ID if not provided."""
        if node_id is None:
            node_id = self.get_next_node_id()
        self.nodes[node_id] = {
            "label": label,
            "x": round(x, 3),
            "y": round(y, 3)
        }
        return node_id

    def add_edge(self, from_node, to_node, action, duration, distance):
        """Adds a directed edge between two nodes representing a travel segment."""
        self.edges.append({
            "from": from_node,
            "to": to_node,
            "action": action,
            "duration": round(duration, 3),
            "distance": round(distance, 3)
        })

    def get_node(self, node_id):
        """Gets a node's details."""
        return self.nodes.get(node_id)

    def save_to_file(self, file_path):
        """Saves the map to a pretty-printed JSON file."""
        data = {
            "nodes": self.nodes,
            "edges": self.edges
        }
        # Ensure directory exists
        dir_name = os.path.dirname(file_path)
        if dir_name and not os.path.exists(dir_name):
            os.makedirs(dir_name, exist_ok=True)
            
        with open(file_path, "w") as f:
            json.dump(data, f, indent=4, sort_keys=True)

    def load_from_file(self, file_path):
        """Loads a map from a JSON file."""
        if not os.path.exists(file_path):
            return False
        with open(file_path, "r") as f:
            data = json.load(f)
            self.nodes = data.get("nodes", {})
            self.edges = data.get("edges", [])
        return True
