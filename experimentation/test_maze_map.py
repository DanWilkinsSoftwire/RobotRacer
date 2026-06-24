import os
from maze_map import MazeMap

def test_maze_map():
    map_file = "test_map.json"
    if os.path.exists(map_file):
        os.remove(map_file)
        
    m = MazeMap()
    
    # Test adding nodes
    n0 = m.add_node("start", 0, 0)
    n1 = m.add_node("branch_left", 0, 1.5)
    n2 = m.add_node("cross_or_multi", -1.0, 1.5)
    
    assert n0 == "node_0", f"Expected node_0, got {n0}"
    assert n1 == "node_1", f"Expected node_1, got {n1}"
    assert n2 == "node_2", f"Expected node_2, got {n2}"
    
    # Test adding edges
    m.add_edge(n0, n1, "start", 3.2, 1.5)
    m.add_edge(n1, n2, "turn_left", 2.1, 1.0)
    
    # Test saving
    m.save_to_file(map_file)
    assert os.path.exists(map_file), "Map file was not created!"
    
    # Test loading
    m2 = MazeMap()
    success = m2.load_from_file(map_file)
    assert success, "Failed to load map file!"
    
    # Verify loaded contents
    assert m2.nodes["node_0"]["label"] == "start"
    assert m2.nodes["node_1"]["x"] == 0
    assert m2.nodes["node_1"]["y"] == 1.5
    assert m2.nodes["node_2"]["label"] == "cross_or_multi"
    assert len(m2.edges) == 2
    assert m2.edges[0]["from"] == "node_0"
    assert m2.edges[0]["to"] == "node_1"
    assert m2.edges[0]["action"] == "start"
    assert m2.edges[1]["action"] == "turn_left"
    
    print("All tests passed successfully!")
    
    # Clean up
    os.remove(map_file)

if __name__ == "__main__":
    test_maze_map()
