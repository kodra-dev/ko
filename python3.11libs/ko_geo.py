import hou
from collections import deque

def are_edges_connected(edges: list[hou.Edge]) -> bool:
    """
    Checks if a list of hou.Edge objects from the same geometry are all connected,
    forming a single contiguous island.

    This is achieved by performing a breadth-first search (BFS) graph traversal.

    Args:
        edges: A list of hou.Edge objects.

    Returns:
        True if all edges are connected, False otherwise. Returns True for an
        empty list or a list with a single edge.
    """
    if not edges or len(edges) < 2:
        return True

    # --- 1. Build Adjacency List and gather all points ---
    adj_list = {}
    all_points_in_selection = set()

    for edge in edges:
        # Get the two points for the current edge
        pts = edge.points() # Returns a tuple of two hou.Point objects
        p0_num = pts[0].number()
        p1_num = pts[1].number()

        # Add points to the set of all unique points in the selection
        all_points_in_selection.add(p0_num)
        all_points_in_selection.add(p1_num)

        # Populate the adjacency list for an undirected graph
        adj_list.setdefault(p0_num, []).append(p1_num)
        adj_list.setdefault(p1_num, []).append(p0_num)

    # --- 2. Perform Breadth-First Search (BFS) Traversal ---
    # Start the traversal from an arbitrary point in the selection
    start_point = next(iter(all_points_in_selection))
    
    q = deque([start_point])
    visited = {start_point}

    while q:
        current_point = q.popleft()
        
        # Check neighbors of the current point
        for neighbor in adj_list.get(current_point, []):
            if neighbor not in visited:
                visited.add(neighbor)
                q.append(neighbor)

    # --- 3. Compare visited points with all points in the selection ---
    # If the set of visited points is the same as all points in the
    # edge selection, the graph is connected.
    return all_points_in_selection == visited