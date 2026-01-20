print("Hi"
import heapq
import sys

class Graph:
    def __init__(self, vertices, directed=False):
        self.V = vertices
        self.graph = [[] for _ in range(vertices)]
        self.directed = directed
        self.vertex_map = {}  # Maps vertex names to indices
        self.index_to_vertex = {}  # Maps indices to vertex names

    def add_vertex_mapping(self, vertex_name, index):
        """Map vertex name to index and vice versa"""
        self.vertex_map[vertex_name] = index
        self.index_to_vertex[index] = vertex_name

    def add_edge(self, u, v, w):
        """Add an edge from u to v with weight w"""
        u_idx = self.vertex_map[u]
        v_idx = self.vertex_map[v]

        self.graph[u_idx].append((v_idx, w))
        if not self.directed:
            self.graph[v_idx].append((u_idx, w))

    def dijkstra(self, src):
        """Find shortest paths from source using Dijkstra's algorithm"""
        src_idx = self.vertex_map[src]

        # Initialize distances and predecessors
        dist = [float('inf')] * self.V
        prev = [-1] * self.V
        dist[src_idx] = 0

        # Priority queue: (distance, vertex_index)
        pq = [(0, src_idx)]

        while pq:
            current_dist, u = heapq.heappop(pq)

            # If we found a better path already, skip
            if current_dist > dist[u]:
                continue

            # Explore neighbors
            for v, weight in self.graph[u]:
                distance = current_dist + weight

                # If found shorter path to v
                if distance < dist[v]:
                    dist[v] = distance
                    prev[v] = u
                    heapq.heappush(pq, (distance, v))

        return dist, prev

    def get_path(self, prev, target_idx):
        """Reconstruct path from source to target using predecessors"""
        path = []
        current = target_idx

        # Backtrack from target to source
        while current != -1:
            path.append(self.index_to_vertex[current])
            current = prev[current]

        # Reverse to get path from source to target
        return path[::-1]

    def print_shortest_paths(self, src):
        """Print all shortest paths from source"""
        print(f"\nShortest Path Tree from vertex {src}:")
        print("=" * 60)

        dist, prev = self.dijkstra(src)

        for i in range(self.V):
            vertex_name = self.index_to_vertex[i]
            if vertex_name == src:
                print(f"Source {src}: distance = 0, path = [{src}]")
                continue

            if dist[i] == float('inf'):
                print(f"Vertex {vertex_name}: No path exists")
            else:
                path = self.get_path(prev, i)
                path_str = " -> ".join(path)
                print(f"Vertex {vertex_name}: distance = {dist[i]}, path = {path_str}")


def read_graph_from_file(filename):
    """Read graph data from input file"""
    try:
        with open(filename, 'r') as file:
            lines = file.readlines()

        # Parse first line: number_of_vertices number_of_edges graph_type
        first_line = lines[0].strip().split()
        num_vertices = int(first_line[0])
        num_edges = int(first_line[1])
        graph_type = first_line[2].upper()
        directed = (graph_type == 'D')

        # Create graph
        graph = Graph(num_vertices, directed)

        # Process edges and build vertex mapping
        vertex_counter = 0
        vertex_mapping = {}

        # First pass: collect all unique vertices
        for i in range(1, 1 + num_edges):
            parts = lines[i].strip().split()
            u, v = parts[0], parts[1]

            if u not in vertex_mapping:
                vertex_mapping[u] = vertex_counter
                graph.add_vertex_mapping(u, vertex_counter)
                vertex_counter += 1
            if v not in vertex_mapping:
                vertex_mapping[v] = vertex_counter
                graph.add_vertex_mapping(v, vertex_counter)
                vertex_counter += 1

        # Second pass: add edges
        for i in range(1, 1 + num_edges):
            parts = lines[i].strip().split()
            u, v, weight = parts[0], parts[1], int(parts[2])
            graph.add_edge(u, v, weight)

        # Get source vertex from last line
        source_vertex = lines[-1].strip()

        return graph, source_vertex

    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return None, None
    except Exception as e:
        print(f"Error reading file: {e}")
        return None, None


def create_sample_input_file():
    """Create a sample input file with the provided data"""
    sample_data = """6 10 U
A B 1
A C 2
B C 1
B D 3
B E 2
C D 1
C E 2
D E 4
D F 3
E F 3
A"""

    with open('graph_input.txt', 'w') as file:
        file.write(sample_data)

    print("Sample input file 'graph_input.txt' created successfully!")
    return 'graph_input.txt'


def main():
    # Option 1: Use the provided sample data
    print("Creating sample input file...")
    filename = create_sample_input_file()

    # Read graph from file
    graph, source_vertex = read_graph_from_file(filename)

    if graph and source_vertex:
        print(f"Graph loaded successfully!")
        print(f"Vertices: {graph.V}")
        print(f"Directed: {graph.directed}")
        print(f"Source vertex: {source_vertex}")

        # Find and print shortest paths
        graph.print_shortest_paths(source_vertex)

    print("\n" + "=" * 60)

    # Option 2: Use custom input file
    custom_file = input("\nEnter custom input filename (or press Enter to skip): ").strip()
    if custom_file:
        graph, source_vertex = read_graph_from_file(custom_file)
        if graph and source_vertex:
            print(f"Graph loaded successfully from {custom_file}!")
            graph.print_shortest_paths(source_vertex)


def display_graph_info(graph):
    """Display graph information for verification"""
    print("\nGraph Structure:")
    print("-" * 30)
    for i in range(graph.V):
        vertex_name = graph.index_to_vertex[i]
        neighbors = []
        for neighbor_idx, weight in graph.graph[i]:
            neighbors.append(f"{graph.index_to_vertex[neighbor_idx]}({weight})")
        print(f"{vertex_name}: {', '.join(neighbors)}")


if __name__ == "__main__":
    main()
