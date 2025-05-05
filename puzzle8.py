import heapq
import time
from collections import deque

class Puzzle:
    def __init__(self, state, parent=None, move=None, depth=0):
        self.state = state
        self.parent = parent
        self.move = move
        self.depth = depth
        self.cost = 0

    def __eq__(self, other):
        return self.state == other.state

    def __lt__(self, other):
        return self.cost < other.cost

    def __hash__(self):
        return hash(tuple(self.state))

    def print_path(self):
        if self.parent:
            self.parent.print_path()
        print(f"Move {self.move} -> {self.state}")

    def is_goal(self):
        return self.state == [1, 2, 3, 4, 5, 6, 7, 8, 0]

    def get_blank_pos(self):
        return self.state.index(0)

    def get_children(self):
        children = []
        blank_pos = self.get_blank_pos()
        row, col = blank_pos // 3, blank_pos % 3

        moves = []
        if row > 0: moves.append(-3)  # Up
        if row < 2: moves.append(3)   # Down
        if col > 0: moves.append(-1)  # Left
        if col < 2: moves.append(1)   # Right

        for move in moves:
            new_state = self.state.copy()
            new_pos = blank_pos + move
            new_state[blank_pos], new_state[new_pos] = new_state[new_pos], new_state[blank_pos]
            children.append(Puzzle(new_state, self, move, self.depth + 1))

        return children

def bfs(start):
    visited = set()
    queue = deque([start])
    nodes_expanded = 0

    while queue:
        current = queue.popleft()
        nodes_expanded += 1

        if current.is_goal():
            return current, nodes_expanded

        for child in current.get_children():
            if tuple(child.state) not in visited:
                visited.add(tuple(child.state))
                queue.append(child)

    return None, nodes_expanded

def greedy(start, heuristic):
    visited = set()
    heap = []
    heapq.heappush(heap, (heuristic(start.state), start))
    nodes_expanded = 0

    while heap:
        _, current = heapq.heappop(heap)
        nodes_expanded += 1

        if current.is_goal():
            return current, nodes_expanded

        for child in current.get_children():
            if tuple(child.state) not in visited:
                visited.add(tuple(child.state))
                heapq.heappush(heap, (heuristic(child.state), child))

    return None, nodes_expanded

def astar(start, heuristic):
    visited = set()
    heap = []
    heapq.heappush(heap, (heuristic(start.state) + start.depth, start))
    nodes_expanded = 0

    while heap:
        _, current = heapq.heappop(heap)
        nodes_expanded += 1

        if current.is_goal():
            return current, nodes_expanded

        for child in current.get_children():
            if tuple(child.state) not in visited:
                visited.add(tuple(child.state))
                heapq.heappush(heap, (heuristic(child.state) + child.depth, child))

    return None, nodes_expanded

def misplaced_tiles(state):
    goal = [1, 2, 3, 4, 5, 6, 7, 8, 0]
    return sum(1 for i in range(9) if state[i] != goal[i] and state[i] != 0)

def manhattan_distance(state):
    goal_pos = {1: (0, 0), 2: (0, 1), 3: (0, 2),
                4: (1, 0), 5: (1, 1), 6: (1, 2),
                7: (2, 0), 8: (2, 1), 0: (2, 2)}
    distance = 0
    for i in range(9):
        if state[i] != 0:
            x, y = i // 3, i % 3
            goal_x, goal_y = goal_pos[state[i]]
            distance += abs(x - goal_x) + abs(y - goal_y)
    return distance

def solve_puzzle(initial_state):
    start = Puzzle(initial_state)
    
    print("Testing BFS...")
    start_time = time.time()
    solution, nodes = bfs(start)
    bfs_time = time.time() - start_time
    print(f"BFS: Time = {bfs_time:.4f}s, Nodes Expanded = {nodes}")

    print("\nTesting Greedy (Misplaced Tiles)...")
    start_time = time.time()
    solution, nodes = greedy(start, misplaced_tiles)
    greedy_time = time.time() - start_time
    print(f"Greedy: Time = {greedy_time:.4f}s, Nodes Expanded = {nodes}")

    print("\nTesting A* (Misplaced Tiles)...")
    start_time = time.time()
    solution, nodes = astar(start, misplaced_tiles)
    astar_misplaced_time = time.time() - start_time
    print(f"A* Misplaced: Time = {astar_misplaced_time:.4f}s, Nodes Expanded = {nodes}")

    print("\nTesting A* (Manhattan Distance)...")
    start_time = time.time()
    solution, nodes = astar(start, manhattan_distance)
    astar_manhattan_time = time.time() - start_time
    print(f"A* Manhattan: Time = {astar_manhattan_time:.4f}s, Nodes Expanded = {nodes}")

    if solution:
        print("\nSolution Path:")
        solution.print_path()
    else:
        print("No solution found")

if __name__ == "__main__":
    # Exemplo de estado inicial (0 representa o espaço vazio)
    initial_state = [1, 2, 3, 4, 0, 5, 6, 7, 8]  # Fácil (resolve em 2 movimentos)
    # initial_state = [1, 2, 3, 4, 5, 6, 0, 7, 8]  # Médio
    # initial_state = [0, 1, 3, 4, 2, 5, 7, 8, 6]  # Difícil
    
    solve_puzzle(initial_state)
