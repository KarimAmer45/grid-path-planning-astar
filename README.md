# Grid Path Planning: BFS, A*, and Weighted A*

Python implementation of grid-based path planning algorithms for a 4-connected occupancy map.

## Contents

- `scripts/assignment_1_astar.py` implements:
  - uninformed uniform-cost search with unit edge costs, equivalent to BFS on this grid
  - A* search with a Manhattan-distance heuristic
  - a faster weighted A* variant for quicker planning on large maps
- `scripts/example_test_case.py` runs the planners on a toy grid and on `map.png`.
- `map.png` is the example occupancy grid used by the larger smoke test.

## Assumptions

- Free cells have values greater than `0.9`; occupied cells are below that threshold.
- The robot moves only in the four cardinal directions.
- Every valid move has unit cost.
- If no valid path exists, the planner returns an empty path.
- The weighted A* planner trades optimality for speed by weighting the heuristic.

## Run

```bash
cd scripts
python example_test_case.py
```

The example script writes path visualizations for each planner.
