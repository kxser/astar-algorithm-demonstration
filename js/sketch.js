/*
A* PATHFINDING ALGORITHM 

=== ALGORITHM OVERVIEW ===
A* (A-star) is a graph traversal and pathfinding algorithm that finds the shortest path
between nodes.

=== KEY CONCEPTS ===
• f(n) = g(n) + h(n) - Total estimated cost of path through node n
• g(n) - Actual cost from start to node n
• h(n) - Heuristic estimate from node n to goal (Manhattan distance)
• Open List - Nodes to be evaluated (frontier)
• Closed List - Nodes already evaluated

=== STEP-BY-STEP ALGORITHM EXECUTION ===

1. INITIALIZATION PHASE:
   - Reset all grid cells to default state
   - Clear open and closed lists
   - Add start node to open list with f=0, g=0, h=heuristic(start, goal)
   - Set algorithm state flags and visualization parameters

2. MAIN LOOP (continues until path found or open list empty):
   a) SELECT CURRENT NODE:
      - Find node in open list with lowest f-score
      - Move current node from open list to closed list
      - Mark as "current" for visualization

   b) GOAL CHECK:
      - If current node == goal: reconstruct path and exit
      - Path reconstruction follows parent pointers backward

   c) NEIGHBOR EVALUATION:
      For each neighbor of current node:
      - Skip if neighbor is wall or in closed list
      - Calculate tentative g-score: current.g + distance(current, neighbor)
      
      If neighbor not in open list:
         - Add to open list
         - Set parent pointer to current node
         - Calculate g, h, and f scores
      
      Else if tentative g-score < neighbor.g:
         - Update neighbor with better path
         - Set new parent pointer
         - Recalculate g, h, and f scores

   d) VISUALIZATION UPDATE:
      - Color-code cells: green (open), red (closed), blue (path)
      - Apply configurable delay for real-time visualization
      - Update display to show algorithm progress


*/

// --- Constants and Variables ---
let cols, rows, cellSize;
let canvasWidth, canvasHeight;

let grid = [];
let startCell;
let goalCell;
let isDrawing = false;
let drawMode = 'obstacle';
let actionMode = 'draw';

let openList = [];
let closedList = [];
let finalPath = [];
let isSimulationRunning = false;
let simulationSpeed = 0.01; // Delay in milliseconds for visualization
let delayEnabled = true; // Toggle for delay on/off

// --- Grid Size Configuration ---
function calculateGridSize() {
  const containerPadding = 32; // 16px padding on each side from p-4
  const availableWidth = window.innerWidth - containerPadding;
  
  // Predetermined height (adjust this value to change grid height)
  const gridHeight = 800; // Fixed height in pixels (2x taller)
  
  // Calculate grid dimensions
  rows = 40; // Fixed number of rows for consistent height (2x more rows)
  cellSize = Math.floor(gridHeight / rows);
  cols = Math.floor(availableWidth / cellSize);
  
  // Ensure minimum grid size
  if (cols < 10) {
    cols = 10;
    cellSize = Math.floor(availableWidth / cols);
  }
  
  canvasWidth = cols * cellSize;
  canvasHeight = rows * cellSize;
}

// --- Helper Functions ---

function addNeighbors(cell) {
  const x = cell.x;
  const y = cell.y;

  // Add orthogonal neighbors
  if (x > 0) cell.neighbors.push(grid[y][x - 1]);
  if (x < cols - 1) cell.neighbors.push(grid[y][x + 1]);
  if (y > 0) cell.neighbors.push(grid[y - 1][x]);
  if (y < rows - 1) cell.neighbors.push(grid[y + 1][x]);
}

function setDrawMode(mode) {
  drawMode = mode;
  updateDrawButtons();
}

function setActionMode(mode) {
  actionMode = mode;
  updateActionButtons();
}

function toggleDelay() {
  delayEnabled = !delayEnabled;
  updateDelayButton();
}

function updateDrawButtons() {
  document.getElementById('draw-obstacles').classList.toggle('bg-gray-900', drawMode === 'obstacle');
  document.getElementById('draw-start').classList.toggle('bg-gray-900', drawMode === 'start');
  document.getElementById('draw-goal').classList.toggle('bg-gray-900', drawMode === 'goal');
}

function updateActionButtons() {
  document.getElementById('action-draw').classList.toggle('bg-gray-900', actionMode === 'draw');
  document.getElementById('action-erase').classList.toggle('bg-gray-900', actionMode === 'erase');
}

function updateDelayButton() {
  const button = document.getElementById('toggle-delay');
  button.textContent = delayEnabled ? 'Delay: On' : 'Delay: Off';
  button.classList.toggle('bg-gray-900', delayEnabled);
}

/*
Handles mouse interactions for drawing obstacles, setting start/goal positions, or erasing.
Converts mouse coordinates to grid coordinates and updates cell states based on current draw and action modes.
*/
function handleMouseAction() {
  let x = floor(mouseX / cellSize);
  let y = floor(mouseY / cellSize);

  if (x >= 0 && x < cols && y >= 0 && y < rows) {
    let cell = grid[y][x];

    if (actionMode === 'draw') {
      if (drawMode === 'obstacle') {
        if (cell !== startCell && cell !== goalCell) {
          cell.obstacle = true;
        }
      } else if (drawMode === 'start') {
        if (startCell) startCell.obstacle = false;
        startCell = cell;
        cell.obstacle = false;
      } else if (drawMode === 'goal') {
        if (goalCell) goalCell.obstacle = false;
        goalCell = cell;
        cell.obstacle = false;
      }
    } else {
      if (cell.obstacle) {
        cell.obstacle = false;
      }
    }
    
    clearAlgorithmState();
  }
}

// Add touch support for mobile devices
function touchStarted() {
  isDrawing = true;
  handleMouseAction();
  return false; // Prevent default touch behavior
}

function touchEnded() {
  isDrawing = false;
  return false;
}

function touchMoved() {
  if (isDrawing) {
    handleMouseAction();
  }
  return false;
}

function sleep(ms) {
  if (!delayEnabled) {
    return Promise.resolve(); // Return immediately if delay is disabled
  }
  return new Promise(resolve => setTimeout(resolve, ms));
}

/*
Main A* pathfinding algorithm implementation.
Initializes the algorithm state, processes cells from open list, evaluates neighbors,
and visualizes the search process in real-time with configurable delay.
*/
async function startSimulation() {
  if (isSimulationRunning) {
    return;
  }

  isSimulationRunning = true;
  document.getElementById('start-simulation').textContent = 'Running...';
  document.getElementById('start-simulation').disabled = true;
  
  for (let y = 0; y < rows; y++) {
    for (let x = 0; x < cols; x++) {
      const cell = grid[y][x];
      cell.g = Infinity;
      cell.h = 0;
      cell.f = Infinity;
      cell.parent = null;
    }
  }
  
  openList = [];
  closedList = [];
  finalPath = [];
  
  startCell.g = 0;
  startCell.h = manhattanDistance({x: startCell.x, y: startCell.y}, {x: goalCell.x, y: goalCell.y});
  startCell.f = startCell.g + startCell.h;
  openList.push(startCell);
  
  let iterations = 0;
  
  while (openList.length > 0 && isSimulationRunning) {
    iterations++;
    
    let currentCell = openList[0];
    let currentIndex = 0;
    
    for (let i = 1; i < openList.length; i++) {
      if (openList[i].f < currentCell.f) {
        currentCell = openList[i];
        currentIndex = i;
      }
    }
    
    openList.splice(currentIndex, 1);
    closedList.push(currentCell);
    
    await sleep(simulationSpeed);
    
    if (currentCell === goalCell) {
      reconstructPath(currentCell);
      break;
    }
    
    currentCell.neighbors.forEach(neighbor => {
      if (neighbor.obstacle || closedList.includes(neighbor)) {
        return;
      }
      
      const tentativeG = currentCell.g + 1;
      
      if (!openList.includes(neighbor)) {
        openList.push(neighbor);
      } else if (tentativeG >= neighbor.g) {
        return;
      }
      
      neighbor.parent = currentCell;
      neighbor.g = tentativeG;
      neighbor.h = manhattanDistance({x: neighbor.x, y: neighbor.y}, {x: goalCell.x, y: goalCell.y});
      neighbor.f = neighbor.g + neighbor.h;
    });
    
    if (iterations > 5000) {
      break;
    }
  }

  isSimulationRunning = false;
  document.getElementById('start-simulation').textContent = 'Start Simulation';
  document.getElementById('start-simulation').disabled = false;
}

function stopSimulation() {
  isSimulationRunning = false;
  document.getElementById('start-simulation').textContent = 'Start Simulation';
  document.getElementById('start-simulation').disabled = false;
}

function clearAlgorithmState() {
  stopSimulation();
  openList = [];
  closedList = [];
  finalPath = [];
  
  for (let y = 0; y < rows; y++) {
    for (let x = 0; x < cols; x++) {
      const cell = grid[y][x];
      cell.g = Infinity;
      cell.h = 0;
      cell.f = Infinity;
      cell.parent = null;
    }
  }
}

function mousePressed() {
  isDrawing = true;
  handleMouseAction();
}

function mouseReleased() {
  isDrawing = false;
}

function mouseDragged() {
  if (isDrawing) {
    handleMouseAction();
  }
}

// Handle window resize
function windowResized() {
  calculateGridSize();
  resizeCanvas(canvasWidth, canvasHeight);
  
  // Reinitialize grid with new dimensions
  grid = [];
  for (let y = 0; y < rows; y++) {
    let row = [];
    for (let x = 0; x < cols; x++) {
      row.push({
        x,
        y,
        obstacle: false,
        g: Infinity,
        h: 0,
        f: Infinity,
        parent: null,
        neighbors: [],
      });
    }
    grid.push(row);
  }

  // Add neighbors to each cell
  for (let y = 0; y < rows; y++) {
    for (let x = 0; x < cols; x++) {
      addNeighbors(grid[y][x]);
    }
  }

  // Reset start and goal positions
  startCell = grid[Math.floor(rows * 0.1)][Math.floor(cols * 0.1)];
  goalCell = grid[Math.floor(rows * 0.9)][Math.floor(cols * 0.9)];
  
  clearAlgorithmState();
}

function cellAt(x, y) {
  if (x < 0 || x >= cols || y < 0 || y >= rows) return null;
  return grid[y][x];
}

function cellType(cell) {
  if (!cell) return null;
  if (cell === startCell) return 'start';
  if (cell === goalCell) return 'goal';
  if (cell.obstacle) return 'obstacle';
  return 'empty';
}

function cellTypeAt(x, y) {
  return cellType(cellAt(x, y));
}

function cellInfoAt(x, y) {
  const c = cellAt(x, y);
  return c ? { x: c.x, y: c.y, type: cellType(c) } : null;
}

function cellAtScreen(px, py) {
  return cellAt(floor(px / cellSize), floor(py / cellSize));
}

function getStartAndGoalCoords() {
  return {
    start: startCell ? { x: startCell.x, y: startCell.y } : null,
    goal: goalCell ? { x: goalCell.x, y: goalCell.y } : null
  };
}

function reconstructPath(endCell) {
  finalPath = [];
  let current = endCell;
  
  while (current) {
    finalPath.push(current);
    current = current.parent;
  }
  
  finalPath.reverse();
}

function manhattanDistance({ x: x1, y: y1 }, { x: x2, y: y2 }) {
  return Math.abs(x1 - x2) + Math.abs(y1 - y2);
}

/*
Initializes the p5.js canvas and grid structure.
Creates the 2D grid of cells with A* properties, establishes neighbor relationships,
sets default start/goal positions, and configures UI event listeners.
*/
function setup() {
  calculateGridSize();
  let canvas = createCanvas(canvasWidth, canvasHeight);
  canvas.parent('canvas-container');

  // Initialize grid cells
  for (let y = 0; y < rows; y++) {
    let row = [];
    for (let x = 0; x < cols; x++) {
      row.push({
        x,
        y,
        obstacle: false,
        // A* properties
        g: Infinity,
        h: 0,
        f: Infinity,
        parent: null,
        neighbors: [],
      });
    }
    grid.push(row);
  }

  // Add neighbors to each cell
  for (let y = 0; y < rows; y++) {
    for (let x = 0; x < cols; x++) {
      addNeighbors(grid[y][x]);
    }
  }

  // Set start and goal with relative positioning
  startCell = grid[Math.floor(rows * 0.1)][Math.floor(cols * 0.1)];
  goalCell = grid[Math.floor(rows * 0.9)][Math.floor(cols * 0.9)];

  // Button listeners
  document.getElementById('start-simulation').addEventListener('click', (e) => {
    e.preventDefault();
    startSimulation();
  }, { passive: false });

  document.getElementById('draw-obstacles').addEventListener('click', (e) => {
    e.preventDefault();
    setDrawMode('obstacle');
  }, { passive: false });

  document.getElementById('draw-start').addEventListener('click', (e) => {
    e.preventDefault();
    setDrawMode('start');
  }, { passive: false });

  document.getElementById('draw-goal').addEventListener('click', (e) => {
    e.preventDefault();
    setDrawMode('goal');
  }, { passive: false });

  document.getElementById('action-draw').addEventListener('click', (e) => {
    e.preventDefault();
    setActionMode('draw');
  }, { passive: false });

  document.getElementById('action-erase').addEventListener('click', (e) => {
    e.preventDefault();
    setActionMode('erase');
  }, { passive: false });

  document.getElementById('toggle-delay').addEventListener('click', (e) => {
    e.preventDefault();
    toggleDelay();
  }, { passive: false });

  // Add touch event listeners for buttons as backup
  const buttonIds = ['start-simulation', 'draw-obstacles', 'draw-start', 'draw-goal', 'action-draw', 'action-erase', 'toggle-delay'];

  buttonIds.forEach(id => {
    const button = document.getElementById(id);
    if (button) {
      button.addEventListener('touchend', (e) => {
        e.preventDefault();
        e.stopPropagation();
        button.click();
      }, { passive: false });
    }
  });

  // Initialize button states
  updateDrawButtons();
  updateActionButtons();
  updateDelayButton();
}

/*
Main rendering loop that visualizes the grid and A* algorithm state.
Colors cells based on their role: start/goal, obstacles, open/closed lists, final path.
Optionally displays f, g, h values for debugging purposes.
*/
function draw() {
  background(50);

  // Draw grid
  for (let y = 0; y < rows; y++) {
    for (let x = 0; x < cols; x++) {
      const cell = grid[y][x];
      stroke(0);
      strokeWeight(1);
      
      // Color cells based on their state
      if (cell === startCell) {
        fill(0, 255, 0); // Bright green for start
      } else if (cell === goalCell) {
        fill(255, 0, 0); // Bright red for goal
      } else if (finalPath.includes(cell)) {
        fill(0, 128, 0);
      } else if (closedList.includes(cell)) {
        fill(255, 150, 150); // Light red for closed list (explored)
      } else if (openList.includes(cell)) {
         
        fill(255, 255, 0); 
      } else if (cell.obstacle) {
        fill(100); // Dark gray for obstacles
      } else {
        fill(255); // White for free space
      }
      
      rect(x * cellSize, y * cellSize, cellSize, cellSize);
      

      if (cellSize > 15 && cell.g !== Infinity && cell !== startCell && cell !== goalCell && !cell.obstacle) {
        fill(0);
        textAlign(CENTER, CENTER);
        textSize(Math.max(8, cellSize * 0.2));
        text(Math.round(cell.f), x * cellSize + cellSize/2, y * cellSize + cellSize/3);
      }
    }
  }
}



