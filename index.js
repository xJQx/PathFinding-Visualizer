// Chebyshev distance
const D = 1;
const D2 = 1;

// variables for A* Pathfinding Algorithm
let OPEN = [];
let CLOSED = [];


// grids
function Grids(rows, cols) {
    let table = document.querySelector('#grids table');

    for (let i = 0; i < rows; i++) {
        // for each row
        let row = document.createElement('tr');

        for (let j = 0; j < cols; j++) {
            // for each column
            let grid = document.createElement('td');
            if (j < 10 && i < 10) {
                grid.setAttribute('id', `0${i}0${j}`);
            }
            else if (i < 10) {
                grid.setAttribute('id', `0${i}${j}`);
            }
            else if (j < 10) {
                grid.setAttribute('id', `${i}0${j}`);
            }
            
            row.append(grid);
        }
        
        table.append(row);
    }

    // add style to td
    document.querySelectorAll('td').forEach((td) => {
        td.setAttribute('class', 'grid');
    })
}

Grids(20, 20);

// A* Pathfinding Algorithm
// f(n) = g(n) + h(n)
// (h) represents vertices far from the goal
// (g) represents vertices far from the starting point.
function Astar(start, goal) {
    console.log('A* Pathfinding Algorithm');

    // convert start and goal to nodes
    [start_x, start_y] = convert(start);
    [goal_x, goal_y] = convert(goal);

    start = new Node(start_x, start_y);
    start.color('blue');
    
    goal = new Node(goal_x, goal_y);
    goal.color('orange');
    
    // put start into OPEN
    OPEN.push(start);

    while (goal.x != start.x || goal.y != start.y) {
        let current = LowestFValue(OPEN);
        // caculate for neighbours
        
        break;
    }
}
Astar('1302', '0317');

// for each node
function Node(x, y) {
    this.x = x;
    this.y = y;

    this.color = (color) => {
        let x_num = x;
        let y_num = y;
        let str_id = String(("0" + x_num).slice(-2)) + String(("0" + y_num).slice(-2));

        document.getElementById(`${str_id}`).style.backgroundColor = color;
    }
    

    this.f = 0;
    this.g = 0;
    this.h = 0;

    // pointer to parent
    this.parent = 'n0';
}

// calculate Heuristic (h) (how far from goal)
// Diagonal distance allowed
function heuristic(node, goal) {
    dx = abs(node.x - goal.x);
    dy = abs(node.y - goal.y);
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
}

// calculate (g) (how far from starting point)
function gPath(node, start) {
    dx = abs(node.x - start.x);
    dy = abs(node.y - start.y);
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
}

// finding lowest F value in OPEN list
// returning the node and removing it from OPEN list
function LowestFValue(open_list) {
    let smallest = [0, open_list[0]];
    list_len = open_list.length;
    for(let i = 0; i < list_len; i++) {
        if (open_list[i] < smallest[1]) {
            smallest[0] = i;
            smallest[1] = open_list[i];
        }
    }
    open_list.splice(smallest[0], 1);
    return smallest[1];
}


// convert id from td to [x, y]
function convert(id) {
    int_id = parseInt(id);
    let y = int_id % 100;
    let x = Math.floor(int_id / 100) % 100;
    return [x, y];
  }