// Chebyshev distance
const D = 1;
const D2 = 1;

// variables for A* Pathfinding Algorithm
let OPEN = [];
let CLOSED = [];

let loop = 0;
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
            else {
                grid.setAttribute('id', `${i}${j}`);
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
        let current_node = LowestFValue(OPEN);

        // caculate for neighbours
        for (let i = -1; i < 2; i++) {
            neighbour_loop:
            for (let j = -1; j < 2; j++) {
                // x and y co-ordinates of neighbour
                let neighbour_x = current_node.x + i;
                let neighbour_y = current_node.y + j;
                
                // make sure the neighbour exists and it is not the current node
                if ((neighbour_x < 0 || neighbour_y < 0) || (neighbour_x == current_node.x && neighbour_y == current_node.y)) {
                    continue neighbour_loop;
                }

                // calculate neighbour node
                let neighbour_node = new Node(neighbour_x, neighbour_y);

                // diagonal steps
                let diagonal = false;
                let temp = [i, j];
                if (temp == [-1, -1] || temp == [-1, 1] || temp == [1, -1] || temp == [1, 1]) {
                    diagonal = true;
                }

                neighbour_node.h = heuristic(neighbour_node, goal, diagonal);
                neighbour_node.g = gPath(neighbour_node, start, diagonal);
                neighbour_node.f = neighbour_node.g + neighbour_node.h;

                // make sure neighbour not in CLOSED list
                let closed_len = CLOSED.length;
                for (let k = 0; k < closed_len; k++) {
                    if (neighbour_node.x == CLOSED[k].x && neighbour_node.y == CLOSED[k].y) {
                        continue neighbour_loop;
                    }
                }

                // if neighbour is in OPEN
                // check if new f is lower
                let open_len = OPEN.length;
                for (let w = 0; w < open_len; w++) {
                    if (neighbour_node.x == OPEN[w].x && neighbour_node.y == OPEN[w].y) {
                        if (neighbour_node.f < OPEN[w].f) {
                            // remove old lower f value neighbour from OPEN
                            OPEN[w].color('white');
                            OPEN.splice(w, 1);
                        }
                        else {
                            // if new f is higher or same, ignore this neighbour
                            continue neighbour_loop;
                        }
                    }
                }

                // ignore neighbours that are start / goal
                if (neighbour_node == start || neighbour_node == goal) {
                    continue neighbour_loop;
                }

                neighbour_node.color('lightblue');
                OPEN.push(neighbour_node);
            }
        }
        // change colors current node that is not start/goal
        if (current_node.x != start.x && current_node.y != start.y) {
            current_node.color('red');
        }
        CLOSED.push(current_node);

        console.log(OPEN);
        console.log(CLOSED);

        loop++;
        if (loop == 13) {
            break;
        }
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
function heuristic(node, goal, diagonal) {
    dx = Math.abs(node.x - goal.x);
    dy = Math.abs(node.y - goal.y);
    if (diagonal) {
        return D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
    }
    else {
        return D * (dx + dy);
    }
}

// calculate (g) (how far from starting point)
function gPath(node, start, diagonal) {
    dx = Math.abs(node.x - start.x);
    dy = Math.abs(node.y - start.y);
    if (diagonal) {
        return D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
    }
    else {
        return D * (dx + dy);
    }
}

// finding lowest F value in OPEN list, tie break by lower h value and then lowest g value
// returning the node and removing it from OPEN list
function LowestFValue(open_list) {
    // smallest = [index, node]
    // smallest_value = [f, h, g]
    let smallest = [0, open_list[0]];
    let smallest_value = [open_list[0].f, open_list[0].h, open_list[0].g]

    list_len = open_list.length;
    for(let i = 0; i < list_len; i++) {
        // compare f value
        if (open_list[i].f < smallest_value[0]) {
            smallest[0] = i;
            smallest[1] = open_list[i];
            smallest_value = [open_list[i].f, open_list[i].h, open_list[i].g];
        }
        // compare h when f are the same
        else if (open_list[i].f == smallest_value[0]) {
            if (open_list[i].h < smallest_value[1]) {
                smallest[0] = i;
                smallest[1] = open_list[i];
                smallest_value = [open_list[i].f, open_list[i].h, open_list[i].g];
            }
            // compare g when f and h are the same
            else if (open_list[i].h == smallest_value[1]) {
                if (open_list[i].g < smallest_value[2]) {
                    smallest[0] = i;
                    smallest[1] = open_list[i];
                    smallest_value = [open_list[i].f, open_list[i].h, open_list[i].g];
                }
            }
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