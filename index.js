// Chebyshev distance
const D = 1;
const D2 = 1;

// variables for A* Pathfinding Algorithm
let OPEN = [];
let CLOSED = [];

// animation speed
let speed = 0.1;

// start and goal points
let start_point_count = 1;
let goal_point_count = 1;
let start_click_count = 0;
let goal_click_count = 0;

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

const GRID_WIDTH = 20;
const GRID_HEIGHT = 20;
Grids(GRID_WIDTH, GRID_HEIGHT);

// random Grid
function RandomGrid() {
    return String(("0" + Math.floor(Math.random() * 20)).slice(-2)) + String(("0" + Math.floor(Math.random() * 20)).slice(-2));
}

// clear board
function ClearBoard() {
    document.querySelectorAll('td').forEach((td) => {
        td.style.backgroundColor = 'white';
    })

    // clear OPEN and CLOSED lists
    OPEN = [];
    CLOSED = [];

    // clear start and end point counts
    start_point_count = 0;
    end_point_count = 0;
}

// random board
function RandomBoard() {
    // clear board
    ClearBoard();

    // set start and goal points count
    start_point_count = 1;
    end_point_count = 1;
    
    // 250 obstacles
    for (let i = 0; i < 251; i++) {
        let obstacle = new Node(Math.floor(Math.random() * 20), Math.floor(Math.random() * 20));
        obstacle.color('black');
        CLOSED.push(obstacle);
    }

    // start point and goal point
    start = RandomGrid();
    goal = RandomGrid();

    while (goal == start) {
        goal = RandomGrid();
    }

    // convert start and goal to nodes
    [start_x, start_y] = convert(start);
    [goal_x, goal_y] = convert(goal);

    start = new Node(start_x, start_y);
    start.color('blue');

    goal = new Node(goal_x, goal_y);
    goal.color('orange');

    // put start into OPEN
    OPEN.push(start);
}


// A* Pathfinding Algorithm
// f(n) = g(n) + h(n)
// (h) represents vertices far from the goal
// (g) represents vertices far from the starting point.
async function Astar() {
    console.log('A* Pathfinding Algorithm');

    while (goal.x != start.x || goal.y != start.y) {
        // if no path available
        if (OPEN.length == 0) {
            alert('no path available!');
            return;
        }
        let current_node = LowestFValue(OPEN);
        CLOSED.push(current_node);

        // change colors current node that is not start/goal
        if (current_node.x != start.x || current_node.y != start.y) {
            current_node.color('red');
            await wait();
        }

        // caculate for neighbours
        for (let i = -1; i < 2; i++) {
            neighbour_loop:
            for (let j = -1; j < 2; j++) {
                // x and y co-ordinates of neighbour
                let neighbour_x = current_node.x + i;
                let neighbour_y = current_node.y + j;

                // if neighbour not on the grid
                if (neighbour_x < 0 || neighbour_x >= GRID_WIDTH || neighbour_y < 0 || neighbour_y >= GRID_HEIGHT) {
                    continue neighbour_loop;
                }

                // if neighbour node is the goal
                if (neighbour_x == goal.x && neighbour_y == goal.y) {
                    return Path(current_node);
                }
                
                // make sure the neighbour exists and it is not the current node
                if ((neighbour_x < 0 || neighbour_y < 0) || (neighbour_x == current_node.x && neighbour_y == current_node.y)) {
                    continue neighbour_loop;
                }

                // create neighbour node
                let neighbour_node = new Node(neighbour_x, neighbour_y);

                // add neighbour parent
                neighbour_node.parent = [current_node.x, current_node.y];

                // diagonal steps
                let diagonal = false;
                let temp = [i, j];
                
                if ((temp[0] == -1 && temp[1] == -1) || (temp[0] == -1 && temp[1] == 1) || (temp[0] == 1 && temp[1] == -1) || (temp[0] == 1 && temp[1] == 1)) {
                    
                    diagonal = true;
                }

                neighbour_node.h = heuristic(neighbour_node, goal, diagonal);
                neighbour_node.g = gCost(neighbour_node, start, diagonal);
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
                            await wait();
                            break;
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
                await wait();
            }
        }
    }
}

// 250 obstacles
for (let i = 0; i < 251; i++) {
    let obstacle = new Node(Math.floor(Math.random() * 20), Math.floor(Math.random() * 20));
    obstacle.color('black');
    CLOSED.push(obstacle);
}

RandomBoard();

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
    this.parent = [-1, -1];
}

// calculate Heuristic (h) (how far from goal)
// Diagonal distance allowed
function heuristic(node, goal, diagonal) {
    dx = Math.abs(node.x - goal.x);
    dy = Math.abs(node.y - goal.y);
    if (diagonal) {
        return D * (dx + dy) * 1.03;
        // D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy)
    }
    else {
        return D * (dx + dy);
    }
}

// calculate (g) (exact cost of the path from the starting point to any vertex n)
function gCost(node, start, diagonal) {
    let gcost = 0;

    dx = Math.abs(node.x - start.x);
    dy = Math.abs(node.y - start.y);
    if (diagonal) {
        gcost += (D * (dx + dy) * 1.03);
    }
    else {
        gcost += (D * (dx + dy));
    }

    
    let [parent_x, parent_y] = node.parent;
    
    // loop through CLOSED list to find parent if the node has a parent
    if (parent_x != -1 && parent_y != -1) {
        let closed_len = CLOSED.length;
        
        for (let i = 0; i < closed_len; i++) {
            if (parent_x == CLOSED[i].x && parent_y == CLOSED[i].y) {
                gcost += CLOSED[i].g;
            }
        }
    }
    
    return 0.65 * gcost;
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

// show path animation
async function Path(current_node) {
    // base case
    if (current_node.x == start.x && current_node.y == start.y) {
        return;
    }

    current_node.color('green');
    await wait();
    let [parent_x, parent_y] = current_node.parent;
    
    // loop through CLOSED list to find parents recursively
    let closed_len = CLOSED.length;
    for (let i = 0; i < closed_len; i++) {
        if (parent_x == CLOSED[i].x && parent_y == CLOSED[i].y) {
            Path(CLOSED[i]);
        }
    }
}

// slow animation
function wait() {
    return new Promise(resolve => {
        setTimeout(() => {
            resolve('resolved');
        }, speed * 20);
    });
}

function Buttons() {
    function SpeedButton() {
        let button = document.querySelector('#speed');
        button.addEventListener('change', () => {
            speed = button.value;
        })
    }

    function ClearBoardButton() {
        document.querySelector('#clear-board').onclick = () => {
            ClearBoard();
        }
    }

    function RandomBoardButton() {
        document.querySelector('#random-board').onclick = () => {
            RandomBoard();
        }
    }

    function StartPointButton() {
        let start_point = document.querySelector('#start-point');
        start_point.addEventListener('click', () => {
            
            // first click
            if (start_click_count == 0) {
                // if end point button is enabled, disable it
                if (goal_click_count != 0) {
                    document.querySelector('#goal-point').click();
                }

                start_click_count++;
                start_point.style.backgroundColor = 'lightblue';
                document.querySelectorAll('td').forEach((td) => {
                    td.addEventListener('click', start_point_function);
                });
            }

            // second click
            else if (start_click_count == 1) {
                start_click_count--;
                start_point.style.backgroundColor = 'lightgray';
                document.querySelectorAll('td').forEach((td) => {
                    td.removeEventListener('click', start_point_function);
                });
            }
        });

        function start_point_function() {
            if (start_point_count > 0) {
                document.querySelectorAll('td').forEach((cell) => {
                    // if starting point already chosen
                    if (cell.style.backgroundColor == 'blue') {
                        cell.style.backgroundColor = 'white';
                        start_point_count = 0;
                    }
                })
            }
            start_point_count++;

            start = this.id;
            [start_x, start_y] = convert(start);
            start = new Node(start_x, start_y);
            start.color('blue');
            
            // remove old start point
            OPEN = [];

            // put start into OPEN
            OPEN.push(start);
        }
    }

    function GoalPointButton() {
        let goal_point = document.querySelector('#goal-point');
        goal_point.addEventListener('click', () => {
            
            // first click
            if (goal_click_count == 0) {
                // if start point button is enabled, disable it
                if (start_click_count != 0) {
                    document.querySelector('#start-point').click();
                }

                goal_click_count++;
                goal_point.style.backgroundColor = 'orange';
                document.querySelectorAll('td').forEach((td) => {
                    td.addEventListener('click', goal_point_function);
                });
            }

            // second click
            else if (goal_click_count == 1) {
                goal_click_count--;
                goal_point.style.backgroundColor = 'lightgray';
                document.querySelectorAll('td').forEach((td) => {
                    td.removeEventListener('click', goal_point_function);
                });
            }
        });

        function goal_point_function() {
            if (goal_point_count > 0) {
                document.querySelectorAll('td').forEach((cell) => {
                    // if starting point already chosen
                    if (cell.style.backgroundColor == 'orange') {
                        cell.style.backgroundColor = 'white';
                        goal_point_count = 0;
                    }
                })
            }
            goal_point_count++;

            goal = this.id;
            [goal_x, goal_y] = convert(goal);
            goal = new Node(goal_x, goal_y);
            goal.color('orange');
        }
    }


    function AstarButton() {
        document.querySelector('#astar').onclick = () => {
            Astar();
        }
    }

    // call all the functions
    SpeedButton();
    ClearBoardButton()
    RandomBoardButton();
    StartPointButton();
    GoalPointButton();
    AstarButton();
}
Buttons();