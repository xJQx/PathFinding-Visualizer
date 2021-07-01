// variables for A* Pathfinding Algorithm
let OPEN = [];
let CLOSED = [];

// animation speed
let speed = 0.1;

// start point, goal point and obstacle button functions
let start_point_count = 1;
let goal_point_count = 1;
let start_click_count = 0;
let goal_click_count = 0;
let obstacle_click_count = 0;

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
    // this may make several nodes of the same grid
    // note to remove all copies while removing obstacle
    for (let i = 0; i < 250; i++) {
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

    // if goal is in CLOSED, remove it
    let closed_len = CLOSED.length;
    for (let i = 0; i < closed_len; i++) {
        if (CLOSED[i].x == goal.x && CLOSED[i].y == goal.y) {
            CLOSED.splice(i, 1);
            closed_len--;
            i--;
        }
    }
}

// reset board back to original
function ResetBoard() {
    document.querySelectorAll('td').forEach((td) => {
        td.style.backgroundColor = 'white';
    })

    // color obstacles
    let closed_len = CLOSED.length;
    for (let i = 0; i < closed_len; i++) {
        CLOSED[i].color('black');
    }

    // set start and goal points count
    start_point_count = 1;
    end_point_count = 1;

    // color start and goal
    if (!start || !goal) {
        Alert2();
        return false;
    }

    start.color('blue');
    goal.color('orange');

    return true;
}

// sample board
function SampleBoard() {
    // clear board
    ClearBoard();

    // set start and goal points count
    start_point_count = 1;
    end_point_count = 1;
    
    // sample obstacles
    for (let i = 1; i < 15; i++) {
        let obstacle = new Node(11, i);
        obstacle.color('black');
        CLOSED.push(obstacle);
    }
    for (let i = 1; i < 12; i++) {
        let obstacle = new Node(i, 14);
        obstacle.color('black');
        CLOSED.push(obstacle);
    }

    // start point and goal point
    start = '0307';
    goal = '1517';

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

RandomBoard();

// A* Pathfinding Algorithm
// f(n) = g(n) + h(n)
// (h) represents vertices far from the goal
// (g) represents the exact cost of the path from the starting point to any vertex n
async function Astar(open, closed) {
    console.log('A* Pathfinding Algorithm');
    DisableButtons();

    // check if there is a starting point and a goal point
    if (!goal || !start) {
        Alert2();
        return EnableButtons();
    }

    while (goal.x != start.x || goal.y != start.y) {
        // if no path available
        if (open.length == 0) {
            Alert()
            return EnableButtons();
        }
        let current_node = LowestFValue(open);
        closed.push(current_node);

        // change colors current node that is not start
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
                    Path(current_node);
                    return EnableButtons();
                }
                
                // make sure the neighbour is not the current node
                if (neighbour_x == current_node.x && neighbour_y == current_node.y) {
                    continue neighbour_loop;
                }

                // create neighbour node
                let neighbour_node = new Node(neighbour_x, neighbour_y);

                // add neighbour parent
                neighbour_node.parent = current_node;

                // diagonal steps
                let diagonal = false;
                let temp = [i, j];
                
                if ((temp[0] == -1 && temp[1] == -1) || (temp[0] == -1 && temp[1] == 1) || (temp[0] == 1 && temp[1] == -1) || (temp[0] == 1 && temp[1] == 1)) {
                    diagonal = true;
                }

                neighbour_node.h = heuristic(neighbour_node, goal, diagonal);
                neighbour_node.g = gCost(neighbour_node, neighbour_node.parent, diagonal);
                neighbour_node.f = neighbour_node.g + neighbour_node.h;

                // make sure neighbour not in CLOSED list
                let closed_len = closed.length;
                for (let k = 0; k < closed_len; k++) {
                    if (neighbour_node.x == closed[k].x && neighbour_node.y == closed[k].y) {
                        continue neighbour_loop;
                    }
                }

                // if neighbour is in OPEN
                // check if new f and g are lower
                let open_len = open.length;
                for (let w = 0; w < open_len; w++) {
                    if (neighbour_node.x == open[w].x && neighbour_node.y == open[w].y) {
                        if (neighbour_node.f <= open[w].f) {
                            if (neighbour_node.g < open[w].g) {
                                // remove old lower f value neighbour from OPEN
                                open[w].color('white');
                                open.splice(w, 1);
                                await wait();
                                break;
                            }
                            else {
                                continue neighbour_loop;
                            }
                        }
                        else {
                            // if new f is higher or same, ignore this neighbour
                            continue neighbour_loop;
                        }
                    }
                }

                neighbour_node.color('lightblue');
                open.push(neighbour_node);
                await wait();
            }
        }
    }
}

// Greedy Best First Search
// consider h(n) only
// how far is the grid from goal
async function Greedy(open, closed) {
    console.log('Greedy Best First Search');
    DisableButtons();

    // check if there is a starting point and a goal point
    if (!goal || !start) {
        Alert2();
        return EnableButtons();
    }

    while (goal.x != start.x || goal.y != start.y) {
        // if no path available
        if (open.length == 0) {
            Alert();
            return EnableButtons();
        }
        
        let current_node = LowestHValue(open);
        closed.push(current_node);

        // change colors current node that is not start
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
                    Path(current_node);
                    return EnableButtons();
                }
                
                // make sure the neighbour is not the current node
                if (neighbour_x == current_node.x && neighbour_y == current_node.y) {
                    continue neighbour_loop;
                }

                // create neighbour node
                let neighbour_node = new Node(neighbour_x, neighbour_y);

                // add neighbour parent
                neighbour_node.parent = current_node;

                // diagonal steps
                let diagonal = false;
                let temp = [i, j];
                
                if ((temp[0] == -1 && temp[1] == -1) || (temp[0] == -1 && temp[1] == 1) || (temp[0] == 1 && temp[1] == -1) || (temp[0] == 1 && temp[1] == 1)) {
                    diagonal = true;
                }

                neighbour_node.h = heuristic(neighbour_node, goal, diagonal);

                // make sure neighbour not in CLOSED list
                let closed_len = closed.length;
                for (let k = 0; k < closed_len; k++) {
                    if (neighbour_node.x == closed[k].x && neighbour_node.y == closed[k].y) {
                        continue neighbour_loop;
                    }
                }

                // make sure neighbour is not in OPEN
                let open_len = open.length;
                for (let w = 0; w < open_len; w++) {
                    if (neighbour_node.x == open[w].x && neighbour_node.y == open[w].y) {
                        continue neighbour_loop;
                    }
                }

                neighbour_node.color('lightblue');
                open.push(neighbour_node);
                await wait();
            }
        }
    }
}

// breadth first search
// search by expanding neighbouring nodes (uninformed search)
async function Breadth(open, closed) {
    console.log('Breadth First Search');
    DisableButtons();

    // check if there is a starting point and a goal point
    if (!goal || !start) {
        Alert2();
        return EnableButtons();
    }

    while (goal.x != start.x || goal.y != start.y) {
        // if no path available
        if (open.length == 0) {
            Alert();
            return EnableButtons();
        }
        
        let current_node = open.shift();
        closed.push(current_node);

        // change colors current node that is not start
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
                    Path(current_node);
                    return EnableButtons();
                }
                
                // make sure the neighbour is not the current node
                if (neighbour_x == current_node.x && neighbour_y == current_node.y) {
                    continue neighbour_loop;
                }

                // create neighbour node
                let neighbour_node = new Node(neighbour_x, neighbour_y);

                // add neighbour parent
                neighbour_node.parent = current_node;

                // make sure neighbour not in CLOSED list
                let closed_len = closed.length;
                for (let k = 0; k < closed_len; k++) {
                    if (neighbour_node.x == closed[k].x && neighbour_node.y == closed[k].y) {
                        continue neighbour_loop;
                    }
                }

                // make sure neighbour is not in OPEN
                let open_len = open.length;
                for (let w = 0; w < open_len; w++) {
                    if (neighbour_node.x == open[w].x && neighbour_node.y == open[w].y) {
                        continue neighbour_loop;
                    }
                }

                neighbour_node.color('lightblue');
                open.push(neighbour_node);
                await wait();
            }
        }
    }
}

// Dijkstra's algorithm
// find shortest path to each node from source (starting node)
// g(n) cost
async function Dijkstra(open, closed) {
    console.log('Dijkstra Algorithm');
    DisableButtons();

    // check if there is a starting point and a goal point
    if (!goal || !start) {
        Alert2();
        return EnableButtons();
    }

    while (goal.x != start.x || goal.y != start.y) {
        // if no path available
        if (open.length == 0) {
            Alert();
            return EnableButtons();
        }
        
        let current_node = LowestGValue(open);
        closed.push(current_node);

        // check if current node is the goal (having min g cost in open list)
        if (current_node.x == goal.x && current_node.y == goal.y) {
            Path(current_node);
            current_node.color('orange');
            return EnableButtons();
        }

        // change colors current node that is not start
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
                
                // make sure the neighbour is not the current node
                if (neighbour_x == current_node.x && neighbour_y == current_node.y) {
                    continue neighbour_loop;
                }

                // create neighbour node
                let neighbour_node = new Node(neighbour_x, neighbour_y);

                // add neighbour parent
                neighbour_node.parent = current_node;

                // diagonal steps
                let diagonal = false;
                let temp = [i, j];
                
                if ((temp[0] == -1 && temp[1] == -1) || (temp[0] == -1 && temp[1] == 1) || (temp[0] == 1 && temp[1] == -1) || (temp[0] == 1 && temp[1] == 1)) {
                    diagonal = true;
                }

                neighbour_node.g = gCost(neighbour_node, neighbour_node.parent, diagonal);

                // make sure neighbour not in CLOSED list
                let closed_len = closed.length;
                for (let k = 0; k < closed_len; k++) {
                    if (neighbour_node.x == closed[k].x && neighbour_node.y == closed[k].y) {
                        continue neighbour_loop;
                    }
                }

                // if neighbour in OPEN, check if new g is lower (more efficient path)
                let open_len = open.length;
                for (let w = 0; w < open_len; w++) {
                    if (neighbour_node.x == open[w].x && neighbour_node.y == open[w].y) {
                        if (neighbour_node.g < open[w].g) {
                            open.splice(w, 1);
                            neighbour_node.color('white');
                            await wait();
                            break;
                        }
                        else {
                            continue neighbour_loop;
                        }
                    }
                }

                // change color if not goal
                if (neighbour_node.x != goal.x || neighbour_node.y != goal.y) {
                    neighbour_node.color('lightblue');
                }

                open.push(neighbour_node);
                await wait();
            }
        }
    }
}

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
        return (Math.sqrt(dx ** 2 + dy ** 2));
        // D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy)
    }
    else {
        return (Math.sqrt(dx ** 2 + dy ** 2));
    }
}

// calculate (g) (exact cost of the path from the starting point to any vertex n)
function gCost(node, parent, diagonal) {
    let gcost = parent.g;

    dx = Math.abs(node.x - parent.x);
    dy = Math.abs(node.y - parent.y);
    if (diagonal) {
        gcost += (Math.sqrt(dx ** 2 + dy ** 2));
    }
    else {
        gcost += (Math.sqrt(dx ** 2 + dy ** 2));
    }
    
    return gcost;
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
        // compare g when f are the same
        else if (open_list[i].f == smallest_value[0]) {
            if (open_list[i].g < smallest_value[2]) {
                smallest[0] = i;
                smallest[1] = open_list[i];
                smallest_value = [open_list[i].f, open_list[i].h, open_list[i].g];
            }
            // compare h when f and g are the same
            else if (open_list[i].g == smallest_value[2]) {
                if (open_list[i].h < smallest_value[1]) {
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

// find lowest h value in OPEN list
// return node and remove from OPEN list
function LowestHValue(open_list) {
    // smallest = [index, node]
    // lowest_h = h value
    let smallest = [0, open_list[0]];
    let lowest_h = open_list[0].h;

    let open_len = open_list.length;
    for (let i = 0; i < open_len; i++) {
        if (open_list[i].h < lowest_h) {
            lowest_h = open_list[i].h;
            smallest[0] = i;
            smallest[1] = open_list[i];
        }
    }
    open_list.splice(smallest[0], 1);
    return smallest[1];
}

// find lowest g value in OPEN list
// return node and remove from OPEN list
function LowestGValue(open_list) {
    // smallest = [index, node]
    // lowest_g = g value
    let smallest = [0, open_list[0]];
    let lowest_g = open_list[0].g;

    let open_len = open_list.length;
    for (let i = 0; i < open_len; i++) {
        if (open_list[i].g < lowest_g) {
            lowest_g = open_list[i].g;
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

// show path animation
async function Path(current_node) {
    // base case
    if (current_node.x == start.x && current_node.y == start.y) {
        return;
    }

    current_node.color('green');
    await wait();
    Path(current_node.parent);
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

    function ResetBoardButton() {
        document.querySelector('#reset-board').onclick = () => {
            ResetBoard();
        }
    }
    
    function SampleBoardButton() {
        document.querySelector('#sample-board').onclick = () => {
            SampleBoard();
        }
    }

    function StartPointButton() {
        let start_point = document.querySelector('#start-point');

        start_point.addEventListener('mouseover', () => {
            start_point.style.backgroundColor = 'blue';
        });
        start_point.addEventListener('mouseout', () => {
            if (start_click_count == 0) {
                start_point.style.backgroundColor = 'white';
            }
        });

        start_point.addEventListener('click', () => {
            
            // first click
            if (start_click_count == 0) {
                // if end point button is enabled, disable it
                if (goal_click_count != 0) {
                    document.querySelector('#goal-point').click();
                }
                // if obstacle button is enabled, disable it
                if (obstacle_click_count != 0) {
                    document.querySelector('#modify-obstacle').click();
                }

                start_click_count++;
                start_point.style.backgroundColor = 'blue';
                start_point.style.color = 'white';
                document.querySelectorAll('td').forEach((td) => {
                    td.addEventListener('click', start_point_function);
                    td.addEventListener('mouseover', hoverColor);
                });
            }

            // second click
            else if (start_click_count == 1) {
                start_click_count--;
                start_point.setAttribute('style', '');
                document.querySelectorAll('td').forEach((td) => {
                    td.removeEventListener('click', start_point_function);
                    td.removeEventListener('mouseover', hoverColor);
                });
            }
        });

        function hoverColor() {
            grid_color = this.style.backgroundColor;
            this.style.backgroundColor = 'blue';
            this.addEventListener('mouseout', hoverOff);

            function hoverOff() {
                this.style.backgroundColor = grid_color;
                this.removeEventListener('mouseout', hoverOff);
            };
        }

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

            // if the block replaced is in CLOSED, remove it from CLOSED
            let closed_len = CLOSED.length;
            if (grid_color == 'black') {
                for (let i = 0; i < closed_len; i++) {
                    if (CLOSED[i].x == start_x && CLOSED[i].y == start_y) {
                        CLOSED.splice(i, 1);
                        closed_len--;
                        i--;
                    }
                }
            }
            // if goal is replaced
            else if (grid_color == 'orange') {
                goal = null;
            }

            grid_color = 'blue';
        }
    }

    function GoalPointButton() {
        let goal_point = document.querySelector('#goal-point');

        goal_point.addEventListener('mouseover', () => {
            goal_point.style.backgroundColor = 'orange';
        });
        goal_point.addEventListener('mouseout', () => {
            if (goal_click_count == 0) {
                goal_point.style.backgroundColor = 'white';
            }
        });

        goal_point.addEventListener('click', () => {
            
            // first click
            if (goal_click_count == 0) {
                // if start point button is enabled, disable it
                if (start_click_count != 0) {
                    document.querySelector('#start-point').click();
                }
                // if obstacle button is enabled, disable it
                if (obstacle_click_count != 0) {
                    document.querySelector('#modify-obstacle').click();
                }

                goal_click_count++;
                goal_point.style.backgroundColor = 'orange';
                goal_point.style.color = 'white';
                document.querySelectorAll('td').forEach((td) => {
                    td.addEventListener('click', goal_point_function);
                    td.addEventListener('mouseover', hoverColor);
                });
            }

            // second click
            else if (goal_click_count == 1) {
                goal_click_count--;
                goal_point.setAttribute('style', '');
                document.querySelectorAll('td').forEach((td) => {
                    td.removeEventListener('click', goal_point_function);
                    td.removeEventListener('mouseover', hoverColor);
                });
            }
        });

        function hoverColor() {
            grid_color = this.style.backgroundColor;
            this.style.backgroundColor = 'orange';
            this.addEventListener('mouseout', hoverOff);

            function hoverOff() {
                this.style.backgroundColor = grid_color;
                this.removeEventListener('mouseout', hoverOff);
            };
        }

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

            // if the block replaced is in CLOSED, remove it from CLOSED
            let closed_len = CLOSED.length;
            if (grid_color == 'black') {
                for (let i = 0; i < closed_len; i++) {
                    if (CLOSED[i].x == goal_x && CLOSED[i].y == goal_y) {
                        CLOSED.splice(i, 1);
                        closed_len--;
                        i--;
                    }
                }
            }
            // if start is replaced
            else if (grid_color == 'blue') {
                start = null;
            }

            grid_color = 'orange';
        }
    }

    function NewObstacleButton() {
        let obstacle_button = document.querySelector('#modify-obstacle');

        obstacle_button.onclick = () => {
            // first click
            if (obstacle_click_count == 0) {
                // if start point button is enabled, disable it
                if (start_click_count != 0) {
                    document.querySelector('#start-point').click();
                }
                // if end point button is enabled, disable it
                if (goal_click_count != 0) {
                    document.querySelector('#goal-point').click();
                }

                obstacle_click_count++;
                obstacle_button.style.backgroundColor = 'black';
                obstacle_button.style.color = 'white';
                document.querySelectorAll('td').forEach((td) => {
                    td.addEventListener('click', obstacle_point_function);
                    td.addEventListener('mouseover', hoverColor);
                });
            }

            // second click
            else if (obstacle_click_count == 1) {
                obstacle_click_count--;
                obstacle_button.setAttribute('style', '');
                document.querySelectorAll('td').forEach((td) => {
                    td.addEventListener('click', obstacle_point_function);
                    td.removeEventListener('mouseover', hoverColor);
                });
            }
        }

        function hoverColor() {
            grid_color = this.style.backgroundColor;
            if (grid_color == 'white') {
                this.style.backgroundColor = 'black';
            }
            else if (grid_color == 'black') {
                this.style.backgroundColor = 'white';
            }
            this.addEventListener('mouseout', hoverOff);

            function hoverOff() {
                this.style.backgroundColor = grid_color;
                this.removeEventListener('mouseout', hoverOff);
            };
        }

        function obstacle_point_function() {
            let [obstacle_x, obstacle_y] = convert(this.id);
            
            // check if obstacle exist, remove obstacle if it does
            if (grid_color == 'black') {
                let closed_len = CLOSED.length;
                
                for (let i = 0; i < closed_len; i++) {
                    if (obstacle_x == CLOSED[i].x && obstacle_y == CLOSED[i].y) {
                        grid_color = 'white';
                        CLOSED[i].color('white');
                        CLOSED.splice(i, 1);
                        closed_len--;
                        i--;
                    }
                }
            }
            else {
                // check if grid replaced is start or goal
                if (this.style.backgroundColor == 'blue') {
                    start = null;
                }
                else if (this.style.backgroundColor == 'orange') {
                    goal = null;
                }
                let obstacle = new Node(obstacle_x, obstacle_y);
                obstacle.color('black');
                grid_color = 'black';
                CLOSED.push(obstacle);
            }
        }
    }

    function AstarButton() {
        document.querySelector('#astar').onclick = () => {
            let open = [];
            let open_len = OPEN.length;
            for (let i = 0; i < open_len; i++) {
                open.push(OPEN[i]);
            }
            let closed = [];
            let closed_len = CLOSED.length;
            for (let i = 0; i < closed_len; i++) {
                closed.push(CLOSED[i]);
            }

            if(ResetBoard()) {
                Astar(open, closed);
            }
        }
    }

    function GreedyButton() {
        document.querySelector('#greedy').onclick = () => {
            let open = [];
            let open_len = OPEN.length;
            for (let i = 0; i < open_len; i++) {
                open.push(OPEN[i]);
            }
            let closed = [];
            let closed_len = CLOSED.length;
            for (let i = 0; i < closed_len; i++) {
                closed.push(CLOSED[i]);
            }

            if(ResetBoard()) {
                Greedy(open, closed);
            }
        }
    }

    function BreadthButton() {
        document.querySelector('#breadth').onclick = () => {
            let open = [];
            let open_len = OPEN.length;
            for (let i = 0; i < open_len; i++) {
                open.push(OPEN[i]);
            }
            let closed = [];
            let closed_len = CLOSED.length;
            for (let i = 0; i < closed_len; i++) {
                closed.push(CLOSED[i]);
            }

            if(ResetBoard()) {
                Breadth(open, closed);
            }

        }
    }

    function DijkstraButton() {
        document.querySelector('#dijkstra').onclick = () => {
            let open = [];
            let open_len = OPEN.length;
            for (let i = 0; i < open_len; i++) {
                open.push(OPEN[i]);
            }
            let closed = [];
            let closed_len = CLOSED.length;
            for (let i = 0; i < closed_len; i++) {
                closed.push(CLOSED[i]);
            }

            if(ResetBoard()) {
                Dijkstra(open, closed);
            }
        }
    }

    // call all the functions
    SpeedButton();

    ClearBoardButton()
    RandomBoardButton();
    ResetBoardButton();
    SampleBoardButton();

    StartPointButton();
    GoalPointButton();
    NewObstacleButton();

    AstarButton();
    GreedyButton();
    BreadthButton();
    DijkstraButton();
}
Buttons();

function DisableButtons() {
    document.querySelectorAll('button').forEach((button) => {
        button.disabled = true;
        button.style.opacity = 0.5;
    })
}
function EnableButtons() {
    document.querySelectorAll('button').forEach((button) => {
        button.disabled = false;
        button.style.opacity = 1;
    })
}

function Alert() {
    let alert = document.querySelector('#alert');
    alert.style.display = 'block';
    document.querySelector('#close').onclick = () => {
        alert.style.display = 'none';
    }
}
function Alert2() {
    let alert2 = document.querySelector('#alert2');
    alert2.style.display = 'block';
    document.querySelector('#close2').onclick = () => {
        alert2.style.display = 'none';
    }
}