#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>

using namespace std;


class Map {
public:
    const static int mapWidth = 6;
    const static int mapHeight = 5;
    vector<vector<int>> grid = {
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 1, 1, 0 }
    };
};

class Planner: Map {
public:
    int start[2] = { 0 , 0 };
    int goal[2] = { mapHeight - 1, mapWidth - 1 };
    int cost = 1;

    string movements_arrows[4] = { "^", "<", "v", ">" };
    vector<vector<int>> movements {
        { -1, 0 },
        { 0, -1 },
        { 1, 0 },
        { 0, 1 }
    };
};

template <typename T>
void print2DVector(T to_print) {
    for (int i = 0; i < to_print.size(); ++i){
        for (int j = 0; j < to_print[i].size(); ++j) {
            cout << to_print[i][j] << " ";
        }
        cout << endl;
    }
}

void search(Map map, Planner planner) {
    vector<vector<int>> closed(map.mapWidth, vector<int>(map.mapWidth));
    closed[planner.start[0]][planner.start[1]] = 1;

    vector<vector<int>> expand(map.mapHeight, vector<int>(map.mapWidth, -1));

    vector<vector<int>> action(map.mapHeight, vector<int>(map.mapWidth, -1));

    int x = planner.start[0];
    int y = planner.start[1];
    int g = 0;
    int count = 0;

    vector<vector<int>> open;
    open.push_back({ g, x, y });

    int x2;
    int y2;

    while (true) {
        if (open.size() == 0) {
            cout << "failed to reach a goal" << endl;
            break;
        }
        sort(open.begin(), open.end());
        reverse(open.begin(), open.end());
        vector<int> next;

        next = open.back();
        open.pop_back();

        g = next[0];
        x = next[1];
        y = next[2];
        expand[x][y] = count;
        count += 1;

        if (x == planner.goal[0] && y == planner.goal[1]) {
            cout << "[" << g << ", " << x << ", " << y << "]" << endl;
            break;
        }

        for (int i = 0; i < planner.movements.size(); i++) {
            x2 = x + planner.movements[i][0];
            y2 = y + planner.movements[i][1];
            if (x2 >= 0 && x2 < map.grid.size() && y2 >= 0 && y2 < map.grid[0].size()) {
                if (closed[x2][y2] == 0 && map.grid[x2][y2] == 0) {
                    int g2 = g + planner.cost;
                    open.push_back({ g2, x2, y2 });
                    closed[x2][y2] = 1;
                    action[x2][y2] = i;
                }
            }
        }
    }
    //print2DVector(expand);

    // path with robot orientation
    vector<vector<string>> policy(map.mapHeight, vector<string>(map.mapWidth, "-"));

    // does the action fill in in reverse
    x = planner.goal[0];
    y = planner.goal[1];
    policy[x][y] = "*";

    while (x != planner.start[0] || y != planner.start[1]) {
        x2 = x - planner.movements[action[x][y]][0];
        y2 = y - planner.movements[action[x][y]][1];
        policy[x2][y2] = planner.movements_arrows[action[x][y]];
        x = x2;
        y = y2;
    }

    // 
    print2DVector(policy);
}


int main() {
    Map map;
    Planner planner;

    // Print classes variables
    cout << "Map:" << endl;
    print2DVector(map.grid);
    cout << "Start: " << planner.start[0] << " , " << planner.start[1] << endl;
    cout << "Goal: " << planner.goal[0] << " , " << planner.goal[1] << endl;
    cout << "Cost: " << planner.cost << endl;
    cout << "Robot Movements: " << planner.movements_arrows[0] << " , " << planner.movements_arrows[1] << " , " << planner.movements_arrows[2] << " , " << planner.movements_arrows[3] << endl;
    cout << "Delta:" << endl;
    print2DVector(planner.movements);

    search(map, planner);


    return 0;

}