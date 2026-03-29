/*
Created By: Radhees Bala BS - Mechanical and Automation Engineering student.
Date: 17 - MARCH - 2026
GitHub: Radhees-Engg 
This is a Maze Exploration Code which uses DFS algorithm to Explore the Maze and to Map.
Webots - C++

Initial Assumption: 
-> North Facing and East on the Left.  
-> If You wanna keep west in left use the second dx and dy values.
-> If you wanna change the Initial position, simply change the DIRECTION position to SOUTH or whatever direction you want But the Robot
Should Face that Direction.
-> Cell Size is 0.1 m x 0.1 m.
-> Total Maze Size = 5 x 5.
-> Total Maze Size with Wall = 7 x 7 because 0th and the 6th Position which will get printed is wall.
-> You can Also change the size of the maze by changing the value of the global variable const int SIZE.
-> e-puck maximum speed is 6.28, Since I used e puck for this project I set the max speed to 6.0, you can vary depends on the
Robot you're using.
-> If you're changing the sensor type Don't forget to check and change the THRESHOLD value.
*/




#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <iostream>
#include <algorithm>
#include <string>
#include <cmath>
#include <stack>
#include <utility>

#define TIMESTEP 16

using namespace webots;

const int SIZE = 7, VISITED = 1, WALL = 8, DEAD_END = 3, PATH = 0, N = 1, E = 2, S = 4, W = 8;
const float THRESHOLD = 550.0f, TargetDist = 0.10, WheelRadius = 0.0205f, WheelBase = 0.052f, SIDETHRESHOLD = 100.0f, 
PI = acos(-1), I_MAX = 0.05, I_MIN = -0.05;
const double Max_Speed = 6.0, Kp = 4.0f, Ki = 0.18f, Kd = 1.30f;

float ForwardSpeed = 0.0f, IntegralError = 0.0f, DerivativeError = 0.0f, PreTurnError = 0.0f;

// This is East as Left
int dx[4] = { 0, -1, 0, 1 };
int dy[4] = { 1, 0, -1, 0 };

// This is West as Left
//int dx[4] = { 0, 1, 0, -1 };
//int dy[4] = { 1, 0, -1, 0 };

enum ROBOT { DECIDE, FORWARD, LEFTTURN, RIGHTTURN, STOP, BT_TURN, BT_DECIDE, BT_FORWARD, BT_LEFTTURN, BT_RIGHTTURN };
ROBOT state = DECIDE;

enum DIRECTION { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
DIRECTION position = NORTH;   //This is the Initial facing position

bool CheckWin(int x, int y, int endX, int endY)
{
    return(x == endX && y == endY);  // This shuold be used when want the robot to stop moving when reached a specific cell or position
}

struct Odometer
{
    double pre_enc_l = 0.0;
    double pre_enc_r = 0.0;
    double Distance = 0.0;
};

float NormalizeAngle(float Angle)
{
    while (Angle <= -PI) Angle += (PI * 2);
    while (Angle > PI) Angle -= (PI * 2);
    return Angle;
}

float std_clamp(float val, float MAX, float MIN) // I made this Clamp because std::clamp is not available
{
    if (val > MAX) return MAX;
    if (val <= MIN) return MIN;
    return val;
}

bool IsValid(int x, int y, int(*Map)[SIZE])
{
    return(x >= 0 && x < SIZE && y >= 0 && y < SIZE);
}

void Stop(Motor* leftMotor, Motor* rightMotor)
{
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
}

double GetDistanceSensor(DistanceSensor* ps[4])
{
    double Main_ = (ps[0]->getValue() + ps[1]->getValue());
    double Side_ = (ps[2]->getValue() + ps[3]->getValue());
    double DistanceSenorVal = (0.95 * Main_) + (0.05 * Side_);  // This is the weightage for both the front and side front i.e right front and left front sensors
    return DistanceSenorVal;
}

float GetTargetYaw(InertialUnit* imu, Motor* leftMotor, Motor* rightMotor)
{
    Stop(leftMotor, rightMotor);
    float CurrentYaw = NormalizeAngle(imu->getRollPitchYaw()[2]);
    if (state == LEFTTURN || state == BT_LEFTTURN)
    {
        float TargetYaw = NormalizeAngle((CurrentYaw + (PI / 2.0f)));
        return TargetYaw;
    }
    else if (state == RIGHTTURN || state == BT_RIGHTTURN)
    {
        float TargetYaw = NormalizeAngle((CurrentYaw - (PI / 2.0f)));
        return TargetYaw;
    }
    else
    {
        float TargetYaw = CurrentYaw + PI;
        return TargetYaw;
    }
    return CurrentYaw;
}

bool CheckFrontWall(DistanceSensor* ps[4], Motor* leftMotor, Motor* rightMotor)
{
    double IR_val = GetDistanceSensor(ps);
    return(IR_val >= THRESHOLD);
}

int CheckSideWall(DistanceSensor* ps[2])
{
    float RightVal = ps[0]->getValue();
    float LeftVal = ps[1]->getValue();
    if (LeftVal <= SIDETHRESHOLD && RightVal <= SIDETHRESHOLD) return 0;
    if (LeftVal <= SIDETHRESHOLD) return 1;
    if (RightVal <= SIDETHRESHOLD) return 2;
    return 3;
}

bool CheckDistance(Odometer& encoder, PositionSensor* enc_l, PositionSensor* enc_r)
{
    float encR = enc_r->getValue();
    float encL = enc_l->getValue();
    float distanceR = (encR - encoder.pre_enc_r) * WheelRadius;
    float distanceL = (encL - encoder.pre_enc_l) * WheelRadius;
    encoder.pre_enc_l = encL;
    encoder.pre_enc_r = encR;
    encoder.Distance += (distanceR + distanceL) / 2.0f;
    float Error = TargetDist - encoder.Distance;
    if (Error <= 0.0001)
    {
        encoder.Distance = 0.0f;
        encoder.pre_enc_l = enc_l->getValue();
        encoder.pre_enc_r = enc_r->getValue();
        return true;
    }
    return false;
}

std::string InverseDir(std::string dir)
{
    if (dir == "leftturn") return "rightturn";
    else if (dir == "rightturn") return "leftturn";
    return dir;
}

std::string BackTrack(std::stack<std::string>& MemoryDir, std::stack<std::pair<int, int>>& MemPath, int(*Map)[SIZE], int& x, int& y, InertialUnit* imu, Motor* leftMotor, Motor* rightMotor, float& TargetYaw)
{
    if (MemoryDir.empty())
    {
        state = STOP;
        return("boom");
    }
    if (MemPath.empty()) 
    {
        state = STOP;
        return ("MemPathBoom");
    }
    auto pre_dir = MemoryDir.top();
    std::transform(pre_dir.begin(), pre_dir.end(), pre_dir.begin(), ::tolower);
    pre_dir = InverseDir(pre_dir);
    MemoryDir.pop();
    MemPath.pop();
    return pre_dir;
}

void Turn(InertialUnit* imu, Motor* leftMotor, Motor* rightMotor, float TargetYaw, int(*Map)[SIZE], int& x, int& y, Odometer& encoder, std::stack<std::string>& MemoryDir, std::stack<std::pair<int, int>>& MemPath, PositionSensor* enc_l, PositionSensor* enc_r)
{
    float Yaw = NormalizeAngle(imu->getRollPitchYaw()[2]);
    float Error = NormalizeAngle((TargetYaw - Yaw));
    float Derivative = (Error - PreTurnError);
    DerivativeError = (Derivative / TIMESTEP);
    if (abs(Error) <= 0.005) IntegralError = 0.0f;
    else IntegralError += std_clamp(Error, I_MAX, I_MIN);
    if (abs(Error) <= 0.001)
    {
        Stop(leftMotor, rightMotor);
        encoder.Distance = 0.0f;

        IntegralError = 0.0f;
        if (state == BT_TURN)
        {
            position = DIRECTION((position + 2) % 4);
            state = BT_DECIDE;
            return;
        }
        else if (state == BT_LEFTTURN)
        {
            state = BT_DECIDE;
            position = DIRECTION((position + 1) % 4);
            return;
        }
        else if (state == BT_RIGHTTURN)
        {
            state = BT_DECIDE;
            position = DIRECTION((position + 3) % 4);
            return;
        }
        else if (state == RIGHTTURN)
        {
            position = DIRECTION((position + 3) % 4);
            MemoryDir.push("rightturn");
            std::cout << "FRWD " << MemoryDir.top() << std::endl;
            MemPath.push({ x,y });
        }
        else if (state == LEFTTURN) 
        {
            position = DIRECTION((position + 1) % 4);
            MemoryDir.push("leftturn");
            std::cout << "FRWD " << MemoryDir.top() << std::endl;
            MemPath.push({ x,y });
        }
        state = DECIDE;
        return;
    }
    float speed = (Kp * Error) + (Ki * IntegralError) + (Kd * DerivativeError);
    speed = std_clamp(speed, Max_Speed, (-Max_Speed));
    leftMotor->setVelocity(-speed);
    rightMotor->setVelocity(speed);
    PreTurnError = Error;
    return;
}

void Forward(Motor* leftMotor, Motor* rightMotor, Odometer& encoder, PositionSensor* enc_l, PositionSensor* enc_r, int& x, int& y, int TargetX, int TargetY, int(*Map)[SIZE], int& preX, int& preY, std::stack<std::string>& MemoryDir, std::stack<std::pair<int, int>>& MemPath)
{
    if (state == FORWARD && CheckDistance(encoder, enc_l, enc_r)) 
    {
        x = TargetX;
        y = TargetY;
        Map[preX][preY] = VISITED;
        state = DECIDE;
        encoder.Distance = 0.0f;
        MemPath.push({ x,y });
        MemoryDir.push("forward");
        return;
    }
    else if (state == BT_FORWARD && CheckDistance(encoder, enc_l, enc_r))
    {
        Map[x][y] = DEAD_END;
        state = BT_DECIDE;
        encoder.Distance = 0.0f;
        auto d = MemPath.top();
        x = d.first;
        y = d.second;
        return;
    }
    leftMotor->setVelocity(Max_Speed);
    rightMotor->setVelocity(Max_Speed);
}

int IsSideVisited(int s, int x, int y, int(*Map)[SIZE], float& TargetYaw, InertialUnit* imu, Motor* leftMotor, Motor* rightMotor)
{
    int d = 0, l = 0, r = 0, px = 0, py = 0, lx = 0, rx = 0, ly = 0, ry = 0;
    if (s == 1)
    {
        d = DIRECTION((position + 1) % 4);
        px = x + dx[d];
        py = y + dy[d];
        if (IsValid(px, py, Map) && Map[px][py] == PATH) state = LEFTTURN;
        else
        {
            state = BT_TURN;
        }
    }
    else if (s == 2) 
    {
        d = DIRECTION((position + 3) % 4);
        px = x + dx[d];
        py = y + dy[d];
        if (IsValid(px, py, Map) && Map[px][py] == PATH) state = RIGHTTURN;
        else 
        {
            state = BT_TURN;
        }
    }
    else if (s == 0) {
        l = DIRECTION((position + 1) % 4); r = DIRECTION((position + 3) % 4);
        lx = x + dx[l]; ly = y + dy[l];
        rx = x + dx[r]; ry = y + dy[r];
        if (IsValid(lx, ly, Map) && Map[lx][ly] == PATH) state = LEFTTURN;
        else if (IsValid(rx, ry, Map) && Map[rx][ry] == PATH) state = RIGHTTURN;
        else if (Map[rx][ry] != PATH && Map[lx][ly] != PATH)
        {
            state = BT_TURN;
        }
        else
        {
            state = BT_TURN;
        }
    }
    else
    {
        state = BT_TURN;
        TargetYaw = GetTargetYaw(imu, leftMotor, rightMotor);
        Map[x][y] = VISITED;
        return 1;
    }
    TargetYaw = GetTargetYaw(imu, leftMotor, rightMotor);
    return 0;
}

void Decision(int& x, int& y, int(*Map)[SIZE], float& TargetYaw, DistanceSensor* ps[4], DistanceSensor* ps_side[2],
    InertialUnit* imu, Motor* leftMotor, Motor* rightMotor, int& TargetX, int& TargetY, int& PreX, int& PreY, 
    int(*WallMap)[SIZE], std::stack<std::string>& MemoryDir, Odometer& encoder, PositionSensor* enc_l, PositionSensor* enc_r)
{
    PreX = x;
    PreY = y;
    int nx = x + dx[position];
    int ny = y + dy[position];
    if (!IsValid(nx, ny, Map) || Map[nx][ny] == WALL || Map[nx][ny] == VISITED || Map[nx][ny] == DEAD_END ||
        CheckFrontWall(ps, leftMotor, rightMotor))
    {
        Stop(leftMotor, rightMotor);
        int SideWallVal = CheckSideWall(ps_side);
        IsSideVisited(SideWallVal, x, y, Map, TargetYaw, imu, leftMotor, rightMotor);
    }
    else 
    {
        TargetX = nx;
        TargetY = ny;
        state = FORWARD;
        return;
    }
}

void checkBT(std::string dir, float& TargetYaw, Motor* leftMotor, Motor* rightMotor, InertialUnit* imu)
{
    if (dir == "leftturn")
    {
        state = BT_LEFTTURN;
    }
    else if (dir == "rightturn") state = BT_RIGHTTURN;
    else if (dir == "forward")
    {
        state = BT_FORWARD;
        return;
    }
    else state = BT_TURN;
    TargetYaw = GetTargetYaw(imu, leftMotor, rightMotor);
    return;
}

void BackTrackDecision(int& x, int& y, int(*Map)[SIZE], std::stack<std::string>& MemoryDir, DistanceSensor* ps_side[2], 
    Motor* leftMotor, Motor* rightMotor, float& TargetYaw, InertialUnit* imu, std::stack<std::pair<int, int>>& MemPath, 
    DistanceSensor* ps_front[4])
{
    int SideWallVal = CheckSideWall(ps_side);
    int l = 0, r = 0, lx = 0, ly = 0, rx = 0, ry = 0, px = 0, py = 0, d = 0;
    int nx = 0, ny = 0;
    l = (position + 1) % 4; r = (position + 3) % 4;
    lx = x + dx[l]; ly = y + dy[l];
    rx = x + dx[r]; ry = y + dy[r];
    nx = x + dx[position]; ny = y + dy[position];

    if ((Map[lx][ly] != VISITED && Map[rx][ry] != VISITED) && Map[nx][ny] == DEAD_END)
    {
        state = BT_TURN;
        TargetYaw = GetTargetYaw(imu, leftMotor, rightMotor);
        return;
    }
    else 
    {
        if (SideWallVal == 3)
        {
            if (CheckFrontWall(ps_front, leftMotor, rightMotor))
            {
                state = BT_TURN;
                TargetYaw = GetTargetYaw(imu, leftMotor, rightMotor);
                return;
            }
            else 
            {
                checkBT(BackTrack(MemoryDir, MemPath, Map, x, y, imu, leftMotor, rightMotor, TargetYaw), TargetYaw, leftMotor, rightMotor, imu);
            }
        }
        else if (SideWallVal == 0) 
        {
            l = (position + 1) % 4; r = (position + 3) % 4;
            lx = x + dx[l]; ly = y + dy[l];
            rx = x + dx[r]; ry = y + dy[r];
            if (IsValid(lx, ly, Map) && Map[lx][ly] == PATH)
            {
                state = LEFTTURN;
                TargetYaw = GetTargetYaw(imu, leftMotor, rightMotor);
                return;
            }
            else if (IsValid(rx, ry, Map) && Map[rx][ry] == PATH)
            {
                state = RIGHTTURN;
                TargetYaw = GetTargetYaw(imu, leftMotor, rightMotor);
                return;
            }
            else if (CheckFrontWall(ps_front, leftMotor, rightMotor) && Map[lx][ly] != PATH && Map[rx][ry] != PATH)
            {
                state = BT_TURN;
                TargetYaw = GetTargetYaw(imu, leftMotor, rightMotor);
                return;
            }
            else if (Map[rx][ry] != PATH && Map[lx][ly] != PATH)
            {
                 checkBT(BackTrack(MemoryDir, MemPath, Map, x, y, imu, leftMotor, rightMotor, TargetYaw), TargetYaw, leftMotor, rightMotor, imu);
            }
            else
            {
                state = STOP;
                return;
            }
        }
        else if (SideWallVal == 1) 
        {
            d = DIRECTION(position + 1) % 4;
            px = x + dx[d];
            py = y + dy[d];
            if (IsValid(px, py, Map) && Map[px][py] == PATH)
            {
                state = LEFTTURN;
                TargetYaw = GetTargetYaw(imu, leftMotor, rightMotor);
                return;
            }
            else if ((Map[px][py] == VISITED || Map[px][py] == DEAD_END))
            {
                checkBT(BackTrack(MemoryDir, MemPath, Map, x, y, imu, leftMotor, rightMotor, TargetYaw), TargetYaw, leftMotor, rightMotor, imu);
            }
            else
            {
                state = STOP;
                return;
            }
        }
        else if (SideWallVal == 2)
        {
            d = DIRECTION(position + 3) % 4;
            px = x + dx[d];
            py = y + dy[d];
            if (IsValid(px, py, Map) && Map[px][py] == PATH)
            {
                state = RIGHTTURN;
                TargetYaw = GetTargetYaw(imu, leftMotor, rightMotor);
                return;
            }
            else if ((Map[px][py] == VISITED || Map[px][py] == DEAD_END))
            {
                checkBT(BackTrack(MemoryDir, MemPath, Map, x, y, imu, leftMotor, rightMotor, TargetYaw), TargetYaw, leftMotor, rightMotor, imu);
            }
            else 
            {
                state = STOP;
                return; 
            }
        }
    }
}

int main(int argv, char** argc) {
    Robot* robot = new Robot();
    Motor* leftMotor = robot->getMotor("left wheel motor");
    Motor* rightMotor = robot->getMotor("right wheel motor");
    DistanceSensor* ps[4];
    DistanceSensor* ps_side[2];
    PositionSensor* enc_l = leftMotor->getPositionSensor();
    PositionSensor* enc_r = rightMotor->getPositionSensor();
    InertialUnit* imu = robot->getInertialUnit("inertial unit");
    Odometer encoder;
    std::string ps_name[4] = { "ps0", "ps7", "ps1", "ps6" };
    std::string ps_side_name[2] = { "ps2", "ps5" };

    imu->enable(TIMESTEP);
    enc_l->enable(TIMESTEP);
    enc_r->enable(TIMESTEP);
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    for (int i = 0; i < 4; i++)
    {
        ps[i] = robot->getDistanceSensor(ps_name[i]);
        ps[i]->enable(TIMESTEP);
    }
    for (int i = 0; i < 2; i++)
    {
        ps_side[i] = robot->getDistanceSensor(ps_side_name[i]);
        ps_side[i]->enable(TIMESTEP);
    }

    encoder.pre_enc_l = 0.0f;
    encoder.pre_enc_r = 0.0f;
    std::stack<std::string>MemoryDir;
    std::stack<std::pair<int, int>> MemPath;
    int Map[SIZE][SIZE] = { 0 };
    int WallMap[SIZE][SIZE] = { 0 };

    for (int i = 0; i < SIZE; i++)
    {
        for (int j = 0; j < SIZE; j++)
        {
            if (i == 0) 
            {
                Map[i][j] = WALL;
                //WallMap[i][j] = WALL;
            }
            if (i == (SIZE - 1))
            {
                Map[i][j] = WALL;
                //WallMap[i][j] = WALL;
            }
            if (j == 0)
            {
                Map[i][j] = WALL;
                //WallMap[i][j] = WALL;
            }
            if (j == (SIZE - 1)) 
            {
                Map[i][j] = WALL;
                //WallMap[i][j] = WALL;
            }
        }
    }

    int x = 1, y = 1, TargetX = x, TargetY = y, PreX = 0, PreY = 0;
    int endX = 2, endY = 2;
    MemPath.push({ x,y });
    float TargetYaw = 0.0f;

    while (robot->step(TIMESTEP) != -1)
    {
        switch (state)
        {
        case(DECIDE):
            Decision(x, y, Map, TargetYaw, ps, ps_side, imu, leftMotor, rightMotor, TargetX, TargetY, PreX, PreY, WallMap, MemoryDir, encoder, enc_l, enc_r);
            break;

        case(LEFTTURN):
            Turn(imu, leftMotor, rightMotor, TargetYaw, Map, x, y, encoder, MemoryDir, MemPath, enc_l, enc_r);
            break;

        case(RIGHTTURN):
            Turn(imu, leftMotor, rightMotor, TargetYaw, Map, x, y, encoder, MemoryDir, MemPath, enc_l, enc_r);
            break;
        case(FORWARD):
            if (CheckFrontWall(ps, leftMotor, rightMotor))
            {
                Stop(leftMotor, rightMotor);
                int SideWallVal = CheckSideWall(ps_side);
                std::cout << "AA" << std::endl;
                IsSideVisited(SideWallVal, x, y, Map, TargetYaw, imu, leftMotor, rightMotor);
            }
            Forward(leftMotor, rightMotor, encoder, enc_l, enc_r, x, y, TargetX, TargetY, Map, PreX, PreY, MemoryDir, MemPath);
            break;

        case(STOP):
            Stop(leftMotor, rightMotor);
            break;

        case(BT_DECIDE):
            BackTrackDecision(x, y, Map, MemoryDir, ps_side, leftMotor, rightMotor, TargetYaw, imu, MemPath, ps);
            break;

        case(BT_TURN):
            Turn(imu, leftMotor, rightMotor, TargetYaw, Map, x, y, encoder, MemoryDir, MemPath, enc_l, enc_r);
            break;

        case(BT_LEFTTURN):
            Turn(imu, leftMotor, rightMotor, TargetYaw, Map, x, y, encoder, MemoryDir, MemPath, enc_l, enc_r);
            break;

        case(BT_RIGHTTURN):
            Turn(imu, leftMotor, rightMotor, TargetYaw, Map, x, y, encoder, MemoryDir, MemPath, enc_l, enc_r);
            break;

        case(BT_FORWARD):
            Forward(leftMotor, rightMotor, encoder, enc_l, enc_r, x, y, TargetX, TargetY, Map, PreX, PreY, MemoryDir, MemPath);
            break;
        }

        for (int i = 0; i < SIZE; i++)
        {
            for (int j = 0; j < SIZE; j++)
            {
                std::cout << Map[i][j];
            }
            std::cout << std::endl;
        }
    }

    delete robot;
    return 0;
}


// PID gain for Accurate Turning
// Best Combination of Gains: Kp = 2.9f, Ki = 0.18f, Kd = 1.3f