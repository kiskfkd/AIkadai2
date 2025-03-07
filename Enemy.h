#pragma once
#include "./Library/GameObject.h"
#include "./globals.h"
#include <vector>

class Enemy :
    public GameObject
{
private:


    Point pos_;
    bool isAlive_;

    Point targetPos_;

    // A*‚ÌŒo˜H
    std::vector<Point> path_;
    int pathIndex_;

    DIR forward_;

    ENEMYMODE chaseMode_;
public:
    Enemy();
    ~Enemy();

    void Update() override;
    void Draw() override;
    bool CheckHit(const Rect& me, const Rect& other);

    void Imgui();
   
};