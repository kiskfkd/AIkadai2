#include "Enemy.h"
#include "Stage.h"
#include "Player.h"
#include "globals.h"
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <queue>
#include <map>
#include <stack>
#include <string>
#include "ImGui/imgui.h"
//　ループなし

namespace {
    Point nDir[4] = { {0, -1}, {0, 1}, {-1, 0}, {1, 0} };
}

// 右手法
namespace RightHandRule {
    // 右手法での移動方向
    const int dx[4] = { 1, 0, -1, 0 };
    const int dy[4] = { 0, 1, 0, -1 };

    // 右手法での次の方向
    int nextDir(int dir) {
        return (dir + 3) % 4;
    }

    // 右手法での前方の方向
    int forwardDir(int dir) {
        return dir;
    }

    // 右手法での左方向
    int leftDir(int dir) {
        return (dir + 1) % 4;
    }

    // 右手法での右方向
    int rightDir(int dir) {
        return (dir + 3) % 4;
    }

    // 右手法での移動
    Point move(int x, int y, int dir) {
        return { x + dx[dir], y + dy[dir] };
    }

    // 右手法での移動可能かどうか
    bool canMove(int x, int y, int dir, Stage* stage) {
        Point next = move(x, y, dir);
        return stage->IsPassable(next.x, next.y);
    }

    // 右手法での次の方向を探す
    int findNextDir(int x, int y, int dir, Stage* stage) {
        for (int i = 0; i < 4; i++) {
            int next = rightDir(dir);
            if (canMove(x, y, next, stage))
                return next;
            dir = next;
        }
        return -1;
    }

    // 右手法での経路を求める
    std::vector<Point> findPath(Point start, Point goal, Stage* stage) {
        std::vector<Point> path;
        int x = start.x;
        int y = start.y;
        int dir = 0;
        while (true) {
            path.push_back({ x, y });

            // ゴールに到達したら終了
            if (x == goal.x && y == goal.y)
                break;

            // 右手法に従って移動方向を決定
            int right = rightDir(dir);
            int forward = forwardDir(dir);
            int left = leftDir(dir);

            if (canMove(x, y, right, stage)) {
                dir = right;
            }
            else if (canMove(x, y, forward, stage)) {
                // そのまま前進
            }
            else if (canMove(x, y, left, stage)) {
                dir = left;
            }
            else {
                // どこにも進めない場合、180度回転
                dir = (dir + 2) % 4;
            }

            // 次の位置へ移動
            Point next = move(x, y, dir);
            x = next.x;
            y = next.y;
        }
        return path;
    }
}
// 左手法
namespace LeftHandRule {

    // 移動方向（右手法と同じ）
    const int dx[4] = { 1, 0, -1, 0 };
    const int dy[4] = { 0, 1, 0, -1 };

    // 左手法での次の方向
    int nextDir(int dir) {
        return (dir + 1) % 4;
    }

    // 左手法での前方方向
    int forwardDir(int dir) {
        return dir;
    }

    // 左手法での右方向
    int rightDir(int dir) {
        return (dir + 3) % 4;
    }

    // 左手法での左方向
    int leftDir(int dir) {
        return (dir + 1) % 4;
    }

    // 左手法での移動
    Point move(int x, int y, int dir) {
        return { x + dx[dir], y + dy[dir] };
    }

    // 左手法での移動可能判定
    bool canMove(int x, int y, int dir, Stage* stage) {
        Point next = move(x, y, dir);
        return stage->IsPassable(next.x, next.y);
    }

    // 左手法での次の移動方向を決定
    int findNextDir(int x, int y, int dir, Stage* stage) {
        for (int i = 0; i < 4; i++) {
            int next = leftDir(dir); // まず左を優先
            if (canMove(x, y, next, stage))
                return next;
            dir = next; // 次の方向を試す
        }
        return -1;
    }

    // 左手法での経路探索
    std::vector<Point> findPath(Point start, Point goal, Stage* stage) {
        std::vector<Point> path;

        // 初期位置
        int x = start.x;
        int y = start.y;

        // 初期方向（右を基準）
        int dir = 0;

        while (true) {
            path.push_back({ x, y });

            // ゴールに到達したら終了
            if (x == goal.x && y == goal.y)
                break;

            // 左手法に従って移動方向を決定
            int left = leftDir(dir);
            int forward = forwardDir(dir);
            int right = rightDir(dir);

            if (canMove(x, y, left, stage)) {
                dir = left;
            }
            else if (canMove(x, y, forward, stage)) {
                // そのまま前進
            }
            else if (canMove(x, y, right, stage)) {
                dir = right;
            }
            else {
                // どこにも進めない場合、180度回転
                dir = (dir + 2) % 4;
            }

            // 次の位置へ移動
            Point next = move(x, y, dir);
            x = next.x;
            y = next.y;
        }
        return path;
    }

}

// 幅優先探索
namespace BreadthFirstSearch {
    std::vector<Point> FindPath(Stage* stage, Point start, Point goal) {
        std::vector<Point> path;
        if (!stage) return path;

        std::queue<Point> queue;
        std::map<Point, Point> cameFrom;  // 経路記録用
        queue.push(start);
        cameFrom[start] = start;  // 開始地点の親を自身に設定

        Point directions[] = {
            {0, -1}, {1, 0}, {0, 1}, {-1, 0}  // 上, 右, 下, 左
        };

        while (!queue.empty()) {
            Point current = queue.front();
            queue.pop();

            // ゴールに到達
            if (current.x == goal.x && current.y == goal.y) {
                break;
            }

            for (const auto& dir : directions) {
                Point next = { current.x + dir.x, current.y + dir.y };

                // 範囲外または通行不可ならスキップ
                if (!stage->IsPassable(next.x, next.y)) continue;

                // まだ通過していない場合
                if (cameFrom.find(next) == cameFrom.end()) {
                    queue.push(next);
                    cameFrom[next] = current;
                }
            }
        }

        // 経路を逆順でたどる
        if (cameFrom.find(goal) != cameFrom.end()) {
            Point current = goal;
            while (current.x != start.x || current.y != start.y) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
        }

        return path;  // 最短経路を返す
    }

}

// 深さ優先探索
namespace DepthFirstSearch {
    std::vector<Point> FindPath(Stage* stage, Point start, Point goal) {
        std::vector<Point> path;
        if (!stage) return path;

        std::stack<Point> stack;
        std::map<Point, Point> cameFrom; // 経路復元用マップ
        stack.push(start);
        cameFrom[start] = start;

        // 4方向の移動
        Point directions[] = {
            {0, -1}, {1, 0}, {0, 1}, {-1, 0}  // 上, 右, 下, 左
        };

        while (!stack.empty()) {
            Point current = stack.top();
            stack.pop();

            // ゴールに到達したら探索終了
            if (current.x == goal.x && current.y == goal.y) {
                break;
            }

            // 隣接するマスをチェック
            for (const auto& dir : directions) {
                Point next = { current.x + dir.x, current.y + dir.y };

                // 通行可能かチェック
                if (!stage->IsPassable(next.x, next.y)) continue;

                // まだ通過していない場合
                if (cameFrom.find(next) == cameFrom.end()) {
                    stack.push(next);
                    cameFrom[next] = current;
                }
            }
        }

        // 経路を逆順でたどる
        if (cameFrom.find(goal) != cameFrom.end()) {
            Point current = goal;
            while (current.x != start.x || current.y != start.y) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
        }
        return path;  // 経路を返す
    }
}

// ダイクストラ法
namespace Dijkstra {

    // 優先度付きキュー用の比較関数（距離が小さい順に処理）
    struct Compare {
        bool operator()(const std::pair<int, Point>& a, const std::pair<int, Point>& b) {
            return a.first > b.first;
        }
    };

    std::vector<Point> FindPath(Stage* stage, Point start, Point goal) {
        std::vector<Point> path;
        if (!stage) return path;

        // 各座標の最短距離を格納するマップ
        std::map<Point, int> cost;
        std::map<Point, Point> cameFrom;  // 経路復元用マップ
        std::priority_queue<std::pair<int, Point>, std::vector<std::pair<int, Point>>, Compare> pq;

        // 初期設定
        pq.push({ 0, start });
        cost[start] = 0;
        cameFrom[start] = start;

        // 4方向の移動
        Point directions[] = {
            {0, -1}, {1, 0}, {0, 1}, {-1, 0}  // 上, 右, 下, 左
        };

        while (!pq.empty()) {
            auto [currentCost, current] = pq.top();
            pq.pop();

            // ゴールに到達したら探索終了
            if (current.x == goal.x && current.y == goal.y) {
                break;
            }

            // 隣接するマスをチェック
            for (const auto& dir : directions) {
                Point next = { current.x + dir.x, current.y + dir.y };

                // 通行可能かチェック
                if (!stage->IsPassable(next.x, next.y)) continue;

                // コストを計算（すべての移動はコスト1）
                int newCost = cost[current] + 1;

                // より短い距離が見つかった場合、更新
                if (cost.find(next) == cost.end() || newCost < cost[next]) {
                    cost[next] = newCost;
                    pq.push({ newCost, next });
                    cameFrom[next] = current;
                }
            }
        }

        // 経路を逆順でたどる
        if (cameFrom.find(goal) != cameFrom.end()) {
            Point current = goal;
            while (current.x != start.x || current.y != start.y) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
        }

        return path;  // 最短経路を返す
    }
}

// A*
namespace AStar {

    struct Node {
        int x, y;
        int g, h, f;
        Node* parent;
        Node(int _x, int _y, int _g, int _h, Node* _parent)
            : x(_x), y(_y), g(_g), h(_h), f(_g + _h), parent(_parent) {}
    };

    // マンハッタン距離
    int manhattan(int x1, int y1, int x2, int y2) {
        return abs(x1 - x2) + abs(y1 - y2);
    }


    Node* findNode(const std::vector<Node*>& list, int x, int y) {
        for (Node* node : list) {
            if (node->x == x && node->y == y)
                return node;
        }
        return nullptr;
    }

    // startからgoalまでの経路
    std::vector<Point> findPath(Point start, Point goal, Stage* stage) {
        std::vector<Point> path;
        std::vector<Node*> openList;
        std::vector<Node*> closedList;

        Node* startNode = new Node(start.x, start.y, 0, manhattan(start.x, start.y, goal.x, goal.y), nullptr);
        openList.push_back(startNode);

        bool pathFound = false;
        Node* goalNode = nullptr;

        while (!openList.empty()) {
            auto currentIt = std::min_element(openList.begin(), openList.end(),
                [](Node* a, Node* b) { return a->f < b->f; });
            Node* current = *currentIt;

            if (current->x == goal.x && current->y == goal.y) {
                pathFound = true;
                goalNode = current;
                break;
            }

            openList.erase(currentIt);
            closedList.push_back(current);

            // 上下左右
            const int dx[4] = { -1, 1, 0, 0 };
            const int dy[4] = { 0, 0, -1, 1 };
            for (int i = 0; i < 4; i++) {
                int nx = current->x + dx[i];
                int ny = current->y + dy[i];

                if (nx < 0 || ny < 0 || nx >= STAGE_WIDTH || ny >= STAGE_HEIGHT)
                    continue;
                if (!stage->IsPassable(nx, ny))
                    continue;
                if (findNode(closedList, nx, ny))
                    continue;
                int tentativeG = current->g + 1;
                Node* neighbor = findNode(openList, nx, ny);
                if (!neighbor) {
                    int h = manhattan(nx, ny, goal.x, goal.y);
                    neighbor = new Node(nx, ny, tentativeG, h, current);
                    openList.push_back(neighbor);
                }
                else {
                    if (tentativeG < neighbor->g) {
                        neighbor->g = tentativeG;
                        neighbor->f = neighbor->g + neighbor->h;
                        neighbor->parent = current;
                    }
                }
            }
        }

        if (pathFound && goalNode) {
            Node* current = goalNode;
            while (current) {
                path.push_back({ current->x, current->y });
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
        }

        for (Node* node : openList)
            delete node;
        for (Node* node : closedList)
            delete node;

        return path;
    }
}

Enemy::Enemy()
    : pos_({ 0, 0 }), isAlive_(true), targetPos_({ 0, 0 }),
    pathIndex_(0), forward_(RIGHT),
    chaseMode_(ENEMYMODE::RANDOM)
{
    path_.clear(); // 経路をクリア
    int rx = 0, ry = 0;
    while (rx % 2 == 0 || ry % 2 == 0) {
        rx = GetRand(STAGE_WIDTH - 1);
        ry = GetRand(STAGE_HEIGHT - 1);
    }
    pos_ = { rx * CHA_WIDTH, ry * CHA_HEIGHT };
}

Enemy::~Enemy()
{
}
void Enemy::Imgui()
{
    ImGui::Begin(u8"敵のモード");
    const char* modeNames[] = { u8"ランダム", u8"右手", u8"左手", u8"幅優先探索", u8"深さ優先探索", u8"ダイクストラ法", u8"A*" };
    const char* currentMode = modeNames[static_cast<int>(chaseMode_)];

    // Player との距離を計算
    Player* player = (Player*)FindGameObject<Player>();
    if (player) {
        Point playerPos = player->GetPosition();
        int distanceX = abs(pos_.x - playerPos.x);
        int distanceY = abs(pos_.y - playerPos.y);
        int distanceThreshold = CHA_WIDTH / 2; // 近づいたと判定する閾値
        // 追いついたかどうかの判定
        if (distanceX < distanceThreshold && distanceY < distanceThreshold) {
            ImGui::Text(u8"現在は、Playerに追いつきました");
        }
        else {
            ImGui::Text(u8"現在は、%s ", currentMode);
        }
    }
    else {
        ImGui::Text(u8"現在は、%s です", currentMode);
    }

    Stage* stage = (Stage*)FindGameObject<Stage>();
    if (!stage) {
        ImGui::Text(u8"ステージ情報が取得できません");
        ImGui::End();
        return;
    }
    
    for (int i = 0; i < MAX_ENEMYMODE; i++) {
        if (ImGui::Button(modeNames[i])) {
            chaseMode_ = (ENEMYMODE)i;  // モードを変更
            pos_ = stage->GetRandomEmptyPosition(); // リスポン
            path_.clear();
            pathIndex_ = 0;
        }
    }

    ImGui::End();
}
void Enemy::Update()
{
    Stage* stage = (Stage*)FindGameObject<Stage>();
    Player* player = (Player*)FindGameObject<Player>();
    if (!stage || !player)
        return;

    Point playerPos = player->GetPosition();
    Point enemyGrid = { pos_.x / CHA_WIDTH, pos_.y / CHA_HEIGHT };
    Point playerGrid = { playerPos.x / CHA_WIDTH, playerPos.y / CHA_HEIGHT };

    Point oldPos = pos_;
    int enemySpeed = 1; // 移動速度（必要に応じて調整してください）

    switch (chaseMode_) {
    case ENEMYMODE::RANDOM: {
        // ランダム移動（既存の処理）
        Point move = { nDir[forward_].x, nDir[forward_].y };
        pos_.x += move.x;
        pos_.y += move.y;

        Rect eRect = { pos_.x, pos_.y, CHA_WIDTH, CHA_HEIGHT };
        for (auto& obj : stage->GetStageRects()) {
            if (CheckHit(eRect, obj)) {
                pos_ = oldPos;
                forward_ = (DIR)(GetRand(4));
                break;
            }
        }
        if ((pos_.x % CHA_WIDTH == 0) && (pos_.y % CHA_HEIGHT == 0)) {
            forward_ = (DIR)(GetRand(4));
        }
        break;
    }
    case ENEMYMODE::RIGHT_HAND: {
        // 右手法（既存のスムーズ移動処理）
        if (path_.empty() || pathIndex_ >= path_.size() ||
            (path_.back().x != playerGrid.x || path_.back().y != playerGrid.y)) {
            path_ = RightHandRule::findPath(enemyGrid, playerGrid, stage);
            pathIndex_ = 0;
        }

        if (!path_.empty() && pathIndex_ < path_.size()) {
            Point nextNode = path_[pathIndex_];
            Point targetPixel = { nextNode.x * CHA_WIDTH, nextNode.y * CHA_HEIGHT };

            // X軸方向の移動
            if (pos_.x < targetPixel.x) {
                pos_.x += enemySpeed;
                if (pos_.x > targetPixel.x)
                    pos_.x = targetPixel.x;
            }
            else if (pos_.x > targetPixel.x) {
                pos_.x -= enemySpeed;
                if (pos_.x < targetPixel.x)
                    pos_.x = targetPixel.x;
            }

            // Y軸方向の移動
            if (pos_.y < targetPixel.y) {
                pos_.y += enemySpeed;
                if (pos_.y > targetPixel.y)
                    pos_.y = targetPixel.y;
            }
            else if (pos_.y > targetPixel.y) {
                pos_.y -= enemySpeed;
                if (pos_.y < targetPixel.y)
                    pos_.y = targetPixel.y;
            }

            // 目標位置に到達したら次のノードへ
            if (pos_.x == targetPixel.x && pos_.y == targetPixel.y) {
                pathIndex_++;
            }
        }
        break;
    }
    case ENEMYMODE::LEFT_HAND: {
        // 左手法（既存のスムーズ移動処理）
        if (path_.empty() || pathIndex_ >= path_.size() ||
            (path_.back().x != playerGrid.x || path_.back().y != playerGrid.y)) {
            path_ = LeftHandRule::findPath(enemyGrid, playerGrid, stage);
            pathIndex_ = 0;
        }

        if (!path_.empty() && pathIndex_ < path_.size()) {
            Point nextNode = path_[pathIndex_];
            Point targetPixel = { nextNode.x * CHA_WIDTH, nextNode.y * CHA_HEIGHT };

            // X軸方向の移動
            if (pos_.x < targetPixel.x) {
                pos_.x += enemySpeed;
                if (pos_.x > targetPixel.x)
                    pos_.x = targetPixel.x;
            }
            else if (pos_.x > targetPixel.x) {
                pos_.x -= enemySpeed;
                if (pos_.x < targetPixel.x)
                    pos_.x = targetPixel.x;
            }

            // Y軸方向の移動
            if (pos_.y < targetPixel.y) {
                pos_.y += enemySpeed;
                if (pos_.y > targetPixel.y)
                    pos_.y = targetPixel.y;
            }
            else if (pos_.y > targetPixel.y) {
                pos_.y -= enemySpeed;
                if (pos_.y < targetPixel.y)
                    pos_.y = targetPixel.y;
            }

            // 目標位置に到達したら次のノードへ
            if (pos_.x == targetPixel.x && pos_.y == targetPixel.y) {
                pathIndex_++;
            }
        }
        break;
    }
    case ENEMYMODE::BFS: {
        // 幅優先探索：経路が空の場合やプレイヤー位置が変わった場合に再計算し、目標に向かって徐々に移動
        if (path_.empty() || pathIndex_ >= path_.size() ||
            (path_.back().x != playerGrid.x || path_.back().y != playerGrid.y)) {
            path_ = BreadthFirstSearch::FindPath(stage, enemyGrid, playerGrid);
            pathIndex_ = 0;
        }

        if (!path_.empty() && pathIndex_ < path_.size()) {
            Point nextNode = path_[pathIndex_];
            Point targetPixel = { nextNode.x * CHA_WIDTH, nextNode.y * CHA_HEIGHT };

            if (pos_.x < targetPixel.x) {
                pos_.x += enemySpeed;
                if (pos_.x > targetPixel.x)
                    pos_.x = targetPixel.x;
            }
            else if (pos_.x > targetPixel.x) {
                pos_.x -= enemySpeed;
                if (pos_.x < targetPixel.x)
                    pos_.x = targetPixel.x;
            }

            if (pos_.y < targetPixel.y) {
                pos_.y += enemySpeed;
                if (pos_.y > targetPixel.y)
                    pos_.y = targetPixel.y;
            }
            else if (pos_.y > targetPixel.y) {
                pos_.y -= enemySpeed;
                if (pos_.y < targetPixel.y)
                    pos_.y = targetPixel.y;
            }

            if (pos_.x == targetPixel.x && pos_.y == targetPixel.y) {
                pathIndex_++;
            }
        }
        break;
    }
    case ENEMYMODE::DFS: {
        // 深さ優先探索：経路再計算とスムーズ移動
        if (path_.empty() || pathIndex_ >= path_.size() ||
            (path_.back().x != playerGrid.x || path_.back().y != playerGrid.y)) {
            path_ = DepthFirstSearch::FindPath(stage, enemyGrid, playerGrid);
            pathIndex_ = 0;
        }

        if (!path_.empty() && pathIndex_ < path_.size()) {
            Point nextNode = path_[pathIndex_];
            Point targetPixel = { nextNode.x * CHA_WIDTH, nextNode.y * CHA_HEIGHT };

            if (pos_.x < targetPixel.x) {
                pos_.x += enemySpeed;
                if (pos_.x > targetPixel.x)
                    pos_.x = targetPixel.x;
            }
            else if (pos_.x > targetPixel.x) {
                pos_.x -= enemySpeed;
                if (pos_.x < targetPixel.x)
                    pos_.x = targetPixel.x;
            }

            if (pos_.y < targetPixel.y) {
                pos_.y += enemySpeed;
                if (pos_.y > targetPixel.y)
                    pos_.y = targetPixel.y;
            }
            else if (pos_.y > targetPixel.y) {
                pos_.y -= enemySpeed;
                if (pos_.y < targetPixel.y)
                    pos_.y = targetPixel.y;
            }

            if (pos_.x == targetPixel.x && pos_.y == targetPixel.y) {
                pathIndex_++;
            }
        }
        break;
    }
    case ENEMYMODE::DIJKSTRA: {
        // ダイクストラ法：経路再計算と目標への徐々の移動
        if (path_.empty() || pathIndex_ >= path_.size() ||
            (path_.back().x != playerGrid.x || path_.back().y != playerGrid.y)) {
            path_ = Dijkstra::FindPath(stage, enemyGrid, playerGrid);
            pathIndex_ = 0;
        }

        if (!path_.empty() && pathIndex_ < path_.size()) {
            Point nextNode = path_[pathIndex_];
            Point targetPixel = { nextNode.x * CHA_WIDTH, nextNode.y * CHA_HEIGHT };

            if (pos_.x < targetPixel.x) {
                pos_.x += enemySpeed;
                if (pos_.x > targetPixel.x)
                    pos_.x = targetPixel.x;
            }
            else if (pos_.x > targetPixel.x) {
                pos_.x -= enemySpeed;
                if (pos_.x < targetPixel.x)
                    pos_.x = targetPixel.x;
            }

            if (pos_.y < targetPixel.y) {
                pos_.y += enemySpeed;
                if (pos_.y > targetPixel.y)
                    pos_.y = targetPixel.y;
            }
            else if (pos_.y > targetPixel.y) {
                pos_.y -= enemySpeed;
                if (pos_.y < targetPixel.y)
                    pos_.y = targetPixel.y;
            }

            if (pos_.x == targetPixel.x && pos_.y == targetPixel.y) {
                pathIndex_++;
            }
        }
        break;
    }
    case ENEMYMODE::ASTAR: {
        // A*：経路再計算とスムーズ移動
        if (path_.empty() || pathIndex_ >= path_.size() ||
            (path_.back().x != playerGrid.x || path_.back().y != playerGrid.y)) {
            path_ = AStar::findPath(enemyGrid, playerGrid, stage);
            pathIndex_ = 0;
        }

        if (!path_.empty() && pathIndex_ < path_.size()) {
            Point nextNode = path_[pathIndex_];
            Point targetPixel = { nextNode.x * CHA_WIDTH, nextNode.y * CHA_HEIGHT };

            if (pos_.x < targetPixel.x) {
                pos_.x += enemySpeed;
                if (pos_.x > targetPixel.x)
                    pos_.x = targetPixel.x;
            }
            else if (pos_.x > targetPixel.x) {
                pos_.x -= enemySpeed;
                if (pos_.x < targetPixel.x)
                    pos_.x = targetPixel.x;
            }

            if (pos_.y < targetPixel.y) {
                pos_.y += enemySpeed;
                if (pos_.y > targetPixel.y)
                    pos_.y = targetPixel.y;
            }
            else if (pos_.y > targetPixel.y) {
                pos_.y -= enemySpeed;
                if (pos_.y < targetPixel.y)
                    pos_.y = targetPixel.y;
            }

            if (pos_.x == targetPixel.x && pos_.y == targetPixel.y) {
                pathIndex_++;
            }
        }
        break;
    }
    default:
        break;
    }
}



void Enemy::Draw()
{
    DrawBox(pos_.x, pos_.y, pos_.x + CHA_WIDTH, pos_.y + CHA_HEIGHT, GetColor(180, 189, 110), TRUE);
    Imgui();
}

bool Enemy::CheckHit(const Rect& me, const Rect& other)
{
    if (me.x < other.x + other.w && me.x + me.w > other.x && me.y < other.y + other.h && me.y + me.h > other.y) {
        return true;
    }
    return false;
}
