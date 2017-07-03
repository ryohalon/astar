// aster.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <list>

// 2次元マップの幅と高さ
const int MW = 5;
const int MH = 5;

// マップステータス
enum {
	OPEN,
	CLOSED,
	B,      // 壁
	N,      // 空間
	S,      // スタート
	G       // ゴール
};

// XYをひとまとめに扱うため
class Point {
public:
	Point() : x(0), y(0) {}
	Point(int _x, int _y) : x(_x), y(_y) {}
	int x;
	int y;

	Point operator + (Point p) {
		return Point(x + p.x, y + p.y);
	}

	bool operator == (Point p) {
		if (x == p.x && y == p.y) return true;
		return false;
	}


};

// 経路探索用ノード
class Node {
public:
	Node() :
		status(N),
		cost_real(0),
		cost_guess(0),
		score(0),
		parent(nullptr)
	{}

	Point pos;         // 2次元配列上の座標
	int status;        // OPEN やら CLOSED やら
	int cost_real;     // 実コスト
	int cost_guess;    // 推定コスト
	int score;         // スコア
	Node* parent;      // 親ノード( 最後にゴールから辿るため )

	bool operator < (const Node& node) const {
		return score > node.score;
	}

};


// マップデータ
// ゲームのマップ情報
int map[MH][MW] = {
	{ N, N, N, N, N },
	{ N, N, B, N, N },
	{ N, N, B, N, N },
	{ N, B, N, B, N },
	{ N, N, S, B, G }
};

// マップのノードデータ
// 上の2次元配列からこの
// ノード情報を作成して経路探索に渡す
Node nodes[MH][MW];

std::list<Point> open_node_list;


// 指定座標が有効な( OPEN 可能な )マップ位置かどうか判定
bool isEnableMapPosition(Point pos, Node **_nodes) {
	if (pos.x < 0) return false;
	if (pos.y < 0) return false;
	if (pos.x >= MW) return false;
	if (pos.y >= MH) return false;
	if (S == _nodes[pos.y][pos.x].status) return false;
	if (N == _nodes[pos.y][pos.x].status) return true;
	if (G == _nodes[pos.y][pos.x].status) return true;
	return false;
}


// [ 現在オープンになっているノードで一番スコアの小さいものを取得 ]
// サンプルの分かり易さ優先なので全ノードを調べる形にしているが、
// 効率は良くないのでライブラリ化する時にでも 既にオープンしている
// ノードだけ調べるように効率化を図った方がよい
Node* getSmallScoreNodeFromOpenNodes(Node **_nodes) {
	Node* p = nullptr;
	std::list<Point>::iterator erase_itr;
	for (std::list<Point>::iterator itr = open_node_list.begin(); itr != open_node_list.end(); ++itr)
	{
		if (nullptr == p)
		{
			p = &_nodes[itr->y][itr->x];
			erase_itr = itr;
		}
		if (p->score > _nodes[itr->y][itr->x].score)
		{
			p = &_nodes[itr->y][itr->x];
			erase_itr = itr;
		}
		else if (p->score == _nodes[itr->y][itr->x].score)
		{
			if (p->cost_real > _nodes[itr->y][itr->x].cost_real)
			{
				p = &_nodes[itr->y][itr->x];
				erase_itr = itr;
			}
		}
	}

	open_node_list.erase(erase_itr);

	/*for (int i = 0; i < MH; ++i) {
	for (int k = 0; k < MW; ++k) {
	if (OPEN != _nodes[k][i].status) continue;
	if (nullptr == p) p = &_nodes[k][i];
	if (p->score > _nodes[k][i].score) p = &_nodes[k][i];
	else if (p->score == _nodes[k][i].score) {
	if (p->cost_real > _nodes[k][i].cost_real) p = &_nodes[k][i];
	}
	}
	}*/
	return p;
}


// 経路探索 A*
bool aster(Node **_nodes, Node *_now, std::vector<Node*> *_route) {

	// スタート地点のスコア計算
	if (S == _now->status) {
		_now->score = _now->cost_real + _now->cost_guess;
	}

	// ４方向の座標
	Point dir[8] = { Point(0, 1), Point(1, 0), Point(0, -1), Point(-1, 0), Point(1, -1), Point(-1, -1), Point(-1,1), Point(1, 1) };

	// 周り４方向を調べて可能ならオープン
	for (int i = 0; i < 8; ++i) {
		Point next = _now->pos + dir[i];

		// 調べ先がオープン可能かどうか
		if (!isEnableMapPosition(next, _nodes)) continue;

		// オープン予定の座標がゴールだった
		if (G == _nodes[next.y][next.x].status) {

			// ゴールを保存して
			(*_route).push_back(&_nodes[next.y][next.x]);

			// ゴール一歩手前から自分の親ノードを遡って記録
			// この記録が最短経路となる
			Node *p = _now;
			while (nullptr != p) {
				(*_route).push_back(p);
				p = p->parent;
			}

			open_node_list.clear();

			// ゴールが見つかったので true
			return true;
		}

		// ４方向のノードに対するオープンとスコア計算処理
		_nodes[next.y][next.x].status = OPEN;
		_nodes[next.y][next.x].parent = _now;
		_nodes[next.y][next.x].cost_real = _now->cost_real + 1;
		_nodes[next.y][next.x].score = _nodes[next.y][next.x].cost_real + _nodes[next.y][next.x].cost_guess;

		open_node_list.push_back(next);
	}

	// 周りのオープンが終わったので自分はクローズ
	if (S != _now->status) {
		_now->status = CLOSED;
	}

	// 現在オープンしているノードで一番スコアの小さいものが基準ノード
	Node *node = getSmallScoreNodeFromOpenNodes(_nodes);

	// ノードが見つからなければ到達不能
	if (nullptr == node) return false;

	// 再帰的に調べていく
	return aster(_nodes, node, _route);

};




int main(int argc, const char * argv[]) {

	Point g;
	Point s;

	// スタートとゴールの位置を取得
	for (int i = 0; i < MH; ++i) {
		for (int k = 0; k < MW; ++k) {
			if (G == map[i][k]) g = Point(k, i);
			if (S == map[i][k]) s = Point(k, i);
		}
	}

	// ノードデータの初期設定
	for (int i = 0; i < MH; ++i) {
		for (int k = 0; k < MW; ++k) {
			nodes[i][k].pos = Point(k, i);
			nodes[i][k].status = map[i][k];
			nodes[i][k].cost_guess = abs((g.x + g.y) - (i + k));
		}
	}

	// 二次元配列のアドレスを引数に渡す為の準備
	Node *tmp_nodes[MH];
	for (int i = 0; i < MH; ++i) {
		tmp_nodes[i] = nodes[i];
	}

	// 経路探索実行
	std::vector<Node*> route;
	bool is_success = aster(tmp_nodes, &nodes[s.y][s.x], &route);

	// false が帰ってきたら到達不能
	if (!is_success) {
		printf("到達不能\n");
		return 0;
	}

	// 探索結果の表示
	for (int i = 0; i < MH; ++i) {
		for (int k = 0; k < MW; ++k) {
			if (S == nodes[i][k].status) printf("S");
			else if (G == nodes[i][k].status) printf("G");
			else if (B == nodes[i][k].status) printf("@");
			else {
				bool f = false;
				for (auto node : route) {
					if (node->pos == Point(k, i)) {
						f = true;
						printf("*");
						break;
					}
				}
				if (!f) {
					printf("-");
				}
			}
		}
		printf("\n");
	}


	return 0;
}


