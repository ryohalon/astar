// aster.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <list>

// 2�����}�b�v�̕��ƍ���
const int MW = 5;
const int MH = 5;

// �}�b�v�X�e�[�^�X
enum {
	OPEN,
	CLOSED,
	B,      // ��
	N,      // ���
	S,      // �X�^�[�g
	G       // �S�[��
};

// XY���ЂƂ܂Ƃ߂Ɉ�������
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

// �o�H�T���p�m�[�h
class Node {
public:
	Node() :
		status(N),
		cost_real(0),
		cost_guess(0),
		score(0),
		parent(nullptr)
	{}

	Point pos;         // 2�����z���̍��W
	int status;        // OPEN ��� CLOSED ���
	int cost_real;     // ���R�X�g
	int cost_guess;    // ����R�X�g
	int score;         // �X�R�A
	Node* parent;      // �e�m�[�h( �Ō�ɃS�[������H�邽�� )

	bool operator < (const Node& node) const {
		return score > node.score;
	}

};


// �}�b�v�f�[�^
// �Q�[���̃}�b�v���
int map[MH][MW] = {
	{ N, N, N, N, N },
	{ N, N, B, N, N },
	{ N, N, B, N, N },
	{ N, B, N, B, N },
	{ N, N, S, B, G }
};

// �}�b�v�̃m�[�h�f�[�^
// ���2�����z�񂩂炱��
// �m�[�h�����쐬���Čo�H�T���ɓn��
Node nodes[MH][MW];

std::list<Point> open_node_list;


// �w����W���L����( OPEN �\�� )�}�b�v�ʒu���ǂ�������
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


// [ ���݃I�[�v���ɂȂ��Ă���m�[�h�ň�ԃX�R�A�̏��������̂��擾 ]
// �T���v���̕�����Ղ��D��Ȃ̂őS�m�[�h�𒲂ׂ�`�ɂ��Ă��邪�A
// �����͗ǂ��Ȃ��̂Ń��C�u���������鎞�ɂł� ���ɃI�[�v�����Ă���
// �m�[�h�������ׂ�悤�Ɍ�������}���������悢
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


// �o�H�T�� A*
bool aster(Node **_nodes, Node *_now, std::vector<Node*> *_route) {

	// �X�^�[�g�n�_�̃X�R�A�v�Z
	if (S == _now->status) {
		_now->score = _now->cost_real + _now->cost_guess;
	}

	// �S�����̍��W
	Point dir[8] = { Point(0, 1), Point(1, 0), Point(0, -1), Point(-1, 0), Point(1, -1), Point(-1, -1), Point(-1,1), Point(1, 1) };

	// ����S�����𒲂ׂĉ\�Ȃ�I�[�v��
	for (int i = 0; i < 8; ++i) {
		Point next = _now->pos + dir[i];

		// ���א悪�I�[�v���\���ǂ���
		if (!isEnableMapPosition(next, _nodes)) continue;

		// �I�[�v���\��̍��W���S�[��������
		if (G == _nodes[next.y][next.x].status) {

			// �S�[����ۑ�����
			(*_route).push_back(&_nodes[next.y][next.x]);

			// �S�[�������O���玩���̐e�m�[�h��k���ċL�^
			// ���̋L�^���ŒZ�o�H�ƂȂ�
			Node *p = _now;
			while (nullptr != p) {
				(*_route).push_back(p);
				p = p->parent;
			}

			open_node_list.clear();

			// �S�[�������������̂� true
			return true;
		}

		// �S�����̃m�[�h�ɑ΂���I�[�v���ƃX�R�A�v�Z����
		_nodes[next.y][next.x].status = OPEN;
		_nodes[next.y][next.x].parent = _now;
		_nodes[next.y][next.x].cost_real = _now->cost_real + 1;
		_nodes[next.y][next.x].score = _nodes[next.y][next.x].cost_real + _nodes[next.y][next.x].cost_guess;

		open_node_list.push_back(next);
	}

	// ����̃I�[�v�����I������̂Ŏ����̓N���[�Y
	if (S != _now->status) {
		_now->status = CLOSED;
	}

	// ���݃I�[�v�����Ă���m�[�h�ň�ԃX�R�A�̏��������̂���m�[�h
	Node *node = getSmallScoreNodeFromOpenNodes(_nodes);

	// �m�[�h��������Ȃ���Γ��B�s�\
	if (nullptr == node) return false;

	// �ċA�I�ɒ��ׂĂ���
	return aster(_nodes, node, _route);

};




int main(int argc, const char * argv[]) {

	Point g;
	Point s;

	// �X�^�[�g�ƃS�[���̈ʒu���擾
	for (int i = 0; i < MH; ++i) {
		for (int k = 0; k < MW; ++k) {
			if (G == map[i][k]) g = Point(k, i);
			if (S == map[i][k]) s = Point(k, i);
		}
	}

	// �m�[�h�f�[�^�̏����ݒ�
	for (int i = 0; i < MH; ++i) {
		for (int k = 0; k < MW; ++k) {
			nodes[i][k].pos = Point(k, i);
			nodes[i][k].status = map[i][k];
			nodes[i][k].cost_guess = abs((g.x + g.y) - (i + k));
		}
	}

	// �񎟌��z��̃A�h���X�������ɓn���ׂ̏���
	Node *tmp_nodes[MH];
	for (int i = 0; i < MH; ++i) {
		tmp_nodes[i] = nodes[i];
	}

	// �o�H�T�����s
	std::vector<Node*> route;
	bool is_success = aster(tmp_nodes, &nodes[s.y][s.x], &route);

	// false ���A���Ă����瓞�B�s�\
	if (!is_success) {
		printf("���B�s�\\n");
		return 0;
	}

	// �T�����ʂ̕\��
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


