#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <queue>
#include <cmath>
using namespace std;

const double inf = 1e10, eps = 1e-6;
const double eps2 = eps*eps;

class Node {
public:
	double x,y,z;

	void read(stringstream &fin) {
		fin >> x >> y >> z;
	}
	Node(double x0=0, double y0=0, double z0=0) {
		x=x0,y=y0,z=z0;
	}
	double &operator[](int ind) { return ind==0?x:(ind==1?y:z); }
	const double &operator[](int ind) const { return ind==0?x:(ind==1?y:z); }
};

Node operator-(const Node &a, const Node &b) { return Node(a.x-b.x,a.y-b.y,a.z-b.z); }
Node operator+(const Node &a, const Node &b) { return Node(a.x+b.x,a.y+b.y,a.z+b.z); }
double dot(const Node &a, const Node &b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
double norm2(const Node &a) { return dot(a, a); }
Node operator*(const Node &a, double b) { return Node(a.x*b,a.y*b,a.z*b); }
Node operator/(const Node &a, double b) { return Node(a.x/b,a.y/b,a.z/b); }
Node cross(const Node &a, const Node &b) {
	return Node(a.y*b.z-b.y*a.z,b.x*a.z-a.x*b.z,a.x*b.y-b.x*a.y);
}
Node normalize(const Node &a) {
	return a / sqrt(norm2(a));
}
bool operator<(const Node &a, const Node &b) {
	for (int i = 0; i < 3; ++i) {
		if (a[i] < b[i] - 1e-8) return true;
		if (a[i] > b[i] + 1e-8) return false;
	}
	return false;
}

ostream &operator<<(ostream &out, const Node &a){out <<"["<<a.x<<","<<a.y<<","<<a.z<<"]";return out;}

class Mesh;

class Face {
public:
	Mesh *root;
	int ind[3], uv[3];
	Node normal;

	void read(stringstream &fin) {
		char ch;
		string s;
		int k;
		for (int i = 0; i < 3; ++i) {
			fin >> s;
			stringstream sin(s);
			sin >> k;
			ind[i] = k-1;
			sin >> ch >> k;
			uv[i] = k-1;
		}
	}
	Face(Mesh *r=NULL) {root=r;}
	Node *n(int x);
};

class Mesh {
public:
	vector<Node*> nodes;
	map<Node*, Node*> normal;
	map<pair<Node*,Node*>,Node*> enorm;
	map<Node*, vector<Face*> > adjf;
	vector<Face*> faces;
	vector<double> u, v;

	void read(ifstream &fin) {
		nodes.clear();
		normal.clear();
		enorm.clear();
		adjf.clear();
		faces.clear();
		u.clear();
		v.clear();

		string s;
		while (getline(fin, s)) {
			if (s[0] == 'v' && s[1] == ' ') {
				Node *v = new Node();
				stringstream sin(s.substr(1));
				v->read(sin);
				nodes.push_back(v);
			} else if (s[0] == 'v' && s[1] == 't') {
				stringstream sin(s.substr(2));
				double x, y;
				sin >> x >> y;
				u.push_back(x);
				v.push_back(y);
			} else if (s[0] == 'f') {
				Face *f = new Face(this);
				stringstream sin(s.substr(1));
				f->read(sin);
				faces.push_back(f);
			}
		}
		for (auto *face : faces) {
			face->normal = normalize(cross(*face->n(1)-*face->n(0), *face->n(2)-*face->n(0)));
			for (int i = 0; i < 3; ++i) {
				adjf[face->n(i)].push_back(face);
				enorm[{face->n(i),face->n((i+1)%3)}] = &face->normal;
			}
		}

        for (auto *face : faces) {
            for (int i = 0; i < 3; i++) {
                pair<Node*, Node*> pair = { face->n((i + 1) % 3), face->n(i) };
                if (enorm.find(pair) == enorm.end())
                    enorm[pair] = &face->normal;
            }
        }

		for (auto *node : nodes) {
			Node *norm = new Node();
			for (auto *face : adjf[node])
				*norm = *norm + face->normal;
			*norm = normalize(*norm);
			normal[node] = norm;
		}
	}

	// Face make_face(int ind, int uv, int new0, int new1) {
	// 	Face ans(this);
	// 	ans.ind[0]=ind;ans.ind[1]=nodes.size()+new0;ans.ind[2]=nodes.size()+new1;
	// 	ans.uv[0]=uv;ans.uv[1]=u.size()+new0;ans.uv[2]=u.size()+new1;
	// 	return ans;
	// }

	// void split(int p) {
	// 	for (int i = 0; i < 3; ++i) {
	// 		nodes.push_back((*faces[p].n(i)+*faces[p].n((i+1)%3))*0.5);
	// 		u.push_back((u[faces[p].uv[i]]+u[faces[p].uv[(i+1)%3]])*0.5);
	// 		v.push_back((v[faces[p].uv[i]]+v[faces[p].uv[(i+1)%3]])*0.5);
	// 	}
	// 	Node *n3 = &nodes[nodes.size()-2], *n4 = &nodes[nodes.size()-1], *n5 = &nodes[nodes.size()-3];
	// 	faces.push_back(make_face(faces[p].ind[0], faces[p].uv[0], -3, -1));
	// 	faces.push_back(make_face(faces[p].ind[1], faces[p].uv[1], -2, -3));
	// 	faces.push_back(make_face(faces[p].ind[2], faces[p].uv[2], -1, -2));
	// 	faces[p].ind[0]=nodes.size()-2;faces[p].ind[1]=nodes.size()-1;faces[p].ind[2]=nodes.size()-3;
	// 	faces[p].uv[0]=u.size()-2;faces[p].uv[1]=u.size()-1;faces[p].uv[2]=u.size()-3;
	// }

	// void split() {
	// 	int n = faces.size();
	// 	for (int i = 0; i < n; ++i)
	// 		split(i);
	// }
};
Node *Face::n(int x) { return root->nodes[ind[x]]; }

class BBox {
public:
	double h[3], t[3];

	BBox() {
		for (int i = 0; i < 3; ++i) {
			h[i] = inf;
			t[i] = -inf;
		}
	}
	BBox(double h0[3], double t0[3]) {
		for (int i = 0; i < 3; ++i) {
			h[i] = h0[i];
			t[i] = t0[i];
		}
	}
	BBox(const Node &node) {
		for (int i = 0; i < 3; ++i) {
			h[i] = node[i];
			t[i] = node[i];
		}
	}
};
BBox operator+(const BBox &a0, const BBox &b) {
	BBox a = a0;
	for (int i = 0; i < 3; ++i) {
		if (a.h[i] > b.h[i]) a.h[i] = b.h[i];
		if (a.t[i] < b.t[i]) a.t[i] = b.t[i];
	}
	return a;
}
BBox &operator+=(BBox &a, const BBox &b) {
	a = a + b;
	return a;
}

template <int dim>
bool cmp(Node *a, Node *b) {
	return (*a)[dim] < (*b)[dim] - eps;
}

class KDTree {
public:
	BBox box;
	Node *node;
	KDTree *s[2];

	KDTree(){
		node = NULL;
		s[0] = s[1] = NULL;
	}
	KDTree(vector<Node*> nodes, int dim = 0) {
		node = NULL;
		s[0] = s[1] = NULL;
		if (nodes.size() == 1) {
			node = nodes[0];
			box += BBox(*node);
			return;
		}
		for (Node *node : nodes)
			box += BBox(*node);
		switch (dim) {
			case 0:sort(nodes.begin(), nodes.end(), cmp<0>);break;
			case 1:sort(nodes.begin(), nodes.end(), cmp<1>);break;
			case 2:sort(nodes.begin(), nodes.end(), cmp<2>);break;
		}
		int mid = nodes.size()/2;
		vector<Node*> left(nodes.begin(), nodes.begin()+mid), right(nodes.begin()+mid, nodes.end());
		int newdim = (dim+1)%3;
		s[0] = new KDTree(left, newdim);
		s[1] = new KDTree(right, newdim);
	}
};

vector<Node*> find_closest(KDTree *tree, Node *p, double &mini) {
	Node dis;
	for (int i = 0; i < 3; ++i) {
		if ((*p)[i] < tree->box.h[i])
			dis[i] = tree->box.h[i] - (*p)[i];
		else if ((*p)[i] < tree->box.t[i])
			dis[i] = 0;
		else dis[i] = (*p)[i] - tree->box.t[i];
	}
	if (sqrt(norm2(dis)) > mini + eps)
		return {};
	if (tree->node != NULL) {
		double tmp = sqrt(norm2(*tree->node - *p));
		if (tmp < mini)
			mini = tmp;
		return {tree->node};
	}
	double ori = mini;
	auto n0 = find_closest(tree->s[0], p, mini);
	double ori0 = mini;
	auto n1 = find_closest(tree->s[1], p, mini);
	auto n2 = n1;
	n2.clear();
	for (auto q : n0)
		if (sqrt(norm2(*p-*q)) < mini + eps)
			n2.push_back(q);
	for (auto q : n1)
		if (sqrt(norm2(*p-*q)) < mini + eps)
			n2.push_back(q);
	return n2;
}

bool found(vector<Node*> &vec, Node *p) {
	return find(vec.begin(), vec.end(), p) != vec.end();
}
