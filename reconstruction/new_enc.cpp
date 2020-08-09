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
#include "util.hpp"
#include <opencv2/opencv.hpp>
using namespace std;

Mesh cloth, body, cloth1;

queue<vector<Node*> > q;
map<Node*, Node*> match;
map<Node*, vector<Node*> > closest;
map<Node*, Node*> realNode;
map<Node*, vector<vector<Node*> > > smallFaces;
KDTree root;
set<set<Node> > visited;

void update_match(Node *body, Node *cloth) {
	if (match.find(body) == match.end()) {
		match[body] = cloth;
		return;
	}
	double dis0 = norm2(*body-*match[body]), dis1 = norm2(*body-*cloth);
	if (dis0 < dis1) //farthest point of closest match
		match[body] = cloth;
}

vector<Node*> get_closest(KDTree &root, Node *p) {
	if (closest.find(p) != closest.end())
		return closest[p];
	double mini = inf;
	auto n1 = find_closest(&root, p, mini);
	for (Node *n : n1)
		update_match(n, p);
	closest[p] = n1;
	return n1;
}

map<Node, int> newind;
const int maxn = 256;
Node ans[maxn][maxn];
Node dpMap[maxn][maxn];
bool legal[maxn][maxn];

pair<double, double> get_barycentric(vector<Node> uvs, Node target, Node normal) {
	//(1-wp-wq)*uvs0+wp*uvs1+wq*uvs2=target
	// double A[3][2], b[3];
	// for (int i = 0; i < 3; ++i) {
	// 	A[i][0] = uvs[1][i]-uvs[0][i];
	// 	A[i][1] = uvs[2][i]-uvs[0][i];
	// 	b[i] = target[i]-uvs[0][i];
	// }
	// int s0 = fabs(A[0][0])>fabs(A[1][0]);
	// int s1 = fabs(A[0][0])>fabs(A[2][0]);
	// int s2 = fabs(A[1][0])>fabs(A[2][0]);
	// int p, q;
	// switch (s0*4+s1*2+s2) {
	// 	case 0:p=2;q=1;break;//210
	// 	case 1:p=1;q=2;break;//120
	// 	case 2:break;
	// 	case 3:p=1;q=0;break;//102
	// 	case 4:p=2;q=0;break;//201
	// 	case 5:break;
	// 	case 6:p=0;q=2;break;//021
	// 	case 7:p=0;q=1;break;//012
	// }
	// // cout << (A[p][0]*A[q][1]-A[q][0]*A[p][1]) << endl;
	// // cout << p << " " << q << endl;
	// // cout << uvs[1]-uvs[0] << " " << uvs[2]-uvs[0] << " " << target-uvs[0] << endl;
	// return {(b[p]*A[q][1]-b[q]*A[p][1])/(A[p][0]*A[q][1]-A[q][0]*A[p][1]),
	// 		(b[p]*A[q][0]-b[q]*A[p][0])/(A[p][1]*A[q][0]-A[q][1]*A[p][0])};
	Node a0 = uvs[0]-target, a1 = uvs[1]-target, a2 = uvs[2]-target;
	double w0 = dot(cross(a1, a2), normal);
	double w1 = dot(cross(a2, a0), normal);
	double w2 = dot(cross(a0, a1), normal);
	return {w1/(w0+w1+w2), w2/(w0+w1+w2)};
}

Node get_dist(Node p, Node dir, vector<Node*> &face, bool verbose=false) {
	Node normal = cross(*face[1]-*face[0], *face[2]-*face[0]);
	// if (norm2(normal) < eps*eps)
	// 	return Node(-inf,0,0);
	// cout << p << " " << dir << endl;
	// cout << *face[0] << " " << *face[1] << " " << *face[2] << endl;
	normal = normalize(normal);
	double cur = dot(*face[0]-p, normal), speed = dot(dir, normal);
	// if (fabs(speed) < eps)
	// 	return Node(-inf,0,0);
	double time = cur / speed;
	// cout << time << endl;
	// if (time < 0)
	// 	return Node(-inf,0,0);
	Node point = p + dir * time;
	if (verbose)
		cout << normal << point << endl;
	pair<double, double> w = get_barycentric({*face[0],*face[1],*face[2]}, point, normal);
	double w0 = w.first, w1 = w.second;
	if (verbose) {
		cout << w0 << " " << w1 << endl;
		cout << *face[1]-*face[0] << " " << *face[2]-*face[0] << endl;
		// cout << norm2(point-*face[0]) << endl;
		// cout << norm2(point-*face[1]) << endl;
		// cout << norm2(point-*face[2]) << endl;
	}
	if (w0 < -eps || w1 < -eps || w0+w1 > 1+eps) {
		// if (norm2(point-*face[0])>eps*eps && norm2(point-*face[1])>eps*eps && norm2(point-*face[2])>eps*eps)
			return Node(-inf,0,0);
	}
	Node *n0 = realNode[face[0]], *n1 = realNode[face[1]], *n2 = realNode[face[2]];
	// Node *n0 = face[0], *n1 = face[1], *n2 = face[2];
	return *n0*(1-w0-w1) + *n1*w0 + *n2*w1;
}

Node get_dist(Node p, Node dir, vector<vector<Node*> > &faces, bool verbose=false) {
	static int previous = 0;
	if (previous < (int)faces.size()) {
		Node tmp = get_dist(p, dir, faces[previous],verbose);
		if (tmp[0] > -inf)
			return tmp;
	}
	for (previous = 0; previous < (int)faces.size(); ++previous) {
		Node tmp = get_dist(p, dir, faces[previous],verbose);
		if (tmp[0] > -inf)
			return tmp;
	}
	return Node(-inf,0,0);
}

Node rotate(Node ori, Node target, double weight) {
	double theta = acos(dot(ori, target)/sqrt(norm2(ori)*norm2(target)));
	theta *= weight;
	Node dir = normalize(cross(ori, target));
	double s = cos(theta/2);
	Node v = dir*sin(theta/2);
	return ori*(s*s-dot(v,v)) + v*2*dot(v,ori) + cross(v,ori)*s*2;
	// return ori*(1-weight)+target*weight;
}

void do_mapping(vector<Node*> nodes, vector<Node> uvs, 
				vector<Node> oriuvs,
				vector<vector<Node*> > &faces, Face *bface,
				int side, Node* edgenode) {
	double scale = maxn;
	BBox uvbox = BBox(uvs[0]*scale) + BBox(uvs[1]*scale) + BBox(uvs[2]*scale);
	int st_u = uvbox.h[0];
	int ed_u = uvbox.t[0]+1;
	int st_v = uvbox.h[1];
	int ed_v = uvbox.t[1]+1;
	if (st_u < 0)
	{
		printf("st_u = %d\n", st_u);
		st_u = 0;
	}
	if (st_v < 0)
	{
		printf("st_v = %d\n", st_v);
		st_v = 0;
	}
	if (ed_u >= maxn)
	{
		printf("ed_u = %d\n", ed_u);
		ed_u = maxn - 1;
	}
	if (ed_v >= maxn)
	{
		printf("ed_v = %d\n", ed_v);
		ed_v = maxn - 1;
	}

	for (int u = st_u; u <= ed_u; ++u)
		for (int v = st_v; v <= ed_v; ++v) {
			if (legal[u][v]) continue;
			{
				pair<double, double> w = get_barycentric(oriuvs, Node(u/scale, v/scale, 0), Node(0,0,1));
				double wp = w.first, wq = w.second;
				if (wp < -eps || wq < -eps || wp+wq > 1+eps)
					continue;
			}
			pair<double, double> w = get_barycentric(uvs, Node(u/scale, v/scale, 0), Node(0,0,1));
			double wp = w.first, wq = w.second;
			if (wp < -eps || wq < -eps || wp+wq > 1+eps)
				continue;
			bool verbose = false;//(u==1261&&v==1235);
			if (verbose)
				cout << endl <<wp << " " << wq << endl;
			Node dp = *nodes[0]*(1-wp-wq) + *nodes[1]*wp + *nodes[2]*wq;
			Node dir = rotate(*body.normal[nodes[0]], bface->normal, wp+wq);
			Node dist = get_dist(dp, dir, faces, verbose);
			if (dist[0] > -inf) {
				// cout << u << " " << v << endl;
				legal[u][v] = true;
				ans[u][v] = dist-dp;
                dpMap[u][v] = dp;
			}
		}
	set<pair<int,int>> visited;
	for (int i = 0; i < 1; ++i) {
		int len = 10;
		for (int j = 0; j <= len; ++j) {
			Node cur = (*nodes[i]*(len-j)+*nodes[side]*j)/len;
			pair<double, double> w = get_barycentric({*nodes[0],*nodes[1],*nodes[2]}, cur, Node(0,0,1));
			double wp = w.first, wq = w.second;
			int u = (int)round((uvs[0][0]*(1-wp-wq) + uvs[1][0]*wp + uvs[2][0]*wq)*scale);
			int v = (int)round((uvs[0][1]*(1-wp-wq) + uvs[1][1]*wp + uvs[2][1]*wq)*scale);
			if (visited.find({u,v}) != visited.end()) continue;
			Node dp = cur;
			Node dir = rotate(*body.normal[nodes[0]], (*body.enorm[{nodes[0], edgenode}]+*body.enorm[{edgenode, nodes[0]}])/2, wp+wq);
			// Node dir = rotate(*body.normal[nodes[0]], bface->normal, wp+wq);
			Node dist = get_dist(dp, dir, faces);
			if (dist[0] > -inf) {
				legal[u][v] = true;
                ans[u][v] = dist - dp;
                dpMap[u][v] = dp;
				visited.insert({u,v});
			}
		}
	}
	// cout << endl;
}

void fit_mapping(Node *bodynode, Face *bface, vector<vector<Node*> > &faces) {
	Node p, q;
	// cout << *bodynode << endl;
	int i;
	for (i = 0; i < 3; ++i)
		if (bface->n(i) == bodynode) {
			p = *bface->n((i+1)%3)-*bface->n(i);
			q = *bface->n((i+2)%3)-*bface->n(i);
			break;
		}
	if (i == 3) {
		cout << "???" << endl;
		exit(0);
	}
	double xx = dot(p, p), yy = dot(q, q), xy = dot(p, q);
	double wp = yy*(xy-xx)/2/(xy*xy-xx*yy), wq = xx*(xy-yy)/2/(xy*xy-xx*yy);
	Node uv0 = Node(body.u[bface->uv[i]], body.v[bface->uv[i]], 0);
	Node uv1 = Node(body.u[bface->uv[(i+1)%3]], body.v[bface->uv[(i+1)%3]], 0);
	Node uv2 = Node(body.u[bface->uv[(i+2)%3]], body.v[bface->uv[(i+2)%3]], 0);
	Node uvp = uv1 - uv0;
	Node uvq = uv2 - uv0;
	// cout << uvp << " " << uvq << endl;
	Node side0 = *bodynode + p*0.5, uvside0 = uv0 + uvp*0.5;
	Node center = *bodynode + p*wp + q*wq, uvcen = uv0 + uvp*wp + uvq*wq;
	do_mapping({bodynode, &side0, &center}, {uv0, uvside0, uvcen}, {uv0,uv1,uv2}, faces, bface, 1, bface->n((i+1)%3));
	Node side1 = *bodynode + q*0.5, uvside1 = uv0 + uvq*0.5;
	do_mapping({bodynode, &center, &side1}, {uv0, uvcen, uvside1}, {uv0,uv1,uv2}, faces, bface, 2, bface->n((i+2)%3));
}

inline bool is0(double a)
{
	double EPS = 1e-6;
	return a<EPS && a>-EPS;
}

bool isZeroPoint(const Node& nd)
{
	if (is0(nd.x) && is0(nd.y) && is0(nd.z)) return true;
	return false;
}

inline double sqr(double a)
{
	return a*a;
}

double dist(const Node& a, const Node& b)
{
	return sqrt(sqr(a.x-b.x) + sqr(a.y-b.y) + sqr(a.z-b.z));
}

void print_face(ostream &fout, Node &a, Node &b, Node &c) {
	//if (isZeroPoint(a) || isZeroPoint(b) || isZeroPoint(c)) {puts("zero!!"); return;}
	double edgeThresh = 0.1;
	if (dist(a, b) > edgeThresh || dist(b, c) > edgeThresh || dist(a, c) > edgeThresh) 
	{
		//puts("long edge!!"); 
		return;
	}
	fout << "f " << newind[a] << " " << newind[b] << " " << newind[c] << endl;
}

void readBinaryFileC1(const std::string& filename, bool dataArr[maxn][maxn])
{
    FILE* fin = fopen(filename.c_str(), "rb");
    cv::Mat_<float> mat32fc1(maxn, maxn);

    fread(mat32fc1.data, sizeof(float), mat32fc1.rows*mat32fc1.cols*mat32fc1.channels(), fin);

//     double minVal, maxVal;
//     cv::minMaxLoc(mat32fc1, &minVal, &maxVal);

    for (int i = 0; i < maxn; i++)
    {
        for (int j = 0; j < maxn; j++)
        {
            dataArr[i][j] = (mat32fc1[i][j] > 0.5);
        }
    }

    fclose(fin);
}

bool readBinaryFileC1(const std::string& filename, cv::Mat_<float>& mat32fc1)
{
	FILE* fin = fopen(filename.c_str(), "rb");
    if (fin == NULL) return false;
	mat32fc1 = cv::Mat_<float>(maxn, maxn);

	fread(mat32fc1.data, sizeof(float), mat32fc1.rows*mat32fc1.cols*mat32fc1.channels(), fin);

	fclose(fin);
	return true;
}

void readImageFileC1(const std::string& filename, cv::Mat_<float>& mat32fc1)
{
	cv::Mat_<uchar> matuc1 = cv::imread(filename, 0);
	mat32fc1 = cv::Mat_<float>(maxn, maxn);

	for (int i = 0; i < maxn; i++)
	{
		for (int j = 0; j < maxn; j++)
		{
			mat32fc1[i][j] = 1.0 - matuc1[i][j] / 255.0;
		}
	}
}

bool readBinaryFileC3(const std::string& filename, Node dataArr[maxn][maxn], double sx = 1)
{
    FILE* fin = fopen(filename.c_str(), "rb");
	if (fin == NULL) return false;

    cv::Mat_<cv::Point3f> mat32fc3(maxn, maxn);

    fread(mat32fc3.data, sizeof(float), mat32fc3.rows*mat32fc3.cols*mat32fc3.channels(), fin);

    double minVal, maxVal;
    cv::minMaxLoc(mat32fc3, &minVal, &maxVal);

	mat32fc3 = mat32fc3 * sx;

    for (int i = 0; i < maxn; i++)
    {
        for (int j = 0; j < maxn; j++)
        {
            dataArr[i][j].x = mat32fc3[i][j].x;
            dataArr[i][j].y = mat32fc3[i][j].y;
            dataArr[i][j].z = mat32fc3[i][j].z;
        }
    }

    fclose(fin);
	return true;
}

bool readBinaryFileC3(const std::string& filename, cv::Mat_<cv::Point3f>& mat32fc3)
{
    FILE* fin = fopen(filename.c_str(), "rb");
    if (fin == NULL) return false;
    mat32fc3 = cv::Mat_<cv::Point3f>(maxn, maxn);

    fread(mat32fc3.data, sizeof(float), mat32fc3.rows*mat32fc3.cols*mat32fc3.channels(), fin);
    fclose(fin);
    return true;
}


void encode(int idx = 0, string outpath = "", bool recover = false, string dispPath="", string dpPath="", string legalPath="", bool isInference = false) {

	cout << "reconstruct" << endl;
    char filename[256];
	if (!recover)
	{
		for (auto &it : smallFaces) {
			// if (fabs((*it.first)[0]-0.0812)>1e-4 || fabs((*it.first)[1]-0.2797)>1e-4 || fabs((*it.first)[2]-0.0248)>1e-4)
			// 	continue;
			for (auto *face : body.adjf[it.first])
				fit_mapping(it.first, face, it.second);
			// ofstream fout("output1.obj");
			// int n = 0;
			// for (int i = 0; i < it.second.size(); ++i)
			// 	for (int j = 0; j < 3; ++j) {
			// 		Node *node = it.second[i][j];
			// 		if (newind.find(*node) == newind.end()) {
			// 			newind[*node] = n+++1;
			// 			fout << "v " << node->x << " " << node->y << " " << node->z << endl;
			// 		}
			// 	}
			// for (int i = 0; i < it.second.size(); ++i) {
			// 	auto &vec = it.second[i];
			// 	fout << "f " << newind[*vec[0]] << " " << newind[*vec[1]] << " " << newind[*vec[2]] << endl;
			// }
			// fout.close();
			// break;
		}
	}
	else
	{
		if (isInference)
		{
    		sprintf(filename, "/legal_map_%06d_displacement_map.dat", idx);
		}
		else
		{
    		sprintf(filename, "/displacement_map_%06d.dat", idx);
		}
		
		if (!readBinaryFileC3(dispPath + filename, ans)) 
		{
			printf("read file failed: %s\n", (dispPath + filename).c_str());
			return;
		}
        //readBinaryFileC3("DispMap.dat", ans);
        sprintf(filename, "/dp_map_%06d.dat", idx);
		readBinaryFileC3(dpPath + filename, dpMap);

    	sprintf(filename, "/legal_map_%06d.dat", idx);
		readBinaryFileC1(legalPath + filename, legal);
	}

	if (recover)
	{
        int n = 0;
        sprintf(filename, "/reconstruct_mesh/mesh_%06d.obj", idx);
        //sprintf(filename, "/reconstruct_mesh/mesh_tmp.obj");
		ofstream fout(outpath + filename);
		std::cout << outpath + filename << std::endl;
		
		for (int i = 0; i < maxn; ++i)
			for (int j = 0; j < maxn; ++j)
				if (legal[i][j]) {
					Node node = ans[i][j] + dpMap[i][j];
					if (newind.find(node) == newind.end()) {
						newind[node] = n+++1;
						fout << "v " << node[0] << " " << node[1] << " " << node[2] << endl;
					}
				}

		for (int i = 0; i < maxn-1; ++i)
			for (int j = 0; j < maxn-1; ++j) {
				vector<Node> a = {
					ans[i][j] + dpMap[i][j], 
					ans[i][j+1] + dpMap[i][j+1], 
					ans[i+1][j] + dpMap[i+1][j], 
					ans[i+1][j+1] + dpMap[i+1][j+1] };
				int state = (((legal[i][j])*2+(legal[i][j+1]>0))*2+(legal[i+1][j]>0))*2+(legal[i+1][j+1]>0);
				switch (state) {
					case 7:print_face(fout, a[1], a[2], a[3]);break;
					case 11:print_face(fout, a[0], a[2], a[3]);break;
					case 13:print_face(fout, a[0], a[3], a[1]);break;
					case 14:print_face(fout, a[0], a[2], a[1]);break;
					case 15:print_face(fout, a[0], a[2], a[3]);print_face(fout, a[0], a[3], a[1]);break;
					default:break;
				}
			}
		// ofstream fout("output1.obj");
		// for (auto &it : smallFaces) {
		// 	for (int i = 0; i < it.second.size(); ++i)
		// 		for (int j = 0; j < 3; ++j) {
		// 			Node *node = it.second[i][j];
		// 			if (newind.find(*node) == newind.end()) {
		// 				newind[*node] = n+++1;
		// 				fout << "v " << node->x << " " << node->y << " " << node->z << endl;
		// 			}
		// 		}
		// }
		// for (auto &it : smallFaces) {
		// 	for (int i = 0; i < it.second.size(); ++i) {
		// 		auto &vec = it.second[i];
		// 		fout << "f " << newind[*vec[0]] << " " << newind[*vec[1]] << " " << newind[*vec[2]] << endl;
		// 	}
		// }

		// for (int i = 0; i < body.nodes.size(); ++i) {
		// 	auto node = body.nodes[i];
		// 	if (match.find(node) == match.end())
		// 		continue;
		// 	auto pos = node;//match[node];//
		// 	if (match.find(node) != match.end())
		// 		pos = match[node];
		// 	else
		// 		match[node] = pos;
		// 	if (newind.find(*pos) == newind.end()) {
		// 		newind[*pos] = n+++1;
		// 		// fout << "v " << pos->x << " " << pos->y << " " << pos->z << endl;
		// 		fout << "v " << realNode[pos]->x << " " << realNode[pos]->y << " " << realNode[pos]->z << endl;
		// 	}
		// }
		// for (int i = 0; i < body.u.size(); ++i)
		// 	fout << "vt " << body.u[i] << " " << body.v[i] << endl;
		// for (auto face : body.faces) {
		// 	bool complete = true;
		// 	for (int i = 0; i < 3; ++i)
		// 		if (match.find(face->n(i)) == match.end())
		// 			complete = false;
		// 	if (!complete)
		// 		continue;
		// 	// for (int i = 0; i < 3; ++i)
		// 	// 	cnt[{face.n[i], face.n[(i+1)%3]}]++;
		// 	fout << "f ";
		// 	for (int i = 0; i < 3; ++i)
		// 		fout << newind[*match[face->n(i)]] << "/" << face->uv[i]+1 << " ";
		// 	fout << endl;
		// }

		fout.close();
	}
}

Node* get_cut(Node *n0, Node *n1, Node *b0, Node *b1) {
	//get the intesection of (n0-n1) and (b0|b1)
	if (b0 == b1) return n0;
	Node normal = normalize(*b0-*b1);
	double dist = (dot(*b0, normal) + dot(*b1, normal)) / 2;
	double d0 = dot(*n0, normal), d1 = dot(*n1, normal);
	// if (fabs(d0-d1) < eps)
	// 	return n1;
	double w0 = (d1-dist) / (d1-d0), w1 = (dist-d0) / (d1-d0);
	// if (w0 < 0 || w0 > 1)
	// 	cout << w0<<" "<<w1 << endl;
	Node *newnode = new Node();
	*newnode = *n0*w0 + *n1*w1;
	Node *realnode = new Node();
	*realnode = *realNode[n0]*w0 + *realNode[n1]*w1;
	realNode[newnode] = realnode;
	return newnode;
}

void dopush(const vector<Node*> &f) {
	q.push(f);
}

bool do_001(Node *n0, Node *n1, Node *n2, Node *newnode0, Node *newnode1, Node *newnode2) {
	//001
		// cout << "001" << endl;
	// if (norm2(*newnode0-*n2) < eps2) //goto 000
	// 	return false;
	// bool flag = norm2(*newnode0-*n0) < eps2;
	// if (norm2(*newnode1-*n2) < eps2) //goto 000
	// 	return false;
	// if (flag) {
	// 	if (norm2(*newnode1-*n1) < eps2) //goto 111->000
	// 		return false;
	// 	else {
	// 		dopush({n0, n1, newnode1});
	// 		dopush({newnode1, n2, n0});
	// 	}
	// } else {
		dopush({newnode0, newnode1, n2});
		dopush({n0, n1, newnode0});
		// if (norm2(*newnode1-*n1) > eps2)
			dopush({newnode0, n1, newnode1});
	// }
	return true;
}

void do_cut(vector<Node*> &body0, vector<Node*> &body1, vector<Node*> &body2,
			Node *n0, Node *n1, Node *n2) {
	// 000
	for (auto b0 : body0)
		for (auto b1 : body1)
			for (auto b2 : body2)
				if (b0 == b1 && b1 == b2) {
					// cout << "000" << endl;
					smallFaces[b0].push_back({n0, n1, n2});
					return;
				}
	for (auto b0 : body0)
		for (auto b1 : body1)
			for (auto b2 : body2) {
				Node *newnode0 = get_cut(n0, n2, b0, b2);
				Node *newnode1 = get_cut(n1, n2, b1, b2);
				Node *newnode2 = get_cut(n0, n1, b0, b1);
				if (b0 == b1) {
					if (do_001(n0,n1,n2,newnode0,newnode1,newnode2))
						return;
				} else if (b0 == b2) { //010, symmetric to 001
					if (do_001(n2,n0,n1,newnode1,newnode2,newnode0))
						return;
				} else if (b1 == b2) { //011, symmetric to 001
					if (do_001(n1,n2,n0,newnode2,newnode0,newnode1))
						return;
				}
			}
	for (auto b0 : body0)
		for (auto b1 : body1)
			for (auto b2 : body2) {
				//012
				Node *newnode0 = get_cut(n0, n2, b0, b2);
				Node *newnode1 = get_cut(n1, n2, b1, b2);
				Node *newnode2 = get_cut(n0, n1, b0, b1);
				dopush({n0, newnode2, newnode0});
				dopush({n1, newnode1, newnode2});
				dopush({n2, newnode0, newnode1});
				dopush({newnode0, newnode2, newnode1});
				return;
			}
}

void solve() {
	root = KDTree(body.nodes);
	for (Face *face : cloth.faces) {
		dopush({face->n(0), face->n(1), face->n(2)});
	}
	while (!q.empty()) {
		if (q.size() % 10000 == 0) {
			//cout << q.size() << endl;
			//return;
		}
		vector<Node*> p = q.front();
		q.pop();
		Node *n0 = p[0];
		Node *n1 = p[1];
		Node *n2 = p[2];
		set<Node> state;
		for (int i = 0; i < 3; ++i)
			state.insert(*p[i]);
		if (visited.find(state) != visited.end())
			continue;
		visited.insert(state);
		auto body0 = get_closest(root, n0);
		auto body1 = get_closest(root, n1);
		auto body2 = get_closest(root, n2);
		do_cut(body0, body1, body2, n0, n1, n2);
	}
}

void fullFillMatch()
{
    KDTree clothKDTree = KDTree(cloth.nodes);
    for (auto& bodyNode : body.nodes)
    {
        if (match.find(bodyNode) == match.end())
        {
            match[bodyNode] = get_closest(clothKDTree, bodyNode)[0];
        }
    }
}

void saveBinaryFile(string filename, const cv::Mat& mat32fc3)
{
	//std::cout << filename << std::endl;
    FILE* fout = fopen(filename.c_str(), "wb");
    fwrite(mat32fc3.data, sizeof(float), mat32fc3.rows*mat32fc3.cols*mat32fc3.channels(), fout);

//     for (int i = 0; i < mat32fc3.rows; i++)
//     {
//         const float* ptr_DispMap = mat32fc3.ptr<float>(i);
//         for (int j = 0; j < mat32fc3.cols; j++)
//         {
//         }
//     }

    fclose(fout);
}

void saveLegalMap(string path="", int idx = 0)
{
    cv::Mat_<float> legalMap(maxn, maxn);
    for (int i = 0; i < maxn; i++)
    {
        for (int j = 0; j < maxn; j++)
        {
            if (legal[i][j])
                legalMap[i][j] = 1;
            else
                legalMap[i][j] = 0;
        }
    }

    char filename[256];
    sprintf(filename, "/test_legal_map_image_big/legal_map_%06d.jpg", idx);
    cv::imwrite(path + filename, legalMap*255);

    //sprintf(filename, "/train_legal_map/legal_map_%06d.dat", idx);
    sprintf(filename, "/test_legal_map_big/legal_map_%06d.dat", idx);
    FILE* fout = fopen((path+ filename).c_str(), "wb");
    fwrite(legalMap.data, sizeof(float), legalMap.rows*legalMap.cols*legalMap.channels(), fout);
    fclose(fout);
}

void saveLegalMap(cv::Mat_<float>& legalMap, string path = "", int idx = 0, bool saveImg = true, bool saveBinary = true)
{
	char filename[256];
    if (saveImg)
    {
        sprintf(filename, "/legal_map_%06d.jpg", idx);
        cv::imwrite(path + filename, legalMap * -255 + 255);
        //cv::imwrite(path + filename, legalMap*255);
    }

    if (saveBinary)
    {
        sprintf(filename, "/legal_map_%06d.dat", idx);
        FILE* fout = fopen((path + filename).c_str(), "wb");
        fwrite(legalMap.data, sizeof(float), legalMap.rows*legalMap.cols*legalMap.channels(), fout);
        fclose(fout);
    }
}

void saveData(string path = "", int idx = 0, int siz = maxn)
{
    cv::Mat DispMap(siz, siz, CV_32FC3);

    for (int i = 0; i < siz; i++)
    {
        for (int j = 0; j < siz; j++)
        {
            float* ptr_DispMap = DispMap.ptr<float>(i);
            ptr_DispMap[j * 3 + 0] = (float)ans[i][j].x;
            ptr_DispMap[j * 3 + 1] = (float)ans[i][j].y;
            ptr_DispMap[j * 3 + 2] = (float)ans[i][j].z;
        }
    }

    cv::Mat DpMap(siz, siz, CV_32FC3);

    for (int i = 0; i < siz; i++)
    {
        for (int j = 0; j < siz; j++)
        {
            float* ptr_DpMap = DpMap.ptr<float>(i);
            ptr_DpMap[j * 3 + 0] = (float)dpMap[i][j].x;
            ptr_DpMap[j * 3 + 1] = (float)dpMap[i][j].y;
            ptr_DpMap[j * 3 + 2] = (float)dpMap[i][j].z;
        }
    }

//     double minVal, maxVal;
//     cv::minMaxLoc(DispMap, &minVal, &maxVal);

    //cv::imwrite(path + "displacement.jpg", DispMap*255);
    char filename[256];
    //sprintf(filename, "/train_displacement_map/displacement_map_%06d.dat", idx);
    sprintf(filename, "/test_displacement_map_big/displacement_map_%06d.dat", idx);
    saveBinaryFile(path + filename, DispMap);

    sprintf(filename, "/reconstruct_dp_map/dp_map_%06d.dat", idx);
    saveBinaryFile(path + filename, DpMap);

//     sprintf(filename, "/reconstruct_dp_map/dp_map_%06d.jpg", idx);
//     cv::imwrite(path + filename, DpMap*255);

    saveLegalMap(path, idx);
}

void saveDispMapNew(int siz = maxn)
{
// 	const float EPS = 1e-8f;
// 	cv::Mat DispMap(siz, siz, CV_32FC3);
// 	cv::Mat coeff(siz, siz, CV_32FC3);
// 
// 	for (size_t i = 0; i < body.faces.size(); i++)
// 	{
// 		cv::Point2f p[3];
// 		for (int j = 0; j < 3; j++)
// 		{
// 			int id2d = body.faces[i]->uv[j];
// 			p[j].x = body.u[id2d] * siz;
// 			p[j].y = body.v[id2d] * siz;
// 		}
// 
// 		float xmin = p[0].x;
// 		float xmax = p[0].x;
// 		float ymin = p[0].y;
// 		float ymax = p[0].y;
// 		for (int j = 1; j < 3; j++)
// 		{
// 			if (xmin > p[j].x) xmin = p[j].x;
// 			if (xmax < p[j].x) xmax = p[j].x;
// 			if (ymin > p[j].y) ymin = p[j].y;
// 			if (ymax < p[j].y) ymax = p[j].y;
// 		}
// 
// 		p[1] -= p[0];
// 		p[2] -= p[0];
// 		float a = p[1].x;
// 		float b = p[2].x;
// 		float c = p[1].y;
// 		float d = p[2].y;
// 		float k = a * d - b * c;
// 		if (k < EPS && k > -EPS) continue;
// 
// 		for (int y0 = (int)ymin; y0 < (int)(ymax + 1); y0++)
// 		{
// 			for (int x0 = (int)xmin; x0 < (int)(xmax + 1); x0++)
// 			{
// 				float c0 = (d*(x0 - p[0].x) - b * (y0 - p[0].y)) / k;
// 				float c1 = (a*(y0 - p[0].y) - c * (x0 - p[0].x)) / k;
// 				float c2 = 1 - c0 - c1;
// 				if (c0 < 0 || c1 < 0 || c2 < 0 ||
// 					c0 > 1 || c1 > 1 || c2 > 1)
// 				{
// 					continue;
// 				}
//                 cv::Point3f* ptr_coeff = coeff.ptr<cv::Point3f>(y0);
// 				ptr_coeff[x0].x = c0;
// 				ptr_coeff[x0].y = c1;
// 				ptr_coeff[x0].z = c2;
// 
// 				cv::Point3f disp[3];
// 
// 				for (int j = 0; j < 3; j++)
// 				{
// 					int id3d = body.faces[i]->ind[j];
// 					disp[j].x = match[body.nodes[id3d]]->x - body.nodes[id3d]->x;
// 					disp[j].y = match[body.nodes[id3d]]->y - body.nodes[id3d]->y;
// 					disp[j].z = match[body.nodes[id3d]]->z - body.nodes[id3d]->z;
// 				}
// 
// 				float* ptr_DispMap = DispMap.ptr<float>(y0);
// 				ptr_DispMap[x0 * 3 + 0] = c0 * disp[0].x + c1 * disp[1].x + c2 * disp[2].x;
// 				ptr_DispMap[x0 * 3 + 1] = c0 * disp[0].y + c1 * disp[1].y + c2 * disp[2].y;
// 				ptr_DispMap[x0 * 3 + 2] = c0 * disp[0].z + c1 * disp[1].z + c2 * disp[2].z;
// 			}
// 		}
//     }
//     cv::imwrite("coeff.jpg", coeff);
//     cv::imwrite("displacement.jpg", DispMap);
//     saveBinaryFile("coeff.dat", coeff);
//     saveBinaryFile("DispMap.dat", DispMap);
//     saveLegalMap();
}

bool dealWithOneCase(string objFileName, string bodyFileName, string outPath, int idx=0)
{
	match.clear();
	closest.clear();
	realNode.clear();
	smallFaces.clear();
	visited.clear();
	newind.clear();

    ifstream fin(objFileName);//"cube.obj");//"tshirt.obj");//
	if (!fin.is_open()) return false;
    cloth.read(fin);
    fin.close();
    ifstream fin1(bodyFileName);
    body.read(fin1);
    fin1.close();
    ifstream fin2(objFileName);
    cloth1.read(fin2);
    fin2.close();
    // for (int i = 0; i < 0; ++i)
    // 	body.split();
    for (int i = 0; i < (int)cloth.nodes.size(); ++i)
        realNode[cloth.nodes[i]] = cloth1.nodes[i];
    solve();

    encode(idx, outPath);
    //fullFillMatch();
    saveData(outPath, idx, maxn);

	return true;
}

void recoverObj(string dispPath="", string dpPath="", string legalPath="", string outPath = "", int stId=0, int edId=7, bool isInference = true)
{
	/*
	string path = "/home/yushen/workspace/sim_data_new/cloth105_new/";
    //string outPath = "/home/yushen/workspace/projects/pix2pixHD/datasets/cloth14";
    outPath = path;
    //string outPath = "D:/data/garment project/cloth105/";

	//debug
//     dispPath = "D:/data/garment project/cloth105/train_displacement_map/";
//     //dpPath = "D:/data/garment project/cloth105/reconstruct_dp_map/";
//     dpPath = "D:/data/garment project/cloth105/regenerated_dp_map_new/";
// 	legalPath = "D:/data/garment project/cloth105/train_legal_map/";

	if (isInference)
	{
		dispPath = "/home/yushen/workspace/projects/pix2pixHD/results/cloth105/test_latest/images/";
		edId = 7;
	}
	else
	{
		dispPath = path + "train_displacement_map/";
	}
	
	dpPath = path + "reconstruct_dp_map/";
	//dpPath = path + "regenerated_dp_map_new/";
	legalPath = path + "train_legal_map/";

	const int N_ID = 2;
	edId = N_ID - 1;
	//int idArray[N_ID] = {0, 249, 2500, 2749, 5000, 5249, 7500, 7570, 10000, 17500, 22500, 50000, 65000, 67500, 72500, 75000}; //16
	//int idArray[N_ID] = {0, 2500, 5000, 10000, 17500, 22500, 50000, 65000, 67500, 72500, 75000}; //11
	//int idArray[N_ID] = {0, 10000, 17500, 22500, 65000}; //5
	//int idArray[N_ID] = {0, 249, 2500, 2749, 5000, 5249, 7500, 7570};//16
	//int idArray[N_ID] = {7500, 12700, 17700, 20100, 45000};//5

	//int idArray[N_ID] = {20000, 237501};//2
	//int idArray[N_ID] = {17500, 17501, 17502, 17503, 17504, 17505, 17506};//
	int idArray[N_ID] = {12500, 45000};//2
	*/

/*
	int clothIdArray[5] = {1,2,8,15,22};
	vector<int> idArray;
	for (int i=0; i<5; i++)
	{
		for (int j=0; j<250; j+=10)
		{
			idArray.push_back(clothIdArray[i]*2500 + 250 + j);
		}
	}
	edId = idArray.size()-1;
	*/

	//double st = clock();
	//edId = 7;
	for (int idx = stId; idx <= edId; idx++)
    {
		if (isInference)
		{
        	//encode(idArray[idx], outPath, true, dispPath, dpPath, legalPath, isInference);
        	encode(idx, outPath, true, dispPath, dpPath, legalPath, isInference);
		}
		else
		{
        	//encode(idArray[idx], outPath, true, dispPath, dpPath, legalPath, isInference);
        	encode(idx, outPath, true, dispPath, dpPath, legalPath);
		}
	}
	//double ed = clock();
	//printf("aver time: %f ms\n", (ed - st)/N_ID/CLOCKS_PER_SEC * 1000);
}

void singleObj()
{
    string objFileName = "Tshirt_fit_middle.obj";
    string bodyFileName = "body.obj";
    string outPath = "results/";
    dealWithOneCase(objFileName, bodyFileName, outPath);
}

int getHfromCMU(const string& path, int c, int m, int u)
{
    char filename[256];
	int h = 0;
	for (; h < 100; h++)
	{
		sprintf(filename, "C%03dM%02dH%02d/%04d_00.obj", c, m, h, u);
		FILE* fin = fopen((path + filename).c_str(), "rb");
		if (fin != NULL) 
		{
			fclose(fin);
			break;
		}
	}
	if (h >= 100) 
	{
		return -1;
	}
	return h;
}

void singleObjWithId(int c, int m, int u)
{
    string path = "/home/yushen/workspace/sim_data_new/";
    char filename[256];

	int idx = c*10*250 + m*250 + u;
	printf("idx %d begin\n", idx);

	int h = 0;
	for (; h < 100; h++)
	{
		sprintf(filename, "C%03dM%02dH%02d/%04d_00.obj", c, m, h, u);
		FILE* fin = fopen((path + filename).c_str(), "rb");
		if (fin != NULL) 
		{
			fclose(fin);
			break;
		}
	}
	if (h >= 100) 
	{
		printf("idx %d end\n", idx);
		return;
	}

	sprintf(filename, "C%03dM%02dH%02d/%04d_00.obj", c, m, h, u);
	string objFileName = path + filename;
	std::cout << objFileName << std::endl;

	sprintf(filename, "C%03dM%02dH%02d/obs%04d_00.obj", c, m, h, u);
	string bodyFileName = path + filename;

	string outPath = path + "cloth105/";
    dealWithOneCase(objFileName, bodyFileName, outPath, idx);

	printf("idx %d end\n", idx);
}

void multiObj()
{
    string path = "/home/yushen/workspace/sim_data/";
    char filename[256];
    int idx = 0;
    for (int c = 0; c < 14; c++)
    {
        for (int m = 0; m < 10; m++)
        {
            int h = 0;
            for (; h < 100; h++)
            {
                sprintf(filename, "C%03dM%02dH%02d/0000_00.obj", c, m, h);
                FILE* fin = fopen((path + filename).c_str(), "rb");
                if (fin != NULL) 
				{
					fclose(fin);
					break;
				}
            }
            if (h >= 100) continue;

            for (int u = 0; u < 250; u++)
            {
				idx = c*10*250 + m*250 + u;
                sprintf(filename, "C%03dM%02dH%02d/%04d_00.obj", c, m, h, u);
                string objFileName = path + filename;
				std::cout << objFileName << std::endl;

                sprintf(filename, "C%03dM%02dH%02d/obs%04d_00.obj", c, m, h, u);
                string bodyFileName = path + filename;

                string outPath = path + "cloth14/";

				try
				{
					if (!dealWithOneCase(objFileName, bodyFileName, outPath, idx))
						break;
				}
				catch(...)
				{
					std::cout << idx << " failed!" << std::endl;
				}
				std::cout << idx << " finished!" << std::endl;
            }
        }
    }
}

void legal_map_test()
{
    char filename[256];
    //int idArr[8] = { 0, 249, 2500, 2749, 5000, 5249, 7500, 7570 };
    const int N_ID_ARRAY = 1;
    //int idArr[N_ID_ARRAY] = { 0, 2500, 5000, 10000, 17500, 22500, 50000, 65000, 67500, 72500, 75000 };
    //int idArr[N_ID_ARRAY] = { 0, 10000, 17500, 22500, 65000 };
    //int idArr[N_ID_ARRAY] = { 7500, 12700, 17700, 20100, 45000 };
    //int idArr[N_ID_ARRAY] = { 17700, 17701, 17702, 17703, 17704, 17705, 17706 };
    //int idArr[N_ID_ARRAY] = { 17500, 17501, 17502, 17503, 17504, 17505, 17506 };
    int idArr[N_ID_ARRAY] = { 12500 };

    string path = "results/legal_map_test/simulation/";
    cv::Mat_<float> legalMapSum = cv::Mat_<float>::zeros(maxn, maxn);

    for (int id = 0; id < N_ID_ARRAY; id++)
    {
        cv::Mat_<float> legalMap(maxn, maxn);
        if (false)
        {
            sprintf(filename, "legal_map_%06d.dat", idArr[id]);
            readBinaryFileC1(path + filename, legalMap);
            saveLegalMap(legalMap, path, idArr[id], true, false);
        }
        else
        {
            sprintf(filename, "legal_map_%06d.jpg", idArr[id]);
            readImageFileC1(path + filename, legalMap);
            saveLegalMap(legalMap, path, idArr[id], false, true);
        }
        //sprintf(filename, "legal_map_%06d.jpg", id);


        //readBinaryFileC1(path+filename, legalMap);

        //saveLegalMap(legalMap, path, idArr[id] + 1);

        legalMapSum = legalMapSum + legalMap;
    }

    //saveLegalMap(legalMapSum, path + "new_legal_map/", 0);
}


void do_mapping(Face *face) {
    double scale = maxn;
    vector<Node> uvs;
    uvs.push_back(Node(body.u[face->uv[0]], body.v[face->uv[0]], 0));
    uvs.push_back(Node(body.u[face->uv[1]], body.v[face->uv[1]], 0));
    uvs.push_back(Node(body.u[face->uv[2]], body.v[face->uv[2]], 0));
    BBox uvbox = BBox(uvs[0] * scale) + BBox(uvs[1] * scale) + BBox(uvs[2] * scale);
    for (int u = uvbox.h[0]; u <= uvbox.t[0] + 1; ++u)
        for (int v = uvbox.h[1]; v <= uvbox.t[1] + 1; ++v) {
            pair<double, double> w = get_barycentric(uvs, Node(u / scale, v / scale, 0), Node(0, 0, 1));
            double wp = w.first, wq = w.second;
            if (wp < -eps || wq < -eps || wp + wq > 1 + eps)
                continue;
            bool verbose = false;//(u==1261&&v==1235);
            if (verbose)
                cout << endl << wp << " " << wq << endl;


            //Node dp = *nodes[0] * (1 - wp - wq) + *nodes[1] * wp + *nodes[2] * wq;
            Node dp = (*(body.nodes[face->ind[0]])) * (1 - wp - wq) 
                + (*(body.nodes[face->ind[1]])) * wp 
                + (*(body.nodes[face->ind[2]])) * wq;
            dpMap[u][v] = dp;
        }


    set<pair<int, int>> visited;
    for (int i = 0; i < 3; ++i) {
        int len = 10;
        for (int j = 0; j <= len; ++j) {
            Node cur = ((*(body.nodes[face->ind[i]])) * (len - j) + (*(body.nodes[face->ind[(i+1)%3]])) * j) / len;
            pair<double, double> w = get_barycentric({ *(body.nodes[face->ind[0]]),*(body.nodes[face->ind[1]]),*(body.nodes[face->ind[2]]) }, cur, Node(0, 0, 1));
            double wp = w.first, wq = w.second;
            int u = (int)round((uvs[0][0] * (1 - wp - wq) + uvs[1][0] * wp + uvs[2][0] * wq)*scale);
            int v = (int)round((uvs[0][1] * (1 - wp - wq) + uvs[1][1] * wp + uvs[2][1] * wq)*scale);
            if (visited.find({ u,v }) != visited.end()) continue;
            Node dp = cur;
            dpMap[u][v] = dp;
            visited.insert({ u,v });
        }
    }
}

void clearDpMap()
{
    for (int i = 0; i < maxn; i++)
    {
        for (int j = 0; j < maxn; j++)
        {
            dpMap[i][j].x = 0;
            dpMap[i][j].y = 0;
            dpMap[i][j].z = 0;
        }
    }
}

void saveDpMap(const string& filename)
{
    cv::Mat DpMap(maxn, maxn, CV_32FC3);

    for (int i = 0; i < maxn; i++)
    {
        for (int j = 0; j < maxn; j++)
        {
            float* ptr_DpMap = DpMap.ptr<float>(i);
            ptr_DpMap[j * 3 + 0] = (float)dpMap[i][j].x;
            ptr_DpMap[j * 3 + 1] = (float)dpMap[i][j].y;
            ptr_DpMap[j * 3 + 2] = (float)dpMap[i][j].z;
//             if (is0(dpMap[i][j].x) && is0(dpMap[i][j].y) && is0(dpMap[i][j].z))
//             {
//                 ptr_DpMap[j * 3 + 0] = -2;
//                 ptr_DpMap[j * 3 + 1] = -2;
//                 ptr_DpMap[j * 3 + 2] = -2;
//             }
//             else
//             {
//                 ptr_DpMap[j * 3 + 0] = 1;
//                 ptr_DpMap[j * 3 + 1] = 1;
//                 ptr_DpMap[j * 3 + 2] = 1;
//             }
        }
    }

    double minVal, maxVal;
    cv::minMaxLoc(DpMap, &minVal, &maxVal);

    //string filename_img = "/home/yushen/workspace/sim_data_new/cloth105/regenerated_dp_map_new/dp_map_000000.jpg";
//     string filename_img = "D:/data/garment project/cloth105/regenerated_dp_map_new/dp_map_000000.jpg";
//     cv::imwrite(filename_img, (DpMap- minVal)/(maxVal - minVal) *255);

    saveBinaryFile(filename, DpMap);
}

string getBodyFileNameById(int id, const string& path)
{
	int c = id / 2500;
	id %= 2500;
	int m = id / 250;
	int u = id % 250;

	int h = getHfromCMU(path, c, m, u);
	if (h < 0) {puts("file not exist!!!!!!"); return "";}

    char filename[256];
	sprintf(filename, "C%03dM%02dH%02d/obs%04d_00.obj", c, m, h, u);
	string bodyFileName = path + filename;

	return bodyFileName;
}

void createDpMapFromBody()
{
    //string path = "D:/data/garment project/cloth105/";
    string path = "/home/yushen/workspace/sim_data_new/";
    char filename[256];

	int idArray[8] = {0, 249, 2500, 2749, 5000, 5249, 7500, 7570};

    for (int i = 0; i < 8; i++)
    {
		printf("%d\n", i);
        //sprintf(filename, "retarget_body/retarget_body_%d.obj", i);
        string bodyFileName = getBodyFileNameById(idArray[i], path);
        //string bodyFileName = "/home/yushen/workspace/sim_data_new/C001M00H06/obs0000_00.obj";

//         sprintf(filename, "D:/data/garment project/cloth105/retarget_body/new1/retarget_body_%d.obj", i);
//         string bodyFileName = filename;

        ifstream fin1(bodyFileName);
        if (!fin1.is_open())
        {
            puts("open file fail");
        }
        body.read(fin1);
        fin1.close();

        clearDpMap();
        for (auto *face : body.faces)
            do_mapping(face);

        sprintf(filename, "cloth105/regenerated_dp_map_new/dp_map_%06d.dat", idArray[i]);
        string outFilename = path + filename;
        saveDpMap(outFilename);
    }
}

double compare(const Mesh& meshA, const Mesh& meshB)
{
    KDTree rootA = KDTree(meshA.nodes);
    KDTree rootB = KDTree(meshB.nodes);
	
    double distTotal = 0;
	int cntTotal = 0;
	int step = 10;
    for (int i=0; i<meshB.nodes.size(); i+=step)
    {
		auto& nodeOnB = meshB.nodes[i];
        auto nodesOnA = get_closest(rootA, nodeOnB);
        distTotal += dist(*nodeOnB, *nodesOnA[0]);
		cntTotal++;
    }

    for (int i=0; i<meshA.nodes.size(); i+=step)
    {
		auto& nodeOnA = meshA.nodes[i];
        auto nodesOnB = get_closest(rootB, nodeOnA);
        distTotal += dist(*nodeOnA, *nodesOnB[0]);
		cntTotal++;
    }

	if (cntTotal == 0) cntTotal = 1;

    return distTotal / cntTotal;
}

void compare(int c, int m, int u)
{
    Mesh clothOld, clothNew;

    //string path = "D:/data/garment project/";
    string path = "/home/yushen/workspace/sim_data_new/";
    char filename[256];

    int idx = c * 10 * 250 + m * 250 + u;
    printf("idx %d begin\n", idx);

    int h = 0;
    for (; h < 100; h++)
    {
        sprintf(filename, "C%03dM%02dH%02d/%04d_00.obj", c, m, h, u);
        FILE* fin = fopen((path + filename).c_str(), "rb");
        if (fin != NULL)
        {
            fclose(fin);
            break;
        }
    }
    if (h >= 100)
    {
        printf("idx %d end\n", idx);
        return;
    }

    string dispPath = path + "cloth105/train_displacement_map/";
    string dpPath = path + "cloth105/reconstruct_dp_map/";
    string legalPath = path + "cloth105/train_legal_map/";
    string outPath = path + "cloth105/";
    recoverObj(dispPath, dpPath, legalPath, outPath, idx, idx);

    sprintf(filename, "C%03dM%02dH%02d/%04d_00.obj", c, m, h, u);
    string objFileName = path + filename;
    std::cout << objFileName << std::endl;
    ifstream fin(objFileName);
    clothOld.read(fin);

    sprintf(filename, "/reconstruct_mesh/mesh_tmp.obj");
    string objFileNameNew = outPath + filename;
    ifstream fin1(objFileNameNew);
    clothNew.read(fin1);

    double dist = compare(clothOld, clothNew);

    string outFileName = path + "cloth105/dist.txt";
    FILE* fout = fopen(outFileName.c_str(), "a");
    fprintf(fout, "%d %f\n", idx, dist);
    fclose(fout);


    printf("idx %d end\n", idx);
}

double calDistOfDisp(const cv::Mat_<cv::Point3f>& dispA, const cv::Mat_<cv::Point3f>& dispB)
{
//     auto dispAtmp = dispA * 100;
//     auto dispBtmp = dispB * 100;
    cv::Mat_<cv::Point3f> dispDelta;
    cv::absdiff(dispA, dispB, dispDelta);
    auto dists = cv::sum(dispDelta);
    return dists[0];
}

int findBestMatch(const cv::Mat_<cv::Point3f>& dispBase, const string& path)
{
    char filename[256];
    cv::Mat_<cv::Point3f> dispCompare;
    double minDist = 1e10;
    int minId = -1;

    for (int c = 0; c < 105; c++)
    {
        for (int m = 0; m < 2; m++)
        {
            for (int u = 0; u < 250; u++)
            {
                int idx = c * 2500 + m * 250 + u;
                if (idx == 2500)
                    c = c;
                sprintf(filename, "cloth105/train_displacement_map/displacement_map_%06d.dat", idx);
                string dispFileName = path + filename;

                if (!readBinaryFileC3(dispFileName, dispCompare)) continue;

                double dist = calDistOfDisp(dispBase, dispCompare);
                if (minDist > dist)
                {
                    minDist = dist;
                    minId = idx;
                }
            }
        }
    }

    return minId;
}

void bestMatchTest()
{
    string path = "D:/data/garment project/";

    string filenameBase = "D:/data/garment project/cloth105/displacement_map_002500.dat";
    cv::Mat_<cv::Point3f> dispBase;
    if (!readBinaryFileC3(filenameBase, dispBase))
    {
        puts("No base file");
        return;
    }

    int bestId = findBestMatch(dispBase, path);

    char filename[256];
    sprintf(filename, "cloth105/train_displacement_map/displacement_map_%06d.dat", bestId);

    cv::Mat_<cv::Point3f> dispBest;
    readBinaryFileC3(path + filename, dispBest);

    sprintf(filename, "cloth105/best_match_displacement_map/displacement_map_%06d.dat", bestId);
    saveBinaryFile(path + filename, dispBest);

    sprintf(filename, "cloth105/best_match_displacement_map/displacement_map_%06d.jpg", bestId);
    cv::Mat_<cv::Point3f> tmp = dispBest * 255;
    cv::imwrite(path + filename, dispBest * 255);
    cv::imwrite(path + filename, dispBest * 50);
    cv::imwrite(path + filename, dispBest);
}

void visualize(int argc, char **argv)
{
	int mode = std::stoi(argv[1]);
	if (mode == 0 || mode == 1) // 0:displacement_map   1:dp_map
	{
		printf("Reading displacement_map or dp_map %s\n", argv[2]);
		cv::Mat_<cv::Point3f> data;
    	if (!readBinaryFileC3(string(argv[2]), data))
		{
			printf("Can't read file %s\n", argv[2]);
			return;
		}
     	double minVal, maxVal;
     	cv::minMaxLoc(data, &minVal, &maxVal);
		printf("min value %f, max value %f\n", minVal, maxVal);
    	//cv::imwrite("res/data.jpg", data);
		cv::imwrite("res/data.jpg", (data- minVal)/(maxVal - minVal) *255);
	}
	else if (mode == 2) //legal_map
	{
		printf("Reading legal_map %s\n", argv[2]);
		cv::Mat_<float> data;
    	if (!readBinaryFileC1(string(argv[2]), data))
		{
			printf("Can't read file %s\n", argv[2]);
			return;
		}
     	double minVal, maxVal;
     	cv::minMaxLoc(data, &minVal, &maxVal);
		printf("min value %f, max value %f\n", minVal, maxVal);
    	//cv::imwrite("res/data.jpg", data*255);
		cv::imwrite("res/data.jpg", (data- minVal)/(maxVal - minVal) *255);
	}
	puts("Done");
}

void visualize3(int argc, char **argv)
{
	int idx = std::stoi(argv[1]);
	string path = "/home/yushen/workspace/sim_data_new/cloth105_new/";
	char filename[256];
	sprintf(filename, "%s/train_displacement_map/displacement_map_%06d.dat", path.c_str(), idx); 
	cv::Mat_<cv::Point3f> data3;
	if (!readBinaryFileC3(filename, data3))
	{
		printf("Can't read file %s\n", filename);
		return;
	}
	double minVal, maxVal;
	cv::minMaxLoc(data3, &minVal, &maxVal);
	printf("displacement_map min value %f, max value %f\n", minVal, maxVal);
	cv::imwrite("res/displacement_map.jpg", (data3- minVal)/(maxVal - minVal) *255);

	sprintf(filename, "%s/reconstruct_dp_map/dp_map_%06d.dat", path.c_str(), idx);
	if (!readBinaryFileC3(filename, data3))
	{
		printf("Can't read file %s\n", filename);
		return;
	}
	cv::minMaxLoc(data3, &minVal, &maxVal);
	printf("dp_map min value %f, max value %f\n", minVal, maxVal);
	cv::imwrite("res/dp_map.jpg", (data3- minVal)/(maxVal - minVal) *255);

	sprintf(filename, "%s/train_legal_map/legal_map_%06d.dat", path.c_str(), idx); 
	cv::Mat_<float> data1;
	if (!readBinaryFileC1(filename, data1))
	{
		printf("Can't read file %s\n", filename);
		return;
	}
	cv::minMaxLoc(data1, &minVal, &maxVal);
	printf("legal_map min value %f, max value %f\n", minVal, maxVal);
	cv::imwrite("res/legal_map.jpg", (data1- minVal)/(maxVal - minVal) *255);
}

int main(int argc, char **argv) {

	//visualize3(argc, argv);
	//return 0;

    //legal_map_test();

    //singleObjWithId(1,0,0);

    //createDpMapFromBody();

	//recoverObj();
    //recoverObj("","","","",0,0,true);

    //bestMatchTest();
	/*
	for (int c=0; c<105; c++)
	{
		for (int m=0; m<2; m++)
		{
			for (int u=0; u<250; u++)
			{
				compare(c,m,u);
			}
		}
	}
	*/

	//return 0;

	if (argc < 4)
	{
		recoverObj();
    	//singleObj();
    	//multiObj();
	}
	else
	{
		int mode = std::stoi(argv[1]);
		
		if (mode == 0)
		{
			singleObjWithId(std::stoi(argv[2]), std::stoi(argv[3]), std::stoi(argv[4]));
		}
		else if (mode == 1)
		{
			recoverObj(argv[2], argv[3], argv[4], argv[5], std::stoi(argv[6]), std::stoi(argv[7]));
		}
        else if (mode == 2)
        {
            compare(std::stoi(argv[2]), std::stoi(argv[3]), std::stoi(argv[4]));
        }
	}
	
	return 0;
}
