/*
 * file: KDTree.hpp
 * author: J. Frederico Carvalho
 *
 * This is an adaptation of the KD-tree implementation in rosetta code
 * https://rosettacode.org/wiki/K-d_tree
 *
 * It is a reimplementation of the C code using C++.  It also includes a few
 * more queries than the original, namely finding all points at a distance
 * smaller than some given distance to a point.
 *
 */


#include "KDTree.hpp"

KDNode::KDNode() = default;

KDNode::KDNode(const point_t &pt, const double &x_, const point_t &n_, const size_t &idx_, const KDNodePtr &left_,
               const KDNodePtr &right_) {
    yz = pt;
    x = x_;
    n = n_;
    index = idx_;
    left = left_;
    right = right_;
}

// for vertex normal
KDNode::KDNode(const pointNormalIndex &pi, const KDNodePtr &left_,
               const KDNodePtr &right_) {
    yz = std::get<1>(pi).first;
    x = std::get<0>(pi);
    n = std::get<2>(pi);
    index = std::get<1>(pi).second;
    left = left_;
    right = right_;
}

KDNode::~KDNode() = default;

double KDNode::coord(const size_t &idx) { return yz.at(idx); }
KDNode::operator bool() { return (!yz.empty()); }
KDNode::operator point_t() { return yz; }
KDNode::operator size_t() { return index; }
KDNode::operator pointIndex() { return pointIndex(yz, index); }

point_t KDNode::xyz(){
    return std::vector<double>{x, yz[0], yz[1]};
}


KDNodePtr NewKDNodePtr() {
    KDNodePtr mynode = std::make_shared< KDNode >();
    return mynode;
}

inline double dist2(const point_t &a, const point_t &b) {
    double distc = 0;
    for (size_t i = 0; i < a.size(); i++) {
        double di = a.at(i) - b.at(i);
        distc += di * di;
    }
    return distc;
}

inline double dist2(const KDNodePtr &a, const KDNodePtr &b) {
    return dist2(a->yz, b->yz);
}

inline double dist(const point_t &a, const point_t &b) {
    return std::sqrt(dist2(a, b));
}

inline double dist(const KDNodePtr &a, const KDNodePtr &b) {
    return std::sqrt(dist2(a, b));
}

comparer::comparer(size_t idx_) : idx{idx_} {};

inline bool comparer::compare_idx_n(const pointNormalIndex &a,  //
                                  const pointNormalIndex &b   //
) {
    return (std::get<1>(a).first.at(idx) < std::get<1>(b).first.at(idx));  //
}

comparerD::comparerD(point_t pt_) : pt{pt_} {};

inline bool comparerD::compare_dist(const KDNodePtr &a,  //
                                  const KDNodePtr &b   //
) {
    return (dist2(a->yz, pt) < dist2(b->yz, pt));  //
}

inline void sort_on_idx_n(const pointNormalIndexArr::iterator &begin,  //
                        const pointNormalIndexArr::iterator &end,    //
                        size_t idx) {
    comparer comp(idx);
    comp.idx = idx;

    using std::placeholders::_1;
    using std::placeholders::_2;

    std::nth_element(begin, begin + std::distance(begin, end) / 2,
                     end, std::bind(&comparer::compare_idx_n, comp, _1, _2));
}

inline void sort_on_dist(const std::vector<KDNodePtr>::iterator &begin,  //
                        const std::vector<KDNodePtr>::iterator &end,    //
                        point_t pt) {
    comparerD comp(pt);
    comp.pt = pt;

    using std::placeholders::_1;
    using std::placeholders::_2;

    // std::nth_element(begin, begin + std::distance(begin, end) / 2,
    //                  end, std::bind(&comparerD::compare_dist, comp, _1, _2));
    std::sort(begin, end, std::bind(&comparerD::compare_dist, comp, _1, _2));
}

using pointVec = std::vector< point_t >;

KDNodePtr KDTree::make_tree_n(const pointNormalIndexArr::iterator &begin,  //
                            const pointNormalIndexArr::iterator &end,    //
                            const size_t &length,                  //
                            const size_t &level                    //
) {
    if (begin == end) {
        return NewKDNodePtr();  // empty tree
    }

    size_t dim = std::get<1>(*begin).first.size();

    if (length > 1) {
        sort_on_idx_n(begin, end, level);
    }

    auto middle = begin + (length / 2);

    auto l_begin = begin;
    auto l_end = middle;
    auto r_begin = middle + 1;
    auto r_end = end;

    size_t l_len = length / 2;
    size_t r_len = length - l_len - 1;

    KDNodePtr left;
    if (l_len > 0 && dim > 0) {
        left = make_tree_n(l_begin, l_end, l_len, (level + 1) % dim);
    } else {
        left = leaf;
    }
    KDNodePtr right;
    if (r_len > 0 && dim > 0) {
        right = make_tree_n(r_begin, r_end, r_len, (level + 1) % dim);
    } else {
        right = leaf;
    }

    // KDNode result = KDNode();
    return std::make_shared< KDNode >(*middle, left, right);
}

// KDTree::KDTree(pointVec point_array, pointVec normal_array) {
//     leaf = std::make_shared< KDNode >();
//     // iterators
//     double x;
//     pointIndex pi;
//     pointNormalIndexArr arr;
//     for (size_t i = 0; i < point_array.size(); i++) {
//         x = point_array.at(i)[0];
//         pi = pointIndex({point_array.at(i)[1],point_array.at(i)[2]}, i);
//         arr.push_back(pointNormalIndex(x, pi, normal_array.at(i)));
//     }

//     auto begin = arr.begin();
//     auto end = arr.end();

//     size_t length = arr.size();
//     size_t level = 0;  // starting

//     root = KDTree::make_tree_n(begin, end, length, level);
// }

KDNodePtr KDTree::nearest_(   //
    const KDNodePtr &branch,  //
    const point_t &pt,        //
    const size_t &level,      //
    const KDNodePtr &best,    //
    const double &best_dist   //
) {
    double d, dx, dx2;

    if (!bool(*branch)) {
        return NewKDNodePtr();  // basically, null
    }

    point_t branch_pt(*branch);
    size_t dim = branch_pt.size();

    d = dist2(branch_pt, pt);
    dx = branch_pt.at(level) - pt.at(level);
    dx2 = dx * dx;

    KDNodePtr best_l = best;
    double best_dist_l = best_dist;

    if (d < best_dist) {
        best_dist_l = d;
        best_l = branch;
    }

    size_t next_lv = (level + 1) % dim;
    KDNodePtr section;
    KDNodePtr other;

    // select which branch makes sense to check
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    // keep nearest neighbor from further down the tree
    KDNodePtr further = nearest_(section, pt, next_lv, best_l, best_dist_l);
    if (!further->yz.empty()) {
        double dl = dist2(further->yz, pt);
        if (dl < best_dist_l) {
            best_dist_l = dl;
            best_l = further;
        }
    }
    // only check the other branch if it makes sense to do so
    if (dx2 < best_dist_l) {
        further = nearest_(other, pt, next_lv, best_l, best_dist_l);
        if (!further->yz.empty()) {
            double dl = dist2(further->yz, pt);
            if (dl < best_dist_l) {
                best_dist_l = dl;
                best_l = further;
            }
        }
    }

    return best_l;
};

// default caller
KDNodePtr KDTree::nearest_(const point_t &pt) {
    size_t level = 0;
    // KDNodePtr best = branch;
    double branch_dist = dist2(point_t(*root), pt);
    return nearest_(root,          // beginning of tree
                    pt,            // point we are querying
                    level,         // start from level 0
                    root,          // best is the root
                    branch_dist);  // best_dist = branch_dist
};

point_t KDTree::nearest_point(const point_t &pt) {
    KDNodePtr node = nearest_(pt);
    point_t np = {node->x, node->yz[0], node->yz[1]};
    return np;
};

size_t KDTree::nearest_index(const point_t &pt) {
    return size_t(*nearest_(pt));
};

pointIndex KDTree::nearest_pointIndex(const point_t &pt) {
    KDNodePtr node = nearest_(pt);
    point_t np = {node->x, node->yz[0], node->yz[1]};
    return pointIndex(np, size_t(*node));
}

KDNodeArr KDTree::neighborhood_(  //
    const KDNodePtr &branch,          //
    const point_t &pt,                //
    const double &rad,                //
    const size_t &level               //
) {
    double d, dx, dx2;

    if (!bool(*branch)) {
        // branch has no point, means it is a leaf,
        // no points to add
        return KDNodeArr();
    }

    size_t dim = pt.size();

    double r2 = rad * rad;

    d = dist2(point_t(*branch), pt);
    dx = point_t(*branch).at(level) - pt.at(level);
    dx2 = dx * dx;

    KDNodeArr nbh, nbh_s, nbh_o;
    if (d <= r2) {
        nbh.push_back(branch);
    }

    //
    KDNodePtr section;
    KDNodePtr other;
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    nbh_s = neighborhood_(section, pt, rad, (level + 1) % dim);
    nbh.insert(nbh.end(), nbh_s.begin(), nbh_s.end());
    if (dx2 < r2) {
        nbh_o = neighborhood_(other, pt, rad, (level + 1) % dim);
        nbh.insert(nbh.end(), nbh_o.begin(), nbh_o.end());
    }

    return nbh;
};

KDNodeArr KDTree::neighborhood(  //
    const point_t &pt,               //
    const double &rad) {
    size_t level = 0;
    return neighborhood_(root, pt, rad, level);
}

pointVec KDTree::neighborhood_points(  //
    const point_t &pt,                 //
    const double &rad) {
    size_t level = 0;
    KDNodeArr nbh = neighborhood_(root, pt, rad, level);
    pointVec nbhp;
    nbhp.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhp.begin(),
                   [](KDNodePtr node) { return std::vector<double>{node->x, node->yz[0], node->yz[1]}; });
    return nbhp;
}

indexArr KDTree::neighborhood_indices(  //
    const point_t &pt,                  //
    const double &rad) {
    size_t level = 0;
    KDNodeArr nbh = neighborhood_(root, pt, rad, level);
    indexArr nbhi;
    nbhi.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhi.begin(),
                   [](KDNodePtr node) { return node->index; });
    return nbhi;
}

void KDTree::print_node_(KDNodePtr &node){
    point_t vertex = node->yz;
    std::cout << "vertex(" << node->index << "): " << node->x;
    for(int i = 0; i < vertex.size(); i++) std::cout << " " << vertex[i];

    point_t normal = node->n;
    if(!normal.empty()){
        std::cout << ", normal:";
        for(int i = 0; i < normal.size(); i++) std::cout << " " << normal[i];
    }
    std::cout << std::endl;

    
    if(!node->left->yz.empty()) print_node_(node->left);
    if(!node->right->yz.empty()) print_node_(node->right);
}

void KDTree::print_tree(){
    std::cout << "####Printing KD-Tree: " << std::endl;
    print_node_(root);
}

void KDTree::search_quad_(   //
    const KDNodePtr &branch,  //
    const point_t &pt,        //
    const size_t &level,      //
    KDNodeArr &quad
) {
    std::cout << "Search quad_\n";
    
    std::cout << "find dist2: ";
    double d = dist2(branch->yz, pt);
    size_t dim = branch->yz.size();

    //left-bottom  0 
    if(pt[0] >= branch->yz[0] && pt[1] > branch->yz[1]){
        std::cout << "lb\n";

        if(quad[0]->yz.empty()) quad[0] = branch;
        else{
            if(dist2(quad[0]->yz, pt) > d) quad[0] = branch;
        }
    }

    //right-bottom 1
    else if(pt[0] < branch->yz[0] && pt[1] >= branch->yz[1])
    {
        std::cout << "rb\n";

        if(quad[1]->yz.empty()) quad[1] = branch;
        else{
            if(dist2(quad[1]->yz, pt) > d) quad[1] = branch;
        }
    }

    //right-top    2
    else if(pt[0] <= branch->yz[0] && pt[1] < branch->yz[1]){
        std::cout << "rt\n";

        if(quad[2]->yz.empty()) quad[2] = branch;
        else{
            if(dist2(quad[2]->yz, pt) > d) quad[2] = branch;
        }
    }

    //left-top     3
    else if(pt[0] > branch->yz[0] && pt[1] <= branch->yz[1]){
        std::cout << "lt\n";

        if(quad[3]->yz.empty()) quad[3] = branch;
        else{
            if(dist2(quad[3]->yz, pt) > d) quad[3] = branch;
        }
    }

    if(!branch->right->yz.empty())
        search_quad_(branch->right, pt, (level+1)%dim, quad);
    if(!branch->left->yz.empty())
        search_quad_(branch->left, pt, (level+1)%dim, quad);
};

KDNodeArr KDTree::search_quad(const point_t &pt){
    size_t level = 0;
    KDNodeArr quad; 
    quad.assign(4, NewKDNodePtr());

    std::cout << "Search quad\n";
    search_quad_(root, pt, level, quad);

    for(auto a : quad){
        std::cout << a->x << " " << a->yz[0] << " " << a->yz[1] ;
        std::cout << ", dist: " << dist2(a->yz, pt)<< std::endl;
    }
    return quad;
}

KDNodeArr KDTree::search_quad(const point_t &pt, const double &r){
    KDNodeArr nbh = neighborhood(pt, r);
    // std::cout << nbh.size() << std::endl;

    sort_on_dist(nbh.begin(), nbh.end(), pt);

    KDNodeArr quad;
    quad.assign(4, NewKDNodePtr());

    for(auto a: nbh){
        if(pt[0] >= a->yz[0] && pt[1] > a->yz[1]){ // left_bottom
            if(quad[0]->yz.empty()) {quad[0] = a;} //std::cout << "lb ";}
        }
        else if(pt[0] < a->yz[0] && pt[1] >= a->yz[1]){ //right_bottom
            if(quad[1]->yz.empty()) {quad[1] = a;} //std::cout << "rb ";}
        }
        else if(pt[0] <= a->yz[0] && pt[1] < a->yz[1]){ // right_top
            if(quad[2]->yz.empty()) {quad[2] = a;} //std::cout << "rt ";}
        }
        else if(pt[0] > a->yz[0] && pt[1] <= a->yz[1]){ // left_top
            if(quad[3]->yz.empty()) {quad[3] = a;} //std::cout << "lt ";}
        }
    }
    // std::cout << std::endl;
    
    return quad;
}