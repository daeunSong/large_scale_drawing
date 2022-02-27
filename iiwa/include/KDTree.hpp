#pragma once

/*
 * file: KDTree.hpp
 * author: J. Frederico Carvalho
 *
 * This is an adaptation of the KD-tree implementation in rosetta code
 *  https://rosettacode.org/wiki/K-d_tree
 * It is a reimplementation of the C code using C++.
 * It also includes a few more queries than the original
 *
 */

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

using point_t = std::vector< double >;
using pointVec = std::vector< point_t >;

using indexArr = std::vector< size_t >;

using pointIndex = typename std::pair< std::vector< double >, size_t >;
using pointIndexArr = typename std::vector< pointIndex >;

using pointNormalIndex = typename std::tuple< double, pointIndex , std::vector< double > >;
using pointNormalIndexArr = typename std::vector< pointNormalIndex >;

class KDNode {
   public:
    using KDNodePtr = std::shared_ptr< KDNode >;
    size_t index;
    point_t yz;
    double x;  // vertex
    point_t n;  // vertex normal
    KDNodePtr left;
    KDNodePtr right;

    // initializer
    KDNode();
    KDNode(const point_t &, const double &, const point_t &, const size_t &,
            const KDNodePtr &, const KDNodePtr &);
    KDNode(const pointNormalIndex &, const KDNodePtr &, const KDNodePtr &);
    ~KDNode();

    // getter
    double coord(const size_t &);

    // conversions
    explicit operator bool();
    explicit operator point_t();
    explicit operator size_t();
    explicit operator pointIndex();

    point_t xyz();
};

using KDNodePtr = std::shared_ptr< KDNode >;
using KDNodeArr = typename std::vector<KDNodePtr>;

KDNodePtr NewKDNodePtr();

// square euclidean distance
inline double dist2(const point_t &, const point_t &);
inline double dist2(const KDNodePtr &, const KDNodePtr &);

// euclidean distance
inline double dist(const point_t &, const point_t &);
inline double dist(const KDNodePtr &, const KDNodePtr &);

// Need for sorting
class comparer {
   public:
    size_t idx;
    explicit comparer(size_t idx_);
    inline bool compare_idx_n(
        const pointNormalIndex &,  //
        const pointNormalIndex &   //
    );
};

class comparerD {
   public:
    point_t pt;
    explicit comparerD(point_t pt_);
    inline bool compare_dist(
        const KDNodePtr &a,  //
        const KDNodePtr &b   //
    );
};

inline void sort_on_idx_n(const pointNormalIndexArr::iterator &,  //
                        const pointNormalIndexArr::iterator &,  //
                        size_t idx);

inline void sort_on_dist(const std::vector<KDNodePtr>::iterator &,  //
                        const std::vector<KDNodePtr>::iterator &,    //
                        point_t pt);


class KDTree {
    KDNodePtr root;
    KDNodePtr leaf;

    KDNodePtr make_tree_n(const pointNormalIndexArr::iterator &begin,  //
                        const pointNormalIndexArr::iterator &end,    //
                        const size_t &length,                  //
                        const size_t &level                    //
    );

   public:
    KDTree() = default;
    explicit KDTree(pointVec point_array, pointVec normal_array);

   private:
    KDNodePtr nearest_(           //
        const KDNodePtr &branch,  //
        const point_t &pt,        //
        const size_t &level,      //
        const KDNodePtr &best,    //
        const double &best_dist   //
    );

    // default caller
    KDNodePtr nearest_(const point_t &pt);

   public:
    point_t nearest_point(const point_t &pt);
    size_t nearest_index(const point_t &pt);
    pointIndex nearest_pointIndex(const point_t &pt);

   private:
    KDNodeArr neighborhood_(  //
        const KDNodePtr &branch,  //
        const point_t &pt,        //
        const double &rad,        //
        const size_t &level       //
    );

   public:
    KDNodeArr neighborhood(  //
        const point_t &pt,       //
        const double &rad);

    pointVec neighborhood_points(  //
        const point_t &pt,         //
        const double &rad);

    indexArr neighborhood_indices(  //
        const point_t &pt,          //
        const double &rad);

   private:
    void print_node_(KDNodePtr &node);
    void search_quad_(   //
        const KDNodePtr &branch,  //
        const point_t &pt,        //
        const size_t &level,      //
        KDNodeArr &quad
    );
   public:
    void print_tree();
    KDNodeArr search_quad(const point_t &pt);
    KDNodeArr search_quad(const point_t &pt, const double &r);
};
