//
// Author: Felix Duvallet <felix.duvallet@epfl.ch>
//
#ifndef __SNIPPET_H__
#define __SNIPPET_H__

#include <vector>

// A snippet is a templated class
template<typename point>
class Snippet {

 public:

    point anchor_;

    std::vector<point> absolute_points_;
    std::vector<point> relative_points_;

    // Set the anchor and the absolute trajectory points. Automatically
    // computes the relative points.
    Snippet(point anchor, const std::vector<point> &absolute_points);

    // Initialize the snippet with no points (just an anchor);
    Snippet(point anchor);

    point get_anchor() const { return anchor_; }

    unsigned int num_points() const { return absolute_points_.size(); }

    // Get a vector of const references to the points. Clears the input vector.
    void RelativePoints(std::vector<point> *points) const;

    // Set the anchor and optionally recompute all the relative points. If the
    // anchor is unchanged, then this shortcuts the recomputation.
    void SetAnchor(const point &anchor, bool recompute_relative_points = true);

    // Compute the relative points, essentially doing:
    //    relative = [anchor + abs for abs in absolute]
    bool ComputeRelativePoints(std::vector<point> *relative_points);

    static bool ComputeRelativePoints(
            const point &anchor,
            const std::vector<point> &absolute_points,
            std::vector<point> *relative_points);

};

#include "snippet_gpmds/snippet.hxx"

#endif //__SNIPPET_H__
