#include "snippet_gpmds/snippet.h"

template<typename point>
Snippet<point>::Snippet(point anchor) {
  SetAnchor(anchor);
}

template<typename point>
Snippet<point>::Snippet(point anchor, const std::vector<point> &absolute_points) {
  absolute_points_.assign(absolute_points.begin(), absolute_points.end());
  // NOTE: Set the anchor *after* the absolute points to compute the relative
  // points.
  SetAnchor(anchor);
}

template<typename point>
void Snippet<point>::RelativePoints(
        std::vector<point> *points) const {
  points->clear();
  for(auto pt : relative_points_) {
    points->push_back(pt);
  }
}

template<typename point>
void Snippet<point>::SetAnchor(const point &anchor,
                               bool recompute_relative_points /* = true */) {
  if(!recompute_relative_points) {
    printf("WARNING: Not recomputing relative points, this is probably "
                   "un-intended.");
  }

  anchor_ = anchor;

  ComputeRelativePoints(&relative_points_);
}

template<typename point>
bool Snippet<point>::ComputeRelativePoints(
        std::vector<point> *relative_points) {
  return ComputeRelativePoints(anchor_, absolute_points_, relative_points);
}

template<typename point>  // static
bool Snippet<point>::ComputeRelativePoints(
        const point &anchor,
        const std::vector<point> &absolute_points,
        std::vector<point> *relative_points) {


  relative_points->clear();

  for (const auto &pt : absolute_points) {
    relative_points->push_back(anchor + pt);
  }
  return true;
}