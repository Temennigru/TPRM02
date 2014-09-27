#ifndef _SEGMENT_INTERSECT_H
#define _SEGMENT_INTERSECT_H

// Returns true if two edges p1-q1 and p2-q2 cross each other
template <typename T> extern bool doIntersect(T p1x, T p1y, T q1x, T q1y, T p2x, T p2y, T q2x, T q2y, bool allowCollinearity = false);

#endif