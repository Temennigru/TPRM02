#include <algorithm>
#include "SegmentIntersect.h"

// Adapted from http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
namespace{

	// A point
	template<typename T> struct point_t{
		T x, y;
		point_t(T x, T y) : x(x), y(y) {};
	};

	// Given three colinear point_ts p, q, r, the function checks if point_t q lies on line segment 'pr'
	template<typename T> bool onSegment(point_t<T> p, point_t<T> q, point_t<T> r){
		if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
			q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
			return true;
		return false;
	}

	// To find orientation of ordered triplet (p, q, r).
	// The function returns following values
	// 0 --> p, q and r are colinear
	// 1 --> Clockwise
	// 2 --> Counterclockwise
	template<typename T> int orientation(point_t<T> p, point_t<T> q, point_t<T> r){
		T val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
		if (val == 0) return 0;  // colinear
		return (val > 0) ? 1 : 2; // clock or counterclock wise
	}

	// The main function that returns true if line segment 'p1q1'
	// and 'p2q2' intersect.
	template<typename T> bool doIntersect(point_t<T> p1, point_t<T> q1, point_t<T> p2, point_t<T> q2, bool allowCollinearity = false){

		// Find the four orientations needed for general and
		// special cases
		int o1 = orientation(p1, q1, p2);
		int o2 = orientation(p1, q1, q2);
		int o3 = orientation(p2, q2, p1);
		int o4 = orientation(p2, q2, q1);

		// General case
		if (o1 != o2 && o3 != o4) return true;

		// Special Cases
		if (!allowCollinearity){
		
			// p1, q1 and p2 are colinear and p2 lies on segment p1q1
			if (o1 == 0 && onSegment(p1, p2, q1)) return true;

			// p1, q1 and p2 are colinear and q2 lies on segment p1q1
			if (o2 == 0 && onSegment(p1, q2, q1)) return true;

			// p2, q2 and p1 are colinear and p1 lies on segment p2q2
			if (o3 == 0 && onSegment(p2, p1, q2)) return true;

			// p2, q2 and q1 are colinear and q1 lies on segment p2q2
			if (o4 == 0 && onSegment(p2, q1, q2)) return true;
		}
		return false; // Doesn't fall in any of the above cases
	}
}

// Wrapper that uses direct coordinates, instead of polluting the namespace with a generic point struct
template <typename T> bool doIntersect(T p1x, T p1y, T q1x, T q1y, T p2x, T p2y, T q2x, T q2y, bool allowCollinearity){
	point_t<T> p1(p1x, p1y), p2(p2x, p2y), q1(q1x, q1y), q2(q2x, q2y);
	return doIntersect<T>(p1, q1, p2, q2, allowCollinearity);
}

template bool doIntersect<size_t>(size_t p1x, size_t p1y, size_t q1x, size_t q1y, size_t p2x, size_t p2y, size_t q2x, size_t q2y, bool allowCollinearity);
template bool doIntersect<int>(int p1x, int p1y, int q1x, int q1y, int p2x, int p2y, int q2x, int q2y, bool allowCollinearity);
template bool doIntersect<float>(float p1x, float p1y, float q1x, float q1y, float p2x, float p2y, float q2x, float q2y, bool allowCollinearity);
template bool doIntersect<double>(double p1x, double p1y, double q1x, double q1y, double p2x, double p2y, double q2x, double q2y, bool allowCollinearity);
