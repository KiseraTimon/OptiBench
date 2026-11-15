#pragma once

/*
 * Geometry module umbrella header: include this to get every geometry routine.
 *
 *      Primitives.h    tolerances, orientation, closest points, segment intersection, Aabb
 *      PolygonOps.h    area, centroid, winding, convexity, simplicity, point location,
 *                      outward normals, convex hull, triangulation, transforms
 *      Intersect.h     ray vs box, ray vs polygon (concave included), convex-region spans
 *      Overlap.h       SAT with minimum translation vector, concave polygon and disc overlap
 *      Lens.h          thick biconvex / plano-convex lens shape with lensmaker focal lengths
 *
 * Namespace: ob::geom. See docs/geometry.md.
 */

#include "optibench/geometry/Intersect.h"
#include "optibench/geometry/Lens.h"
#include "optibench/geometry/Overlap.h"
#include "optibench/geometry/PolygonOps.h"
#include "optibench/geometry/Primitives.h"
