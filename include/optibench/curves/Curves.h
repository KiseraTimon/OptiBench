#pragma once

/*
 * Special Notes: Curves Module (umbrella header)
 *
 * Includes the whole curves module. Prefer the individual headers in library code; this one
 * is a convenience for demos and quick experiments. See docs/curves.md for the design.
 *
 * It does not include clip_fill.h or Polygon.h, so it can be combined with either of them.
 */

#include "optibench/curves/BSpline.h"
#include "optibench/curves/Bezier.h"
#include "optibench/curves/CatmullRom.h"
#include "optibench/curves/Composite.h"
#include "optibench/curves/CubicSpline.h"
#include "optibench/curves/CurveEditor.h"
#include "optibench/curves/CurveMath.h"
#include "optibench/curves/CurveRender.h"
#include "optibench/curves/Hermite.h"
#include "optibench/curves/Nurbs.h"
#include "optibench/curves/OpticalCurve.h"
#include "optibench/curves/RayPath.h"
