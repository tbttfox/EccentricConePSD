#include <cmath>
#include <algorithm>
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <maya/MVector.h>

typedef double Float;

bool linePlaneIntersect(const MMatrix &planeMatrix, const MPoint &pt1, const MPoint &pt2, MPoint &ret, UINT normalAxis){
    MMatrix mInv = planeMatrix.inverse();

    MPoint p1 = pt1 * mInv;
    MPoint p2 = pt2 * mInv;

    MVector d = p1 - p2;

    Float div = p1[normalAxis] / d[normalAxis];
    MVector offset = d * div;

    ret = (p1 - offset)  * planeMatrix;
    return div < 0.0; // Return whether we're negative
    // If we're negative, we can just return 0
}

bool vecPlaneIntersect(const MMatrix &planeMatrix, const MPoint &pt1, MPoint &ret, UINT normalAxis){
	return linePlaneIntersect(planeMatrix, pt1, MPoint(), ret, normalAxis);
}


MStatus intersectLineWithUnitCircle(Float x2, Float y2, Float x1, Float y1, Float &ox, Float &oy) {
	Float vx = x1 - x2;
	Float vy = y1 - y2;
	Float v2 = 2 * (vx*vx + vy*vy);
	Float aa = 2 * (x1*vx + y1*vy);
	Float bb = 2 * v2 * ((x1*x1 + y1*y1) - 1);
	Float disc = aa * aa - bb;
	if (disc < 0) return MStatus::kFailure;
	Float t = (sqrt(disc) - aa) / v2;
	ox = t * vx + x1;
	oy = t * vy + y1;
	return MStatus::kSuccess;
}


MStatus intersectLineWithUnitCircle(const MPoint &pt0, const MPoint &pt1, MPoint &out, UINT normalAxis){
	UINT c1 = 0, c2 = 1;
	switch (normalAxis) {
	case 0:
		c1 = 1;
	case 1:
		c2 = 2;
	}

	Float retX, retY;
	MStatus ret = intersectLineWithUnitCircle(pt0[c1], pt0[c2], pt1[c1], pt1[c2], retX, retY);
	if (!ret) return ret;
	Float retZ = 0;

	switch (normalAxis) {
	case 1:
		std::swap(retY, retZ);
		break;
	case 0:
		std::swap(retY, retZ);
		std::swap(retX, retY);
		break;
	}
    out = MPoint(retX, retY, retZ);
	return ret;
}



Float eccentricEllipseFalloff(const MPoint &vertPos, const MMatrix &innerMat, const MMatrix &outerMat){
    MMatrix innerMatInv = innerMat.inverse();
    MMatrix outerMatInv = outerMat.inverse();

    MVector vertInner = vertPos * innerMatInv;
    MVector vertOuter = vertPos * outerMatInv;

    // If we're inside the inner circle, we're at 100% influence, so return
    Float inLen2 = vertInner.x * vertInner.x + vertInner.y * vertInner.y + vertInner.z * vertInner.z;
    if (inLen2 < 1.0) return 1.0;

    // If we're outside the outer circle, we're at 0% influence, so return
    Float outLen2 = vertOuter.x * vertOuter.x + vertOuter.y * vertOuter.y + vertOuter.z * vertOuter.z;
    if (outLen2 > 1.0) return 0.0;

    // get the center of the inner circle in the space of the outer circle
    MPoint innerMatTran(innerMat[3]);
    MPoint insideTran = innerMatTran * outerMatInv;

    // Draw a line between that found center, and the point
    // Then intersect it with the outer circle
    MPoint circInt;
	//MStatus stat = intersectLineWithUnitCircle(vertOuter, insideTran, circInt, 1);
    MStatus stat = intersectLineWithUnitCircle(insideTran, vertOuter, circInt, 1);
    if (!stat)return 0.0;

    // And put it back into worldspace
    MPoint res = circInt * outerMat;

    // Project the vertex onto the inner circle and put it back into worldspace
    MPoint vertInnerN = (MPoint)(vertInner.normal()) * innerMat;

    // Now get the ratio of distances between the two
    // intersections, and the vertPos and one of the intersections

    Float dist1 = (vertInnerN - vertPos).length();
    Float dist2 = (vertInnerN - res).length();
    return 1.0 - (dist1 / dist2);
}
