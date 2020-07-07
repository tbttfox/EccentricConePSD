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





MStatus intersectLineWithUnitCircle3(Float x1, Float y1, Float x2, Float y2, Float &ox, Float &oy) {
	Float dx = x2 - x1;
	Float dy = y2 - y1;

	Float dpd = dx*x1 + dy*y1;
	Float dlen2 = dx*dx + dy*dy;
	Float plen2 = x1*x1 + y1*y1;

	Float disc = dpd*dpd - dlen2*(plen2 - 1);
	if (disc < 0) return;
	disc = sqrt(disc);

	Float t1 = (-dpd + disc) / dlen2;
	Float t2 = (-dpd - disc) / dlen2;

    Float ret = ((0 <= t1) && (t1 <= 1)) ? t1 : t2;
    ox = dx*ret + x1;
    oy = dy*ret + y1;
}



MStatus intersectLineWithUnitCircle3(const MPoint &pt0, const MPoint &pt1, MPoint &out, UINT normalAxis){
	UINT c1 = 0, c2 = 1;
	switch (normalAxis) {
	case 0:
		c1 = 1;
	case 1:
		c2 = 2;
	}

	Float retX, retY;
	intersectLineWithUnitCircle3(pt0[c1], pt0[c2], pt1[c1], pt1[c2], retX, retY);
	Float retZ = 0;

	switch (normalAxis) {
	case 1:
		std::swap(retY, retZ);
	case 0:
		std::swap(retX, retY);
	}
    out = MPoint(retX, retY, retZ);
    return MStatus::kSuccess;
}







MStatus intersectLineWithUnitCircle(const MPoint &pt0, const MPoint &pt1, MPoint &out, UINT normalAxis){

	// Dropthrough is intended
	UINT c1 = 0, c2 = 1;
	switch (normalAxis) {
	case 0:
		c1 = 1;
	case 1:
		c2 = 2;
	}

    Float x0 = pt0[c1];
    Float x1 = pt1[c1];
    Float y0 = pt0[c2];
    Float y1 = pt1[c2];

    Float dx = x1 - x0;
    Float dy = y1 - y0;

    Float a = dx*dx + dy*dy;
    Float b = 2*dx*x0 + 2*dy*y0;
    Float c = x0*x0 - y0*y0 - 1;

    Float disc = b*b - 4*a*c;
    if (disc < 0){
        out = MPoint();
        return MStatus::kFailure;
    }
    disc = sqrt(disc);

    // use a more stable variation of the quadratic formula
    Float t1 = 2 * c / (-b + disc);
    Float t2 = 2 * c / (-b - disc);

    Float ret = ((0 <= t1) && (t1 <= 1)) ? t1 : t2;
    Float retX = dx*ret + x0;
    Float retY = dy*ret + y0;
	Float retZ = 0;
	switch (normalAxis) {
	case 1:
		std::swap(retY, retZ);
	case 0:
		std::swap(retX, retY);
	}
    out = MPoint(retX, retY, retZ);
    return MStatus::kSuccess;
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






