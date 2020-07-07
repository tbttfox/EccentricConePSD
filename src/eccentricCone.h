#include <cmath>
#include <maya/MPoint.h>

typedef double Float;

bool linePlaneIntersect(const MMatrix &planeMatrix, const MPoint &pt1, const MPoint &pt2, MPoint &ret){
    MMatrix mInv = planeMatrix.inverse();

    MPoint p1 = pt1 * mInv;
    MPoint p2 = pt2 * mInv;

    MVector d = p1 - p2;

    Float div = p1[0] / d[0];
    MVector offset = d * div;

    ret = (p1 - offset)  * planeMatrix;
    return div < 0; // Return whether we're negative
    // If we're negative, we can just return 0
}

bool vecPlaneIntersect(const MMatrix &planeMatrix, const MPoint &pt1, MPoint &ret){
    MMatrix mInv = planeMatrix.inverse();

    MPoint p1 = pt1 * mInv;
    MPoint p2 = MPoint() * mInv;

    MVector d = p1 - p2;

    Float div = p1[0] / d[0];
    MVector offset = d * div;

    ret = (p1 - offset)  * planeMatrix;
    return div < 0; // Return whether we're negative
    // If we're negative, we can just return 0
}



MStatus intersectLineWithUnitCircle(const MPoint &pt0, const MPoint &pt1, MPoint &out){
    Float x0 = pt0[0];
    Float x1 = pt1[0];
    Float y0 = pt0[1];
    Float y1 = pt1[1];

    Float dx = x1 - x0;
    Float dy = y1 - y0;

    Float a = dx*dx + dy*dy;
    Float b = 2*dx*x0  + 2*dy*y0;
    Float c = 2 * (x0*x0 - y0*y0 - 1);

    Float disc = b*b - 2*a*c;
    if (disc < 0){
        out = MPoint();
        return MStatus::kFailure;
    }
    disc = sqrt(disc);

    // use a more stable variation of the quadratic formula
    Float t1 = c / (-b + disc);
    Float t2 = c / (-b - disc);

    Float ret = ((0 <= t1) && (t1 <= 1)) ? t1 : t2;
    Float retX = dx*ret + x0;
    Float retY = dy*ret + y0;
    out = MPoint(retX, retY);
    return MStatus::kSuccess;
}

Float eccentricEllipseFalloff(const MPoint &vertPos, const MMatrix &innerMat, const MMatrix &outerMat){
    MMatrix innerMatInv = innerMat.inverse();
    MMatrix outerMatInv = outerMat.inverse();

    MVector vertInner = vertPos * innerMatInv;
    MVector vertOuter = vertPos * outerMatInv;

    // If we're inside the inner circle, we're at 100% influence, so return
    Float inLen2 = vertInner.x * vertInner.x + vertInner.y * vertInner.y + vertInner.z * vertInner.z;
    if (inLen < 1.0) return 1.0;

    // If we're outside the outer circle, we're at 0% influence, so return
    Float outLen2 = vertOuter.x * vertOuter.x + vertOuter.y * vertOuter.y + vertOuter.z * vertOuter.z;
    if (outLen > 1.0) return 0.0;

    // get the center of the inner circle in the space of the outer circle
    MPoint innerMatTran(innerMat[3]);
    MPoint insideTran = innerMatTran * outerMatInv;

    // Draw a line between that found center, and the point
    // Then intersect it with the outer circle
    MPoint circInt;
    MStatus stat = intersectLineWithUnitCircle(vertOuter, insideTran, circInt);
    if (!stat)return 0.0;

    // And put it back into worldspace
    MPoint res = circInt * outerMat;

    // Project the vertex onto the inner circle and put it back into worldspace
    MVector vertInnerN = (MPoint)(vertInner.normal()) * innerMat;

    // Now get the ratio of distances between the two
    // intersections, and the vertPos and one of the intersections

    Float dist1 = (vertInnerN - vertPos).length();
    Float dist2 = (vertInnerN - res).length();
    return dist1 / dist2;
}






