#include "eccentricCone.h"
#include "eccentricConePSD.h"
#include <maya/MMatrix.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MTypeID.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MRampAttribute.h>
#include <maya/MFloatArray.h>
#include <maya/MIntArray.h>

MTypeId EccentricConePSDNode::id(0x00122709);
MString EccentricConePSDNode::kName = "eccentricConePSD";

MObject EccentricConePSDNode::aOuterMatrix;
MObject EccentricConePSDNode::aInnerMatrix;
MObject EccentricConePSDNode::aReferenceInvMatrix;
MObject EccentricConePSDNode::aSamplePoint;
MObject EccentricConePSDNode::aSamplePointX;
MObject EccentricConePSDNode::aSamplePointY;
MObject EccentricConePSDNode::aSamplePointZ;
MObject EccentricConePSDNode::aOutputValue;

EccentricConePSDNode::EccentricConePSDNode() {}
EccentricConePSDNode::~EccentricConePSDNode() {}

void* EccentricConePSDNode::creator(){
    return new EccentricConePSDNode();
}

MStatus EccentricConePSDNode::initialize(){
	MStatus stat;
	MFnUnitAttribute fnUnit;
	MFnNumericAttribute fnNum;
	MFnMatrixAttribute fnMat;
    MRampAttribute rAttr;

	// Input Attributes
	aOuterMatrix = fnMat.create("outerMatrix", "om", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
	stat = addAttribute(aOuterMatrix);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

	aInnerMatrix = fnMat.create("innerMatrix", "im", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
	stat = addAttribute(aInnerMatrix);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

	aReferenceInvMatrix = fnMat.create("referenceInverseMatrix", "rim", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
	stat = addAttribute(aReferenceInvMatrix);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

	aSamplePointX = fnUnit.create("pointX", "px", MFnUnitAttribute::kDistance, 0.0, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
	aSamplePointY = fnUnit.create("pointY", "py", MFnUnitAttribute::kDistance, 0.0, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
	aSamplePointZ = fnUnit.create("pointZ", "pz", MFnUnitAttribute::kDistance, 0.0, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
	aSamplePoint = fnNum.create("point", "p", aSamplePointX, aSamplePointY, aSamplePointZ, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
    stat = addAttribute(aSamplePoint);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

    // Output Attribute
    aOutputValue = fnNum.create("value", "v", MFnNumericData::kFloat, 0.0, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
    stat = addAttribute(aOutputValue);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

    // Affectations
    attributeAffects(aOuterMatrix, aOutputValue);
    attributeAffects(aInnerMatrix, aOutputValue);
    attributeAffects(aReferenceInvMatrix, aOutputValue);
    attributeAffects(aSamplePoint, aOutputValue);
    attributeAffects(aSamplePointX, aOutputValue);
    attributeAffects(aSamplePointY, aOutputValue);
    attributeAffects(aSamplePointZ, aOutputValue);

    return MStatus::kSuccess;
}



MStatus EccentricConePSDNode::compute(const MPlug& plug, MDataBlock& block){
    MStatus stat;

    if (plug != aOutputValue) return MStatus::kUnknownParameter;

    MMatrix outerMat = block.inputValue(aOuterMatrix).asMatrix();
    MMatrix innerMat = block.inputValue(aInnerMatrix).asMatrix();
    MMatrix refInvMat = block.inputValue(aReferenceInvMatrix).asMatrix();

    MDataHandle hPoint = block.inputValue(aSamplePoint);
    MPoint pt;
    pt.x = hPoint.child(aSamplePointX).asDouble();
    pt.y = hPoint.child(aSamplePointY).asDouble();
    pt.z = hPoint.child(aSamplePointZ).asDouble();

    outerMat = outerMat * refInvMat;
    innerMat = innerMat * refInvMat;
    pt = pt * refInvMat;

    // Calculate the value
	float outValue = 0.0;
    MPoint sect;
    bool wrongSide = vecPlaneIntersect(outerMat, pt, sect, 1);
    if (!wrongSide){
        outValue = eccentricEllipseFalloff(sect, innerMat, outerMat);
    }

    // set the output plug
    MDataHandle outHandle = block.outputValue(aOutputValue);
    outHandle.set(outValue);
    block.setClean(plug);
    return MStatus::kSuccess;
}
