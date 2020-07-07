#pragma once

#include <maya/MArrayDataHandle.h>
#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MDataBlock.h>


class EccentricConePSDNode : public MPxNode {
    public:
        EccentricConePSDNode();
        virtual ~EccentricConePSDNode();
        static void* creator();
        static MStatus initialize();

        virtual MStatus compute(const MPlug& plug, MDataBlock& data) override;

        static MTypeId id;
        static MString kName;

        static MObject aOuterMatrix;
        static MObject aInnerMatrix;
        static MObject aReferenceInvMatrix;
        static MObject aSamplePoint;
        static MObject aSamplePointX;
        static MObject aSamplePointY;
        static MObject aSamplePointZ;
        static MObject aFalloffRamp;
        static MObject aOutputValue;
};

