#ifndef COMMON_H
#define COMMON_H

#include <math.h>

#include <maya/MPxNode.h>

#include <maya/MPlug.h>

#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnGenericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnEnumAttribute.h>

#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MTransformationMatrix.h>

#include <maya/MTimer.h>
#include <maya/MThreadUtils.h>

#include <maya/MEvaluationNode.h>
#include <maya/MEvaluationManager.h>


class AngleDriver : public MPxNode
{
public:
    // class method declarations:

    static const double _2PI;

    static const MVector X_vec;
    static const MVector Y_vec;
    static const MVector Z_vec;

    AngleDriver();

    virtual ~AngleDriver();

    static void* creator();
    static MStatus initialize();


    MStatus compute(const MPlug &plug, MDataBlock &data);

    virtual MStatus setDependentsDirty(const MPlug& plug, MPlugArray& plugArray);

#if MAYA_API_VERSION >= 201600
    virtual SchedulingType schedulingType() const;
#endif

    MStatus _decomposeRotate(MDataBlock&);

    double boundAngle(double x);
    double normalizeAngle(double x);

    MVector toTwistBend(MQuaternion quad);
    void slerp(MQuaternion q1, MQuaternion q2, MQuaternion &qr, double lambda);


    static MTypeId id;
    static MString name;

    static MObject nAttr;
    static MObject uAttr;
    static MObject eAttr;

    static MObject method;
    static MObject reverseOrder;
    static MObject weight;
    static MObject interpType;
    static MObject outAngleUnitType;

    static MObject rotate;
        static MObject rotateX;
        static MObject rotateY;
        static MObject rotateZ;

    static MObject rotateOrder;

    static MObject jointOrient;
        static MObject jointOrientX;
        static MObject jointOrientY;
        static MObject jointOrientZ;

    static MObject outAngle;
        static MObject outRoll;
        static MObject outPitch;
        static MObject outYaw;

    static MObject outNormAngle;
        static MObject outNormRoll;
        static MObject outNormPitch;
        static MObject outNormYaw;

    static MObject outLinear;
        static MObject outPitchLinear;
        static MObject outYawLinear;

    static MObject parentMatrix;

};

#endif