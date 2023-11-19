#ifndef _angleDriver
#define _angleDriver

#include "common.h"


MTypeId AngleDriver::id(0x12345678);
MString AngleDriver::name("angleDriver");
MObject AngleDriver::rotate;
    MObject AngleDriver::rotateX;
    MObject AngleDriver::rotateY;
    MObject AngleDriver::rotateZ;

MObject AngleDriver::rotateOrder;

MObject AngleDriver::jointOrient;
    MObject AngleDriver::jointOrientX;
    MObject AngleDriver::jointOrientY;
    MObject AngleDriver::jointOrientZ;

MObject AngleDriver::outAngle;
    MObject AngleDriver::outRoll;
    MObject AngleDriver::outPitch;
    MObject AngleDriver::outYaw;

MObject AngleDriver::outNormAngle;
    MObject AngleDriver::outNormRoll;
    MObject AngleDriver::outNormPitch;
    MObject AngleDriver::outNormYaw;

MObject AngleDriver::outLinear;
    MObject AngleDriver::outPitchLinear;
    MObject AngleDriver::outYawLinear;

MObject AngleDriver::method;
MObject AngleDriver::reverseOrder;
MObject AngleDriver::weight;
MObject AngleDriver::interpType;

MObject AngleDriver::parentMatrix;


// define constants
const double AngleDriver::_2PI = 2.0 * M_PI;

const MVector AngleDriver::X_vec(1, 0, 0);
const MVector AngleDriver::Y_vec(0, 1, 0);
const MVector AngleDriver::Z_vec(0, 0, 1);

const int RotateOrderXYZ = MEulerRotation::RotationOrder::kXYZ;
const int RotateOrderYZX = MEulerRotation::RotationOrder::kYZX;
const int RotateOrderZXY = MEulerRotation::RotationOrder::kZXY;
const int RotateOrderXZY = MEulerRotation::RotationOrder::kXZY;
const int RotateOrderYXZ = MEulerRotation::RotationOrder::kYXZ;
const int RotateOrderZYX = MEulerRotation::RotationOrder::kZYX;


// constructor
AngleDriver::AngleDriver() {}

// destructor
AngleDriver::~AngleDriver() {}

// instance creator
void *AngleDriver::creator(){return new AngleDriver();}

// initialization.
MStatus AngleDriver::initialize(){
    MStatus stat = MS::kSuccess;

    MFnNumericAttribute nAttr;
    MFnUnitAttribute uAttr;
    MFnEnumAttribute eAttr;
    MFnMatrixAttribute nMAttr;

    // Method
    method = eAttr.create("method", "met");
    eAttr.addField("Stereographic Projection", 0);
    eAttr.addField("Spherical Linear Interpolation", 1);
    eAttr.addField("Exponential Map", 2);
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(method));

    // reverse order
    reverseOrder = nAttr.create("reverseOrder", "ror", MFnNumericData::kBoolean, false);
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(reverseOrder));

    // weight
    weight = nAttr.create("weight", "w", MFnNumericData::kDouble, 1.);
    nAttr.setMin(0.);
    nAttr.setMax(1.);
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(weight));

    // interp type for slerp
    interpType = eAttr.create("interpType", "it");
    eAttr.addField("Shortest", 0);
    eAttr.addField("Longest", 1);
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(interpType));

    // in rotate
    rotateX = uAttr.create("rotateX", "rx", MFnUnitAttribute::kAngle, 0.);
    uAttr.setReadable(true);
    uAttr.setStorable(false);

    rotateY = uAttr.create("rotateY", "ry", MFnUnitAttribute::kAngle, 0.);
    uAttr.setReadable(true);
    uAttr.setStorable(false);

    rotateZ = uAttr.create("rotateZ", "rz", MFnUnitAttribute::kAngle, 0.);
    uAttr.setReadable(true);
    uAttr.setStorable(false);

    rotate = nAttr.create("rotate", "r", rotateX, rotateY, rotateZ);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setReadable(false);
    nAttr.setWritable(true);
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(rotate));

    // in rotate order
    rotateOrder = eAttr.create("rotateOrder", "ro", MEulerRotation::RotationOrder::kXYZ);
        eAttr.addField("xyz", RotateOrderXYZ);
        eAttr.addField("yzx", RotateOrderYZX);
        eAttr.addField("zxy", RotateOrderZXY);
        eAttr.addField("xzy", RotateOrderXZY);
        eAttr.addField("yxz", RotateOrderYXZ);
        eAttr.addField("zyx", RotateOrderZYX);
    eAttr.setWritable(true);
    addAttribute(rotateOrder);

    // in joint orient
    jointOrientX = uAttr.create("jointOrientX", "aox", MFnUnitAttribute::kAngle, 0.);
    uAttr.setReadable(true);
    uAttr.setStorable(false);

    jointOrientY = uAttr.create("jointOrientY", "aoy", MFnUnitAttribute::kAngle, 0.);
    uAttr.setReadable(true);
    uAttr.setStorable(false);

    jointOrientZ = uAttr.create("jointOrientZ", "aoz", MFnUnitAttribute::kAngle, 0.);
    uAttr.setReadable(true);
    uAttr.setStorable(false);

    jointOrient = nAttr.create("jointOrient", "ao", jointOrientX, jointOrientY, jointOrientZ);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setReadable(false);
    nAttr.setWritable(true);
    addAttribute(jointOrient);


    // out angle
    outRoll  = uAttr.create("roll", "oar", MFnUnitAttribute::kAngle, 0.);
    outPitch = uAttr.create("pitch", "oap", MFnUnitAttribute::kAngle, 0.);
    outYaw   = uAttr.create("yaw", "oay", MFnUnitAttribute::kAngle, 0.);

    outAngle = nAttr.create("outAngle", "oa", outRoll, outPitch, outYaw);
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    nAttr.setChannelBox(true);
    addAttribute(outAngle);

    // in parent matrix
    parentMatrix = nMAttr.create("inParentMatrix", "ipm", MFnMatrixAttribute::kDouble);
    nMAttr.setStorable(false);
    nMAttr.setKeyable(false);

    addAttribute(parentMatrix);

    // attribute affects
    attributeAffects(method, outAngle);
    attributeAffects(reverseOrder, outAngle);
    attributeAffects(rotate, outAngle);
    attributeAffects(rotateOrder, outAngle);
    attributeAffects(jointOrient, outAngle);
    attributeAffects(weight, outAngle);
    attributeAffects(interpType, outAngle);
    attributeAffects(parentMatrix, outAngle);

    return stat;
}

MStatus AngleDriver::setDependentsDirty(const MPlug& plug, MPlugArray& plugArray){
    MPlug dirtyPlug = plug.isChild() ? plug.parent() : plug;

    return MPxNode::setDependentsDirty(plug, plugArray);
}

#if MAYA_API_VERSION >= 201600
AngleDriver::SchedulingType AngleDriver::schedulingType() const { return MPxNode::kParallel; }
#endif


double AngleDriver::boundAngle( double x){
    if (x > M_PI)
    {
        x = x - _2PI;
    }
    else if( x < -M_PI)
    {
            x = x + _2PI;
    }
    return x;
}

double AngleDriver::normalizeAngle(double x){
    if (x == 0.)
    {
        return 0.;
    }
    else
    {
        return (x - -M_PI) / ( _2PI);
    }
}


void AngleDriver::slerp(MQuaternion q1, MQuaternion q2, MQuaternion &qr, double lambda)
{
    double dotProd = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
    double theta, st, sut, sout, coeff1, coeff2;

    lambda = lambda / 2.0;
    theta = acos(dotProd);
    if (theta < 0.0)
    {
        theta = -theta;
    }

    st     = sin(theta);
    sut    = sin(lambda * theta);
    sout   = sin((1 - lambda) * theta);
    coeff1 = sout / st;
    coeff2 = sut  / st;

    qr.x = coeff1 * q1.x + coeff2 * q2.x;
    qr.y = coeff1 * q1.y + coeff2 * q2.y;
    qr.z = coeff1 * q1.z + coeff2 * q2.z;
    qr.w = coeff1 * q1.w + coeff2 * q2.w;

    qr.normalizeIt();
}


MVector AngleDriver::toTwistBend( MQuaternion quat){
    MVector qvec = X_vec.rotateBy(quat);
    MQuaternion bendQ = MQuaternion(X_vec, qvec);
    MQuaternion twistQ = quat * bendQ.inverse();

    double b = (X_vec * qvec) + 1.;
    double x = (atan2(twistQ[0], twistQ[3]) * 2.);
    x = boundAngle(x);

    double z = (atan2(Z_vec * qvec, b) * -2.);
    double y = (atan2(Y_vec * qvec, b) * 2.);

    MVector result = MVector(x, y, z);
    return result;
}


MStatus AngleDriver::compute(const MPlug& plug, MDataBlock& block)
{
    MPlug evalPlug = plug.isChild() ? plug.parent() : plug;

    if (evalPlug == outAngle || evalPlug == outNormAngle)
    {
        _decomposeRotate(block);
    }
    else
    {
        return MS::kUnknownParameter;
    }


    return MS::kSuccess;
}

MStatus AngleDriver::_decomposeRotate(MDataBlock& block)
{
    MVector rhv;

    // offset parent matrix
    MTransformationMatrix mmult = block.inputValue(parentMatrix).asMatrix();
    double4 wQuat;
    mmult.getRotationQuaternion(wQuat[0], wQuat[1], wQuat[2], wQuat[3]);

    // offset joint orient
    MQuaternion quat = MEulerRotation(block.inputValue(jointOrient).asDouble3() ).asQuaternion();
    MQuaternion oriQ = quat.inverse();

    quat *= wQuat;

    // consider rotate order
    MDataHandle rotateOrderHandle = block.inputValue(rotateOrder);
    const MEulerRotation::RotationOrder rotationOrder = MEulerRotation::RotationOrder(rotateOrderHandle.asShort());

    quat *= MEulerRotation(block.inputValue(rotate).asDouble3(), rotationOrder ).asQuaternion();
    quat *= oriQ;

    // option attribute for slerp
    double wgt = block.inputValue(weight).asDouble();
    short it = block.inputValue(interpType).asShort();
    if (it == 1)
    {
        it = -1;
    }

    bool reverse = block.inputValue(reverseOrder).asBool();
    if (reverse)
    {
        quat = quat.inverse();
    }

    int methodType = block.inputValue(method).asShort();
    if (methodType == 2)// Exponential Map
    {
        quat = quat.log();
        rhv = MVector(boundAngle(quat[0] * 2.),
                      boundAngle(quat[1] * 2.),
                      boundAngle(quat[2] * 2.));
    }
    else if (methodType == 1)// Spherical Linear Interpolation
    {
        quat  = slerp(oriQ, quat, wgt, it);
        rhv = MVector(boundAngle(quat[0] * 2.),
                      boundAngle(quat[1] * 2.),
                      boundAngle(quat[2] * 2.));
    }
    else// Stereographic Projection
    {
        rhv = toTwistBend(quat);
    }

    if (reverse)
    {
        rhv = MVector(-rhv[0], -rhv[1], -rhv[2]);
    }
    block.outputValue(outAngle).setMVector(rhv);

    return MS::kSuccess;
}
#endif