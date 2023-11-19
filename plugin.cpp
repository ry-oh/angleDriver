#include "angleDriver.h"
#include <maya/MFnPlugin.h>


PLUGIN_EXPORT MStatus initializePlugin(MObject pluginObj)
{
    MStatus status;

    static const char* VERSION = "1.0.0.20220720";
    static const char* VENDER  = "Ry.O";
    MFnPlugin pluginFn(pluginObj, VENDER, VERSION, "Any");


    status = pluginFn.registerNode("angleDriver",
                                    AngleDriver::id,
                                    AngleDriver::creator,
                                    AngleDriver::initialize);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return status;
}

PLUGIN_EXPORT MStatus uninitializePlugin(MObject pluginObj){
    MStatus status;
    MFnPlugin pluginFn(pluginObj);
    status = pluginFn.deregisterNode(AngleDriver::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return status;
}