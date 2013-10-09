#include <maya/MPxCommand.h>
#include <maya/MFnPlugin.h>
#include <maya/MIOStream.h>
#include <maya/MString.h>
#include <maya/MArgList.h>
#include <maya/MGlobal.h>
#include <maya/MSimple.h>
#include <maya/MDoubleArray.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MDGModifier.h>
#include <maya/MPlugArray.h>
#include <maya/MVector.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MStringArray.h>
#include <list>

#include "LSTetgenNode.h"
#include "LSSolverNode.h"
#include "LSComponentNode.h"

MStatus initializePlugin( MObject obj )
{
    MStatus   status = MStatus::kSuccess;
    MFnPlugin plugin( obj, "LocoSoft", "1.0", "Any");

    // Register Command
	MGlobal::executeCommand("global string $gMainWindow;\n setParent $gMainWindow;");
	MGlobal::executeCommand("menu -l \"Loco Soft\" -p MayaWindow -to 1 -aob true TetgenMenu;");
	MGlobal::executeCommand(MString("menuItem -l \"Tetrahedralize\" -echoCommand true -command \"source \\\"") +
									plugin.loadPath()+ "/node.mel" +"\\\"\" TetgenMenuItem1;" );
	MGlobal::executeCommand(MString("menuItem -l \"Simulate Tet Mesh\" -echoCommand true -command \"source \\\"") +
									plugin.loadPath()+ "/solver.mel" +"\\\"\" TetgenMenuItem2;" );
	MGlobal::executeCommand(MString("menuItem -l \"Loco Soft Simulate\" -echoCommand true -command \"source \\\"") +
									plugin.loadPath()+ "/sim.mel" +"\\\"\" TetgenMenuItem3;" );
	MGlobal::executeCommand(MString("menuItem -l \"Loco Soft Constrained Vertices\" -echoCommand true -command \"source \\\"") + 
									plugin.loadPath()+ "/constrainedVert.mel" +"\\\"\" TetgenMenuItem4;" );
	MGlobal::executeCommand(MString("menuItem -l \"Loco Soft Force Vertices\" -echoCommand true -command \"source \\\"") + 
									plugin.loadPath()+ "/forceVert.mel" +"\\\"\" TetgenMenuItem5;" );
	MGlobal::executeCommand(MString("menuItem -l \"Bake Geometry\" -echoCommand true -command \"source \\\"") +
									plugin.loadPath()+ "/bake.mel" +"\\\"\" TetgenMenuItem6;" );
	MGlobal::executeCommand(MString("menuItem -l \"Load Geometry\" -echoCommand true -command \"source \\\"") +
									plugin.loadPath()+ "/loadAbc.mel" +"\\\"\" TetgenMenuItem7;" );


	status = plugin.registerNode("LSTetgenNode",LSTetgenNode::id,LSTetgenNode::creator, LSTetgenNode::initialize);
    status = plugin.registerNode("LSSolverNode",LSSolverNode::id,LSSolverNode::creator, LSSolverNode::initialize);
	status = plugin.registerNode("LSComponentNode", LSComponentNode::id, LSComponentNode::creator, LSComponentNode::initialize);


    return status;
}

MStatus uninitializePlugin( MObject obj)
{
    MStatus   status = MStatus::kSuccess;
    MFnPlugin plugin( obj );

	MGlobal::executeCommand("deleteUI -m TetgenMenu");
	status = plugin.deregisterNode(LSTetgenNode::id);
	status = plugin.deregisterNode(LSSolverNode::id);
	status = plugin.deregisterNode(LSComponentNode::id);
    status = plugin.deregisterCommand( "TetgenCmd" );
    if (!status) {
	    status.perror("deregisterCommand");
	    return status;
    }

    return status;
}


