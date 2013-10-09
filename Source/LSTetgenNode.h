#ifndef LSYSTEMNODE_H
#define LSYSTEMNODE_H

#include <maya/MPxCommand.h>
#include <maya/MPxNode.h>
#include <maya/MDGModifier.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnStringData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MTime.h>
#include <maya/MGlobal.h>
#include <maya/MPointArray.h>
#include <maya/MFnPlugin.h>
#include <maya/MFloatPoint.h>
#include <maya/MFloatPointArray.h>
#include <maya/MIOStream.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MArrayDataHandle.h>

#include <fstream>
#include <string>
#include <sstream>

#include "tetgen.h"

using std::string;

// Given an input mesh, LSTetgenNode will tetrahedralize and output the result, which can then be used by LSSolverNode
// There are two ways this can be done:
//		1. 3 Files can be written to the local drive (.ele, .node, and .face). These files are then parsed by LSSolverNode to obtain the tetrahedralized mesh information
//		2. An array of doubles containing vertex coordinates and an array of ints containing the indices of the tetrahedra can be passed to LSSolverNode.

class LSTetgenNode: public MPxNode
{
public:
	LSTetgenNode() {};
	virtual ~LSTetgenNode() {};
	virtual MStatus compute(const MPlug& plug, MDataBlock& data);
	static void* creator();
	static MStatus initialize();

public:
	static MObject worldMatrix;
	static MObject inputMesh;
	static MObject outputMesh;
	static MObject radiusEdgeRatio;
	static MObject volumeConstraint;
	static MObject outputFilePath;
	static MObject outputVerticesArray;
	static MObject outputElementsArray;
	static MTypeId id;
	static MObject saveFile;
private: 
	tetgenio in, out;

	void toTetMesh(MObject &inputMesh, MObject &outputMesh, MMatrix &wmat, double radEdgeRatio, double volConstraint, string outputPath,bool saveFile);
	void makeDummyMesh(MObject &out, MStatus &stat);

	// Given the inputMesh, extract the vertices, faces, and store in tetgenio
	MStatus toTetgenIO(MObject &inputMesh, MMatrix &wmat); // converts inputMesh into tetgenio format and return
	MStatus toOutTriMesh(MObject &outputMesh);
	MStatus toOutTetMesh(MObject &outputMesh);

	// populate vertices with the vertex coordinates
	void buildOutVerticesArray(MDoubleArray &vertices);

	// populate elements with the indices of each tetrahedron
	void buildOutElementsArray(MIntArray &elements);
};


#endif