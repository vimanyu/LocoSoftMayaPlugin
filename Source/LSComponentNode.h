#ifndef LSCOMPONENT_H
#define LSCOMPONENT_H
#include <maya/MPxCommand.h>
#include <maya/MPxNode.h>
#include <maya/MDGModifier.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnMesh.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MFnTypedAttribute.h>

using std::string;

class LSComponentNode: public MPxNode
{
public:
	LSComponentNode() {};
	virtual ~LSComponentNode() {};
	virtual MStatus compute(const MPlug& plug, MDataBlock& data);
	static void* creator();
	static MStatus initialize();

public:
	static MObject in_vertIndicesArray;
	static MObject out_vertIndicesArray;
	static MTypeId id;
};

#endif
