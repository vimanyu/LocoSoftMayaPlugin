#ifndef LSSOLVER_H
#define LSSOLVER_H
#include <maya/MPxCommand.h>
#include <maya/MPxNode.h>
#include <maya/MDGModifier.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnMesh.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MFnIntArrayData.h>
#include "soft_body_sim.h"
#include <vector>

using std::vector;

class LSSolverNode: public MPxNode
{
public:
	LSSolverNode() : sm(0) {};
	virtual ~LSSolverNode() {};
	virtual MStatus compute(const MPlug& plug, MDataBlock& data);
	static void* creator();
	static MStatus initialize();
	
	MStatus buildOutputMesh(MFnMesh& inputMesh, float* vertexPositions, MObject& outputMesh); 

public:
	static MObject tetWorldMatrix;
	static MObject restShape;
	static MObject restVertices; // rest shape vertex coordinates passed from TetgenNode
	static MObject restElements; // rest shape tetrahedron indices passed from TetgenNode
	static MObject deformed;
	static MObject time;
	//static MObject inputFilePath; // no longer necessary
	static MObject userSuppliedDt;
	
	// paramters for soft body sim: tetmesh -> setSingleMaterial
	static MObject poissonRatio;
	static MObject youngsModulus;
	static MObject objectDensity;

	// paramters for soft body sim: addExternalForces
	static MObject friction;
	static MObject restitution;
	static MObject damping;
	static MObject integrationType;
	static MObject forceModelType;
	
	// stores user selected vertices: input attribute connected to LSComponentNode's output
	static MObject selectedConstraintVerts;
	static MObject selectedForceVerts;

	static MObject useSuppliedConstraints;

	// stores force attributes
	static MObject forceApplicationTime;
	static MObject forceReleasedTime;
	static MObject forceIncrementTime;
	static MObject forceStartTime;
	static MObject forceStopTime;
	static MObject forceMagnitude;
	static MObject forceDirection; 
	static MObject useSuppliedForce;
	static MObject contactKs;
	static MObject contactKd;

	static MTypeId id;

	double prevDeformed;
	SoftBodySim* sm;
	
};

#endif