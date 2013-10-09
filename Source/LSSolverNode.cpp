#define MNoVersionString
#define MNoPluginEntry
#include "LSSolverNode.h"
#include <maya/MPointArray.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MTime.h>
#include <maya/MGlobal.h>
#include <maya/MAnimControl.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnMatrixAttribute.h>


static MStatus returnStatus;

#define McheckErr(stat,msg)			\
	if ( MS::kSuccess != stat ) {	\
		cerr << msg;				\
		return MS::kFailure;		\
	}


#define MAKE_INPUT(attr) \
CHECK_MSTATUS(attr.setKeyable(true)); \
CHECK_MSTATUS(attr.setStorable(true)); \
CHECK_MSTATUS(attr.setReadable(true)); \
CHECK_MSTATUS(attr.setWritable(true));
#define MAKE_OUTPUT(attr) \
CHECK_MSTATUS(attr.setKeyable(false)); \
CHECK_MSTATUS(attr.setStorable(false)); \
CHECK_MSTATUS(attr.setReadable(true)); \
CHECK_MSTATUS(attr.setWritable(false));
#define MAKE_ADDR(attr) \
CHECK_MSTATUS(attr.setKeyable(false)); \
CHECK_MSTATUS(attr.setStorable(false)); \
CHECK_MSTATUS(attr.setReadable(true)); \
CHECK_MSTATUS(attr.setWritable(false)); \
CHECK_MSTATUS(attr.setHidden(true));


MTypeId LSSolverNode::id( 0x00323);

MObject LSSolverNode::tetWorldMatrix;
MObject LSSolverNode::restShape; // input
MObject LSSolverNode::deformed; // output
MObject LSSolverNode::time;
MObject LSSolverNode::poissonRatio;
MObject LSSolverNode::youngsModulus;
MObject LSSolverNode::objectDensity;
MObject LSSolverNode::friction;
MObject LSSolverNode::restitution;
MObject LSSolverNode::damping;
MObject LSSolverNode::userSuppliedDt;
//MObject LSSolverNode::inputFilePath;
MObject LSSolverNode::restElements;
MObject LSSolverNode::restVertices;
MObject LSSolverNode::integrationType;
MObject LSSolverNode::forceModelType;
MObject LSSolverNode::selectedConstraintVerts;
MObject LSSolverNode::selectedForceVerts;
MObject LSSolverNode::forceApplicationTime;
MObject LSSolverNode::forceIncrementTime;
MObject LSSolverNode::forceReleasedTime;
MObject LSSolverNode::forceStartTime;
MObject LSSolverNode::forceStopTime;
MObject LSSolverNode::forceMagnitude;
MObject LSSolverNode::useSuppliedConstraints;
MObject LSSolverNode::useSuppliedForce;
MObject LSSolverNode::forceDirection; 
MObject LSSolverNode::contactKd;
MObject LSSolverNode::contactKs;

MStatus LSSolverNode::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus stat;
	
	if( plug == deformed)
	{
		MDataHandle tetWorldMatrixData = data.inputValue(tetWorldMatrix, &returnStatus);
		McheckErr(returnStatus, "Error getting tetWorldMatrix data handle\n");

		MDataHandle restShapeData = data.inputValue(restShape, &returnStatus);
		McheckErr(returnStatus, "Error getting step data handle\n");

		MDataHandle restVerticesData = data.inputValue(restVertices, &returnStatus);
		McheckErr(returnStatus, "Error getting step data handle\n");

		MDataHandle restElementsData = data.inputValue(restElements, &returnStatus);
		McheckErr(returnStatus, "Error getting step data handle\n");

		MDataHandle selectedConstraintVertsData = data.inputValue(selectedConstraintVerts, &returnStatus);
		McheckErr(returnStatus, "Error getting step data handle\n");

		MDataHandle selectedForceVertsData = data.inputValue(selectedForceVerts, &returnStatus);
		McheckErr(returnStatus, "Error getting step data handle\n");

		MDataHandle timeData = data.inputValue(time, &returnStatus);
		McheckErr(returnStatus, "Error getting step data handle\n");

		MDataHandle outputMeshData = data.outputValue(deformed, &returnStatus);
		McheckErr(returnStatus, "Error getting outputMesh data handle\n");
		
		MMatrix twmat = tetWorldMatrixData.asMatrix();
		MObject rs = restShapeData.asMesh();
		double t = timeData.asDouble();

		MDataHandle poissonRatioData = data.inputValue(poissonRatio, &returnStatus);
		McheckErr(returnStatus, "Error getting poissonRatio data handle\n");

		MDataHandle youngsModulusData = data.inputValue(youngsModulus, &returnStatus);
		McheckErr(returnStatus, "Error getting youngsmodulus data handle\n");

		MDataHandle objectDensityData = data.inputValue(objectDensity, &returnStatus);
		McheckErr(returnStatus, "Error getting objectDensity data handle\n");

		MDataHandle frictionData = data.inputValue(friction, &returnStatus);
		McheckErr(returnStatus, "Error getting friction data handle\n");

		MDataHandle restitutionData = data.inputValue(restitution, &returnStatus);
		McheckErr(returnStatus, "Error getting restitution data handle\n");

		MDataHandle dampingData = data.inputValue(damping, &returnStatus);
		McheckErr(returnStatus, "Error getting damping data handle\n");

		MDataHandle userSuppliedDtData = data.inputValue(userSuppliedDt, &returnStatus);
		McheckErr(returnStatus, "Error getting user supplied dt data handle\n");


		MDataHandle integrationTypeData = data.inputValue(integrationType, &returnStatus);
		McheckErr(returnStatus, "Error getting user integrationTypeData\n");

		MDataHandle forceModelTypeData = data.inputValue(forceModelType, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceModelTypeData\n");

		MDataHandle forceApplicationTimeData = data.inputValue(forceApplicationTime, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceApplicationTime\n");
	
		MDataHandle forceReleasedTimeData = data.inputValue(forceReleasedTime, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceReleasedTime\n");

		MDataHandle forceIncrementTimeData = data.inputValue(forceIncrementTime, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceIncrementTime\n");

		MDataHandle forceStartTimeData = data.inputValue(forceStartTime, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceStartTime\n");

		MDataHandle forceStopTimeData = data.inputValue(forceStopTime, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceStopTime\n");

		MDataHandle forceMagnitudeData = data.inputValue(forceMagnitude, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceIdleTime\n");

		MDataHandle useSuppliedForceData = data.inputValue(useSuppliedForce, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceIdleTime\n");

		MDataHandle useSuppliedConstraintsData = data.inputValue(useSuppliedConstraints, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceIdleTime\n");

		MDataHandle forceDirectionData = data.inputValue(forceDirection, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceDirection\n");

		MDataHandle contactKsData = data.inputValue(contactKs, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceDirection\n");	

		MDataHandle contactKdData = data.inputValue(contactKd, &returnStatus);
		McheckErr(returnStatus, "Error getting user forceDirection\n");

		MTime currentTime, maxTime;
		currentTime = MAnimControl::currentTime();
		maxTime = MAnimControl::maxTime();
					
		if (currentTime == MAnimControl::minTime())
		{
			// retrive restVertices and restElements
			MFnDoubleArrayData restVertArrayData(restVerticesData.data());
			MDoubleArray verts = restVertArrayData.array();
			int vertArrayLen = verts.length();
			double *vertArray = new double[vertArrayLen];
			verts.get(vertArray);

			for(int v=0;v<vertArrayLen;v=v+3)
			{
				MPoint mpoint = MPoint(vertArray[v],vertArray[v+1],vertArray[v+2])*twmat;
				vertArray[v] = mpoint.x;
				vertArray[v+1] = mpoint.y;
				vertArray[v+2] = mpoint.z;
			}

			MFnIntArrayData restEleArrayData(restElementsData.data());
			MIntArray ele = restEleArrayData.array();
			int eleArrayLen = ele.length();
			int *eleArray = new int[eleArrayLen];
			ele.get(eleArray);

			MFnIntArrayData selectedConstraintVertsArrayData(selectedConstraintVertsData.data());
			MIntArray sv = selectedConstraintVertsArrayData.array();

			// building selectedConstraintVerts
			vector<int> selectedConstraintVertIndices;
			for (int i = 0 ; i < sv.length() ; i++)
			{
				selectedConstraintVertIndices.push_back(sv[i]);
			}

			MFnIntArrayData selectedForceVertsArrayData(selectedForceVertsData.data());
			MIntArray sf = selectedForceVertsArrayData.array();

			vector<int> selectedForceVertIndices;
			for (int i = 0 ; i < sf.length() ; i++)
			{
				selectedForceVertIndices.push_back(sf[i]);
			}


			// temporarily create force direction vector
			double *forceDir = forceDirectionData.asDouble3();

	
			vector<double> dir;
			dir.push_back(forceDir[0]); dir.push_back(forceDir[1]);dir.push_back(forceDir[2]);

			prevDeformed = 0;
			double youngsModulusDouble = youngsModulusData.asDouble();
			double poissonRatioDouble = poissonRatioData.asDouble();
			double objectDensityDouble = objectDensityData.asDouble();
			double frictionDouble = frictionData.asDouble();
			double restitutionDouble = restitutionData.asDouble();
			double dampingDouble = dampingData.asDouble();
			double userSuppliedDtDouble = userSuppliedDtData.asDouble();
			double forceMagnitudeDouble = forceMagnitudeData.asDouble();
			int fAppT = forceApplicationTimeData.asInt();
			int fReleasedT = forceReleasedTimeData.asInt();
			int fIncT = forceIncrementTimeData.asInt();
			int fStartT = forceStartTimeData.asInt();
			int fStopT = forceStopTimeData.asInt();
			int integrationTypeInt = integrationTypeData.asShort();
			int forceModelTypeInt = forceModelTypeData.asShort();

			bool useSuppliedForceBool = useSuppliedForceData.asBool();
			bool useSuppliedConstraintsBool = useSuppliedConstraintsData.asBool();

			double contactKs = contactKsData.asDouble();
			double contactKd = contactKdData.asDouble();

			if( sm)
			{
				delete sm;
			}
			sm = new SoftBodySim(youngsModulusDouble,poissonRatioDouble,objectDensityDouble,
				frictionDouble,restitutionDouble,dampingDouble, eleArrayLen, eleArray, vertArrayLen, vertArray,integrationTypeInt,forceModelTypeInt);
			sm->setContactAttributes(contactKs,contactKd);
			if (useSuppliedConstraintsBool)
				sm->initialize("",userSuppliedDtDouble, selectedConstraintVertIndices);
			else
			{
				vector<int> empty;
				sm->initialize("",userSuppliedDtDouble, empty);
			}
			
			if (useSuppliedForceBool)
				sm->setUserForceAttributes(forceMagnitudeDouble, dir,selectedForceVertIndices,fAppT,fReleasedT,fIncT,fStartT,fStopT);
		}

		else
		{
			sm->update();
		}

		MFnMesh surfFn(rs,&stat);
		McheckErr( stat, "compute - MFnMesh error" );

		MFnMeshData ouputMeshDataCreator;
		MObject oMesh = ouputMeshDataCreator.create(&stat);
		buildOutputMesh(surfFn, sm->m_vertices,oMesh);
		outputMeshData.set(oMesh);
		data.setClean(plug);

	}

	else
		stat = MS::kUnknownParameter;

	return stat;
}

void* LSSolverNode::creator()
{
	return new LSSolverNode();
}

MStatus LSSolverNode::initialize()
{
	MFnNumericAttribute nAttr;
	MFnUnitAttribute uAttr;
	MFnTypedAttribute tAttr;
	MFnEnumAttribute eAttr;
	MFnMatrixAttribute mAttr;

	tetWorldMatrix = mAttr.create("tet_world_matrix","twm",MFnMatrixAttribute::kDouble,&returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode worldMatrix attribute\n");
	MAKE_INPUT(mAttr);
	mAttr.setHidden(true);

	restShape = tAttr.create("restShape", "rs", MFnMeshData::kMesh, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode restShape attribute\n");
	MAKE_INPUT(tAttr);

	restElements = tAttr.create("restElements", "re", MFnData::kIntArray, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode restShape attribute\n");
	MAKE_INPUT(tAttr);

	restVertices = tAttr.create("restVertices", "rv", MFnData::kDoubleArray, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode restShape attribute\n");
	MAKE_INPUT(tAttr);

	selectedConstraintVerts = tAttr.create("selectedConstraintVerts", "scv", MFnData::kIntArray, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode selectedConstraintVerts attribute\n");
	MAKE_INPUT(tAttr);

	selectedForceVerts = tAttr.create("selectedForceVerts", "sfv", MFnData::kIntArray, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode selectedForceVerts attribute\n");
	MAKE_INPUT(tAttr);

	time = uAttr.create("time", "t", MFnUnitAttribute::kTime,0.0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode time attribute\n");
	MAKE_INPUT(uAttr);

	deformed = tAttr.create("deformed", "d", MFnMeshData::kMesh, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode deformed attribute\n");
	MAKE_OUTPUT(tAttr);

	poissonRatio = nAttr.create("poissonRatio", "pr", MFnNumericData::kDouble, 0.45, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode poisson ratio\n");
	MAKE_INPUT(nAttr);
	nAttr.setMax(0.45);
	nAttr.setMin(0);

	youngsModulus = nAttr.create("youngsModulus", "ym", MFnNumericData::kDouble, 50000, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode youngsModulus\n");
	MAKE_INPUT(nAttr);
	//nAttr.setMax(50000);
	//nAttr.setMin(1);

	objectDensity = nAttr.create("density", "objd", MFnNumericData::kDouble, 1000, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode objectDensity\n");
	MAKE_INPUT(nAttr);
	nAttr.setMax(10000);
	nAttr.setMin(1);

	friction = nAttr.create("friction", "f", MFnNumericData::kDouble, 0.98, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode friction\n");
	MAKE_INPUT(nAttr);
	nAttr.setMax(1);
	nAttr.setMin(0);

	restitution = nAttr.create("restitution", "r", MFnNumericData::kDouble, 0.4, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode restitution\n");
	MAKE_INPUT(nAttr);
	nAttr.setMax(1);
	nAttr.setMin(0);

	damping = nAttr.create("damping", "dmp", MFnNumericData::kDouble, 0.0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode damping\n");
	MAKE_INPUT(nAttr);
	nAttr.setMax(1);
	nAttr.setMin(0);

	userSuppliedDt = nAttr.create("timeStep","ts",MFnNumericData::kDouble,0.01, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode userSuppliedDt\n");
	MAKE_INPUT(nAttr);
	nAttr.setMax(1);
	nAttr.setMin(0.00001);

	forceApplicationTime = nAttr.create("forceApplicationTime", "fat", MFnNumericData::kInt, 50, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode forceApplicationTime ratio\n");
	MAKE_INPUT(nAttr);

	forceReleasedTime = nAttr.create("forceReleasedTime", "frt", MFnNumericData::kInt, 100, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode forceReleasedTime ratio\n");
	MAKE_INPUT(nAttr);

	forceIncrementTime = nAttr.create("forceIncrementTime", "fit", MFnNumericData::kInt, 10, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode forceIncrementTime ratio\n");
	MAKE_INPUT(nAttr);

	forceStartTime = nAttr.create("forceStartTime", "fst", MFnNumericData::kInt,100, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode forceStartTime\n");
	MAKE_INPUT(nAttr);

	forceStopTime = nAttr.create("forceStopTime", "fet", MFnNumericData::kInt, 200, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode forceStopTime\n");
	MAKE_INPUT(nAttr);

	forceMagnitude = nAttr.create("forceMagnitude", "fm", MFnNumericData::kDouble, 10.0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode forceIdleTime ratio\n");
	MAKE_INPUT(nAttr);

	forceDirection = nAttr.create("forceDirection", "fd", MFnNumericData::k3Double, 0.0 , &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode forceIdleTime ratio\n");
	MAKE_INPUT(nAttr);

	useSuppliedConstraints = nAttr.create("useSuppliedConstraints", "usc", MFnNumericData::kBoolean, 0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode useSuppliedConstraints ratio\n");
	MAKE_INPUT(nAttr);

	useSuppliedForce = nAttr.create("useSuppliedForce", "usf", MFnNumericData::kBoolean, 0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode useSuppliedForces ratio\n");
	MAKE_INPUT(nAttr);


//	inputFilePath = tAttr.create("InputFilePath", "ifp", MFnData::kString, &returnStatus);
//	McheckErr(returnStatus, "ERROR creating LSSolverNode inputFilePath\n");

	integrationType = eAttr.create("timeIntegrationMethod","it",0,&returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode integrationType\n");
	eAttr.addField("Implicit Backward Euler",0);
	eAttr.addField("Implicit Newmark",1);
	eAttr.addField("Central Differences",2);
	eAttr.addField("Symplectic Euler",3);
	MAKE_INPUT(eAttr);

	forceModelType = eAttr.create("system","ft",0,&returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode forceModelType\n");
	eAttr.addField("Corotational Linear FEM",0);
	eAttr.addField("Mass Spring System",1);
	MAKE_INPUT(eAttr);
	
	contactKs = nAttr.create("contactSpringForce", "cKs", MFnNumericData::kDouble, 1000.0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode contactSpringForces\n");
	MAKE_INPUT(nAttr);

	contactKd = nAttr.create("contactDampForce", "cKd", MFnNumericData::kDouble, 50.0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode contactDampForces\n");
	MAKE_INPUT(nAttr);

	returnStatus = addAttribute(tetWorldMatrix);
	McheckErr(returnStatus, "ERROR adding LSSolverNode world matrix attribute\n");

	returnStatus = addAttribute(restShape);
	McheckErr(returnStatus, "ERROR adding LSSolverNode step attribute\n");
	
	returnStatus = addAttribute(restElements);
	McheckErr(returnStatus, "ERROR adding LSSolverNode restElements attribute\n");

	returnStatus = addAttribute(restVertices);
	McheckErr(returnStatus, "ERROR adding LSSolverNode restVertices attribute\n");

	returnStatus = addAttribute(selectedConstraintVerts);
	McheckErr(returnStatus, "ERROR adding LSSolverNode selectedConstraintVerts attribute\n");

	returnStatus = addAttribute(selectedForceVerts);
	McheckErr(returnStatus, "ERROR adding LSSolverNode selectedForceVerts attribute\n");

	returnStatus = addAttribute(time);
	McheckErr(returnStatus, "ERROR adding LSSolverNode time attribute\n");

	returnStatus = addAttribute(deformed);
	McheckErr(returnStatus, "ERROR adding LSSolverNode deformed attribute\n");

	returnStatus = addAttribute(youngsModulus);
	McheckErr(returnStatus, "ERROR adding LSSolverNode youngsModulus\n");

	returnStatus = addAttribute(poissonRatio);
	McheckErr(returnStatus, "ERROR adding LSSolverNode poissonRation\n");

	returnStatus = addAttribute(objectDensity);
	McheckErr(returnStatus, "ERROR adding LSSolverNode objectDensity\n");

	returnStatus = addAttribute(friction);
	McheckErr(returnStatus, "ERROR adding LSSolverNode friction\n");

	returnStatus = addAttribute(restitution);
	McheckErr(returnStatus, "ERROR adding LSSolverNode resitution\n");

	returnStatus = addAttribute(damping);
	McheckErr(returnStatus, "ERROR adding LSSolverNode damping\n");

	returnStatus = addAttribute(userSuppliedDt);
	McheckErr(returnStatus, "ERROR adding LSSolverNode userSuppliedDt\n");

	returnStatus = addAttribute(useSuppliedConstraints);
	McheckErr(returnStatus, "ERROR adding LSSolverNode useSuppliedConstraints\n");
	
	returnStatus = addAttribute(useSuppliedForce);
	McheckErr(returnStatus, "ERROR adding LSSolverNode useSuppliedForce\n");
	
	returnStatus = addAttribute(forceMagnitude);
	McheckErr(returnStatus, "ERROR adding LSSolverNode forceMagnitude\n");

	returnStatus = addAttribute(forceDirection);
	McheckErr(returnStatus, "ERROR adding LSSolverNode forceDirection\n");
	
	returnStatus = addAttribute(forceApplicationTime);
	McheckErr(returnStatus, "ERROR adding LSSolverNode forceApplicationTime\n");
	
	returnStatus = addAttribute(forceReleasedTime);
	McheckErr(returnStatus, "ERROR adding LSSolverNode forceReleasedTime\n");
	
	returnStatus = addAttribute(forceIncrementTime);
	McheckErr(returnStatus, "ERROR adding LSSolverNode forceIncrementTime\n");

	returnStatus = addAttribute(forceStartTime);
	McheckErr(returnStatus, "ERROR adding LSSolverNode forceIdleTime\n");
	
	returnStatus = addAttribute(forceStopTime);
	McheckErr(returnStatus, "ERROR adding LSSolverNode forceIdleTime\n");	

//	returnStatus = addAttribute(inputFilePath);
//	McheckErr(returnStatus, "ERROR adding LSSolverNode inputFilePath\n");

	returnStatus = addAttribute(integrationType);
	McheckErr(returnStatus, "ERROR adding LSSolverNode integrationType\n");

	returnStatus = addAttribute(forceModelType);
	McheckErr(returnStatus, "ERROR adding LSSolverNode forceModelType\n");

	returnStatus = addAttribute(contactKd);
	McheckErr(returnStatus, "ERROR adding LSSolverNode contactKd\n");

	returnStatus = addAttribute(contactKs);
	McheckErr(returnStatus, "ERROR adding LSSolverNode contactKs\n");

	returnStatus = attributeAffects(restShape, deformed);
	McheckErr(returnStatus, "ERROR in attributeAffects: restShape - deformed\n");
	
	returnStatus = attributeAffects(time, deformed);
	McheckErr(returnStatus, "ERROR in attributeAffects: time - deformed\n");
	

	return MS::kSuccess;
}

MStatus LSSolverNode::buildOutputMesh(MFnMesh& inputMesh, float* vertices, MObject &outputMesh)
{
	MStatus stat;
	MPointArray points;

	unsigned vIndex = 0;
	int numVertices = inputMesh.numVertices();
	for(int i=0; i<numVertices;i++)
	{
		double x = vertices[vIndex++];
		double y = vertices[vIndex++];
		double z = vertices[vIndex++];
		//std::cout<<"("<<x<<","<<y<<","<<z<<")"<<endl;
		MPoint point(x,y,z);
		points.append(point);
	}
	
	const int numFaces = inputMesh.numPolygons();
	int *face_counts = new int[numFaces];
	for(int i = 0 ; i < numFaces ; i++)
	{
		face_counts[i] = 3;
	}

	MIntArray faceCounts( face_counts, numFaces );

	// Set up and array to assign vertices from points to each face 
	int numFaceConnects = numFaces * 3;
	int *face_connects = new int[numFaceConnects];

	int faceConnectsIdx = 0;
	for ( int i=0; i<numFaces; i++ )
	{
		MIntArray polyVerts;
		inputMesh.getPolygonVertices( i, polyVerts );
		int pvc = polyVerts.length();
		//MString txt;
		//txt = MString("Polygon:")+i;
		//txt+="\n [";

		//std::cout<<txt<<std::endl;
		face_connects[faceConnectsIdx++] = polyVerts[0];
		face_connects[faceConnectsIdx++]= polyVerts[1];
		face_connects[faceConnectsIdx++] = polyVerts[2];
	}

	MIntArray faceConnects( face_connects, numFaceConnects );
	
	MFnMesh meshFS;
	MObject newMesh = meshFS.create(numVertices, numFaces,
									points, faceCounts, faceConnects,
									outputMesh, &stat);

	return stat;

}
