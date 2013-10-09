#define MNoVersionString
#define MNoPluginEntry
#include <iostream>
#include "LSTetgenNode.h"
#include <maya/MMatrix.h>
#include <maya/MFnMatrixData.h>
#include <maya/MPointArray.h>
#include <maya/MFnMatrixAttribute.h>

using std::string;
using std::ostringstream;

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

MTypeId LSTetgenNode::id( 0x00333);

MObject LSTetgenNode::worldMatrix;
MObject LSTetgenNode::inputMesh;
MObject LSTetgenNode::outputMesh;
MObject LSTetgenNode::radiusEdgeRatio;
MObject LSTetgenNode::volumeConstraint;
MObject LSTetgenNode::outputFilePath;
MObject LSTetgenNode::outputVerticesArray;
MObject LSTetgenNode::outputElementsArray;
MObject LSTetgenNode::saveFile;

MStatus LSTetgenNode::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus stat;
	
	if (plug == outputMesh)
	{
		MDataHandle radiusEdgeRatioData = data.inputValue(radiusEdgeRatio, &returnStatus);
		McheckErr(returnStatus, "Error getting radiusEdgeRatio data handle\n");

		MDataHandle volumeConstraintData = data.inputValue(volumeConstraint, &returnStatus);
		McheckErr(returnStatus, "Error getting volumeConstraint data handle\n");

		MDataHandle worldMatrixData = data.inputValue(worldMatrix, &returnStatus);
		McheckErr(returnStatus, "Error getting worldMatrix data handle\n");

		MDataHandle inputMeshData = data.inputValue(inputMesh, &returnStatus);
		McheckErr(returnStatus, "Error getting inputMesh data handle\n");

		MDataHandle outputMeshData = data.outputValue(outputMesh, &returnStatus);
		McheckErr(returnStatus, "Error getting outputMesh data handle\n");

		MDataHandle outputFilePathData = data.outputValue(outputFilePath, &returnStatus);
		McheckErr(returnStatus, "Error getting outputFilePath\n");

		MDataHandle saveFileData = data.inputValue(saveFile, &returnStatus);
		McheckErr(returnStatus, "Error getting user saveFile\n");

		
		MMatrix wmat = worldMatrixData.asMatrix();
		MObject surf = inputMeshData.asMesh();
		
		MObject thisObj = thisMObject();
		MPlug surfPlug( thisObj, inputMesh );
		if ( !surfPlug.isConnected() ) {
    		stat = data.setClean( plug );
			McheckErr( stat, "compute setClean" )
			return MS::kFailure;
		}
	
   		MFnMesh surfFn (surf, &stat);
		McheckErr( stat, "compute - MFnMesh error" );


		////////////////
		// Use Tetgen //
		////////////////

		
		MFnMeshData tetMeshDataCreator;
		MObject tetMesh = tetMeshDataCreator.create(&stat);
		toTetMesh(surf, tetMesh, wmat, radiusEdgeRatioData.asDouble(), volumeConstraintData.asDouble(), outputFilePathData.asString().asChar(),saveFileData.asBool());

		outputMeshData.set(tetMesh);

		
		// first compute the arrays and store in verts and ele
		MDoubleArray verts;
		buildOutVerticesArray(verts);
		MIntArray ele;
		buildOutElementsArray(ele);

		// then get handle for outputVerticesArray and set it up
		MDataHandle outputVerticesArrayData = data.outputValue(outputVerticesArray, &returnStatus);
		McheckErr(returnStatus, "Error getting outputVerticesArrayData Handle\n");
		MFnDoubleArrayData ovaData;
		MObject vertArrayObj = ovaData.create(verts);
		outputVerticesArrayData.set(vertArrayObj);
		
		// do similiar steps for outputElementsArray
		MDataHandle outputElementsArrayData = data.outputValue(outputElementsArray, &returnStatus);
		McheckErr(returnStatus, "Error getting outputVerticesArrayData Handle\n");
		MFnIntArrayData oeaData;
		MObject eleArrayObj = oeaData.create(ele);
		outputElementsArrayData.set(eleArrayObj);

		
		data.setClean(plug);
	}

	else
		stat = MS::kUnknownParameter;

	return stat;
}

void* LSTetgenNode::creator()
{
	return new LSTetgenNode();
}

MStatus LSTetgenNode::initialize()
{
	MFnUnitAttribute unitAttr;
	MFnNumericAttribute nAttr;
	MFnTypedAttribute tAttr;
	MFnMatrixAttribute mAttr;

	worldMatrix = mAttr.create("world_matrix","wm",MFnMatrixAttribute::kDouble,&returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode worldMatrix attribute\n");
	MAKE_INPUT(mAttr);
	mAttr.setHidden(true);

	inputMesh = tAttr.create("inputMesh", "im", MFnMeshData::kMesh, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode inputMesh attribute\n");
	MAKE_INPUT(tAttr);
	tAttr.setHidden(true);
	
	outputMesh = tAttr.create("outputMesh", "om", MFnMeshData::kMesh, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode outputMesh attribute\n");
	MAKE_OUTPUT(tAttr);

	outputVerticesArray = tAttr.create("outVerticesArray", "ova", MFnData::kDoubleArray, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode outputMesh attribute\n");
	MAKE_OUTPUT(tAttr);

	outputElementsArray = tAttr.create("outElementsArray", "oea", MFnData::kIntArray, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode outputMesh attribute\n");
	MAKE_OUTPUT(tAttr);

	radiusEdgeRatio = nAttr.create("radiusEdgeRatio", "rer", MFnNumericData::kDouble, 1.414, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode radiusEdgeRatio attribute\n");
	nAttr.setMin(1.0);
	nAttr.setMax(2.0);
	MAKE_INPUT(nAttr);

	volumeConstraint = nAttr.create("volumeConstraint", "vc", MFnNumericData::kDouble, 0.1, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode volumeConstraint attribute\n");
	nAttr.setMin(0.0001);
	nAttr.setMax(5.0);
	MAKE_INPUT(nAttr);
	
	outputFilePath = tAttr.create("OutputFilePath", "ofp", MFnData::kString, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode inputFilePath\n");
	tAttr.setKeyable(false);
	tAttr.setStorable(false);
	tAttr.setReadable(true); 
	tAttr.setWritable(true);

	saveFile = nAttr.create("saveFile", "sf", MFnNumericData::kBoolean, 0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSSolverNode saveFile ratio\n");
	MAKE_INPUT(nAttr);

	returnStatus = addAttribute(worldMatrix);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode world matrix attribute\n");
	returnStatus = addAttribute(inputMesh);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode inputMesh attribute\n");	
	returnStatus = addAttribute(radiusEdgeRatio);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode radiusEdgeRatio attribute\n");	
	returnStatus = addAttribute(volumeConstraint);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode volumeConstraint attribute\n");
	returnStatus = addAttribute(outputMesh);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode outputMesh attribute\n");
	returnStatus = addAttribute(outputFilePath);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode outputFilePath attribute\n");
	returnStatus = addAttribute(outputVerticesArray);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode outputFilePath attribute\n");
	returnStatus = addAttribute(outputElementsArray);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode outputFilePath attribute\n");
	returnStatus = addAttribute(saveFile);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode saveFile attribute\n");

	// set up attribute Affects for outputMesh
	returnStatus = attributeAffects(inputMesh, outputMesh);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode inputMesh - outputMesh\n");
	returnStatus = attributeAffects(radiusEdgeRatio, outputMesh);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode radiusEdgeRatio - outputMesh\n");
	returnStatus = attributeAffects(volumeConstraint, outputMesh);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode volumeConstraint - outputMesh\n");
	returnStatus = attributeAffects(worldMatrix, outputMesh);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");
	returnStatus = attributeAffects(saveFile, outputMesh);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");

	// set up attribute Affects for outputFilePath
	returnStatus = attributeAffects(inputMesh, outputFilePath);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode inputMesh - outputFilePath\n");
	returnStatus = attributeAffects(radiusEdgeRatio, outputFilePath);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode radiusEdgeRatio - outputFilePath\n");
	returnStatus = attributeAffects(volumeConstraint, outputFilePath);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode volumeConstraint - outputFilePath\n");
	returnStatus = attributeAffects(worldMatrix, outputFilePath);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");
	returnStatus = attributeAffects(saveFile, outputFilePath);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");

	// set up attribute Affects for outputVerticesArray
	returnStatus = attributeAffects(inputMesh, outputVerticesArray);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode inputMesh - outputVerticesArray\n");
	returnStatus = attributeAffects(radiusEdgeRatio, outputVerticesArray);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode radiusEdgeRatio - outputVerticesArray\n");
	returnStatus = attributeAffects(volumeConstraint, outputVerticesArray);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode volumeConstraint - outputVerticesArray\n");
	returnStatus = attributeAffects(worldMatrix, outputVerticesArray);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");
	returnStatus = attributeAffects(saveFile, outputVerticesArray);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");

	// set up attribute Affects for outputElementsArray
	returnStatus = attributeAffects(inputMesh, outputElementsArray);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode inputMesh - outputMesh\n");
	returnStatus = attributeAffects(radiusEdgeRatio, outputElementsArray);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode radiusEdgeRatio - outputMesh\n");
	returnStatus = attributeAffects(volumeConstraint, outputElementsArray);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode volumeConstraint - outputMesh\n");
	returnStatus = attributeAffects(worldMatrix, outputElementsArray);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");
	returnStatus = attributeAffects(saveFile, outputElementsArray);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");

	return MS::kSuccess;
}

void LSTetgenNode::toTetMesh(MObject &inputMesh, MObject &outputMesh, MMatrix &wmat, double radEdgeRatio, double volConstraint, string outputPath,bool saveFile)
{
	toTetgenIO(inputMesh, wmat);


	// parsing double arguments
	ostringstream strs;
	strs << radEdgeRatio;
	string radEdgeRatioString = strs.str();
	strs.clear();
	strs.seekp(0);
	strs << volConstraint;
	string volConstraintString = strs.str();
	string cmd = "Qpq"+radEdgeRatioString+"a"+volConstraintString;
	char* c_cmd = new char[cmd.size() + 1];
	std::copy(cmd.begin(), cmd.end(), c_cmd);
	c_cmd[cmd.size()] = '\0';
	
	tetrahedralize(c_cmd, &in, &out);
	
	char* path = new char[outputPath.size()+1];
	std::copy(outputPath.begin(), outputPath.end(), path);
	path[outputPath.size()] = '\0';
	
	if(saveFile)
	{
	out.save_poly(path);
	out.save_nodes(path);
	out.save_elements(path);
	out.save_faces(path);
	}

	toOutTetMesh(outputMesh);

	delete [] path;
	delete [] c_cmd;
}

void LSTetgenNode::buildOutElementsArray(MIntArray &elements)
{
	int numTets = out.numberoftetrahedra;

	elements.clear();
	for (int i = 0 ; i < numTets * 4 ; i++)
	{
		int index = out.tetrahedronlist[i];
		elements.append(index-1); // assume tetgen indexing start at 1 so subtract 1 for maya
	}
}

void LSTetgenNode::buildOutVerticesArray(MDoubleArray &vertices)
{
	int numVerts = out.numberofpoints;

	vertices.clear();
	for (int i = 0 ; i < numVerts * 3 ; i++)
	{
		double coord = out.pointlist[i];
		vertices.append(coord);
	}
}

MStatus LSTetgenNode::toOutTetMesh(MObject &outputMesh)
{
	MStatus stat;

	int	numVertices = out.numberofpoints;
	MPointArray points;
	MFnMesh	meshFS;

	REAL *tetpoints = out.pointlist;
	
	for(int i = 0 ; i < numVertices * 3; i++)
	{
		double x = tetpoints[i]; i++;
		double y = tetpoints[i]; i++;
		double z = tetpoints[i];
		MPoint point(x, y, z);
		points.append(point);
	}

	const int numFaces = out.numberoftetrahedra * 4;
	int *face_counts = new int[numFaces];

	for(int i = 0 ; i < numFaces ; i++)
	{
		face_counts[i] = 3;
	}

	MIntArray faceCounts( face_counts, numFaces );

	// Set up and array to assign vertices from points to each face 
	int numFaceConnects = numFaces * 3;
	int *face_connects = new int[numFaceConnects];

	int tetVertexIndex = 0;
	for(int i = 0 ; i < numFaceConnects ; i++)
	{
		int index1 = out.tetrahedronlist[tetVertexIndex]; tetVertexIndex++;
		int index2 = out.tetrahedronlist[tetVertexIndex]; tetVertexIndex++;
		int index3 = out.tetrahedronlist[tetVertexIndex]; tetVertexIndex++;
		int index4 = out.tetrahedronlist[tetVertexIndex]; tetVertexIndex++;
		
		// assume tetgen indexing start at 1 so subtract 1 for maya
		face_connects[i] = index1-1; i++;
		face_connects[i] = index3-1; i++;
		face_connects[i] = index2-1; i++;

		face_connects[i] = index1-1; i++;
		face_connects[i] = index2-1; i++;
		face_connects[i] = index4-1; i++;

		face_connects[i] = index1-1; i++;
		face_connects[i] = index4-1; i++;
		face_connects[i] = index3-1; i++;

		face_connects[i] = index4-1; i++;
		face_connects[i] = index2-1; i++;
		face_connects[i] = index3-1;
	}

	MIntArray faceConnects( face_connects, numFaceConnects );

	MObject newMesh = meshFS.create(numVertices, numFaces,
									points, faceCounts, faceConnects,
									outputMesh, &stat);

	return stat;
}

MStatus LSTetgenNode::toOutTriMesh(MObject &outputMesh)
{
	MStatus stat;

	int	numVertices = out.numberofpoints;
	MPointArray points;
	MFnMesh	meshFS;

	REAL *tetpoints = out.pointlist;
	
	for(int i = 0 ; i < numVertices * 3; i++)
	{
		double x = tetpoints[i]; i++;
		double y = tetpoints[i]; i++;
		double z = tetpoints[i];
		MPoint point(x, y, z);
		points.append(point);
	}

	const int numFaces = out.numberoftrifaces;
	int *face_counts = new int[numFaces];

	for(int i = 0 ; i < numFaces ; i++)
	{
		face_counts[i] = 3;
	}

	MIntArray faceCounts( face_counts, numFaces );

	// Set up and array to assign vertices from points to each face 
	int numFaceConnects = numFaces * 3;
	int *face_connects = new int[numFaceConnects];

	for(int i = 0 ; i < numFaceConnects ; i++)
	{
		int index = out.trifacelist[i] - 1; // assume index for tetgen starts at 1 so subtract 1 for maya
		face_connects[i] = index;
	}

	MIntArray faceConnects( face_connects, numFaceConnects );

	MObject newMesh = meshFS.create(numVertices, numFaces,
									points, faceCounts, faceConnects,
									outputMesh, &stat);

	return stat;
}

MStatus LSTetgenNode::toTetgenIO(MObject &inputMesh, MMatrix &wmat)
{
	MStatus stat;
	MFnMesh meshFn(inputMesh, &stat);
	McheckErr(stat, "LSTetgenNode::toTetgenIO: MFnMesh constructor failed");

	int numVertices = meshFn.numVertices();
	MPointArray vertexList;
	meshFn.getPoints(vertexList);

	tetgenio::facet *f;
	tetgenio::polygon *p;

	in.firstnumber = 1;
	in.numberofpoints = numVertices;
	in.pointlist = new REAL[in.numberofpoints * 3];

	// set up points
	int pointlistIndex = 0;
	for(int vertexListIndex = 0 ; vertexListIndex < numVertices ; vertexListIndex++)
	{
		MPoint mpoint = vertexList[vertexListIndex] * wmat;
		in.pointlist[pointlistIndex] = mpoint.x; pointlistIndex++;
		in.pointlist[pointlistIndex] = mpoint.y; pointlistIndex++;
		in.pointlist[pointlistIndex] = mpoint.z; pointlistIndex++;
	}

	
	// set up facets
	MIntArray facetVertexCount;
	MIntArray facetVertexIndexList;
	meshFn.getVertices(facetVertexCount, facetVertexIndexList);

	int numFacets = meshFn.numPolygons();
	in.numberoffacets = numFacets;
	in.facetlist = new tetgenio::facet[in.numberoffacets];

	for(int facetIndex = 0 ; facetIndex < numFacets ; facetIndex++)
	{
		f = &in.facetlist[facetIndex];
		f->numberofpolygons = 1;
		f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
		f->numberofholes = 0;
		f->holelist = NULL;
		p = &f->polygonlist[0];
		p->numberofvertices = facetVertexCount[facetIndex];
		p->vertexlist = new int[p->numberofvertices];
		
		MIntArray curFacetVertexIndices;
		meshFn.getPolygonVertices(facetIndex, curFacetVertexIndices);
		for(int facetVertexIndex = 0 ; facetVertexIndex < p->numberofvertices ; facetVertexIndex++)
		{
			p->vertexlist[facetVertexIndex] = curFacetVertexIndices[facetVertexIndex] + 1; // assume tetgen indexing start at 1 so add one to the index from maya
		}
	}
	
	return stat;
}

void LSTetgenNode::makeDummyMesh(MObject &out, MStatus &stat)
{
	int				numVertices;
	float			cubeSize;
	MFloatPointArray		points;
	MFnMesh			meshFS;

	// Scale the cube on the frame number, wrap every 10 frames.
	cubeSize		    		= 2.5;

	const int numFaces			= 6;
	numVertices					= 8;
	const int numFaceConnects	= 24;

	MFloatPoint vtx_1( -cubeSize, -cubeSize, -cubeSize );
	MFloatPoint vtx_2(  cubeSize, -cubeSize, -cubeSize );
	MFloatPoint vtx_3(  cubeSize, -cubeSize,  cubeSize );
	MFloatPoint vtx_4( -cubeSize, -cubeSize,  cubeSize );
	MFloatPoint vtx_5( -cubeSize,  cubeSize, -cubeSize );
	MFloatPoint vtx_6( -cubeSize,  cubeSize,  cubeSize );
	MFloatPoint vtx_7(  cubeSize,  cubeSize,  cubeSize );
	MFloatPoint vtx_8(  cubeSize,  cubeSize, -cubeSize );
	points.append( vtx_1 );
	points.append( vtx_2 );
	points.append( vtx_3 );
	points.append( vtx_4 );
	points.append( vtx_5 );
	points.append( vtx_6 );
	points.append( vtx_7 );
	points.append( vtx_8 );

	// Set up an array containing the number of vertices
	// for each of the 6 cube faces (4 verticies per face)
	//
	int face_counts[numFaces] = { 4, 4, 4, 4, 4, 4 };
	MIntArray faceCounts( face_counts, numFaces );

	// Set up and array to assign vertices from points to each face 
	//
	int face_connects[ numFaceConnects ] = {	0, 1, 2, 3,
												4, 5, 6, 7,
												3, 2, 6, 5,
												0, 3, 5, 4,
												0, 4, 7, 1,
												1, 7, 6, 2	};
	MIntArray faceConnects( face_connects, numFaceConnects );

	MObject newMesh = meshFS.create(numVertices, numFaces,
									points, faceCounts, faceConnects,
									out, &stat);

}