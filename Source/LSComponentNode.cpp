#define MNoVersionString
#define MNoPluginEntry
#include "LSComponentNode.h"

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

MTypeId LSComponentNode::id( 0x00383);
MObject LSComponentNode::in_vertIndicesArray;
MObject LSComponentNode::out_vertIndicesArray;

MStatus LSComponentNode::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus stat;
	if (plug == out_vertIndicesArray)
	{
		// retrieve array from in_vertIndicesArray
		MDataHandle inVertIndicesArrayData = data.inputValue(in_vertIndicesArray, &returnStatus);
		McheckErr(returnStatus, "Error getting inVertIndicesArrayData Handle\n");
		MFnIntArrayData inVertArrayData = inVertIndicesArrayData.data();
		MIntArray inVertArray = inVertArrayData.array();


		// copy values from in_vertIndicesArray to out_vertIndicesArray
		MIntArray vertIndices;
		vertIndices.clear();
		vertIndices.copy(inVertArray);

		MDataHandle outVertIndicesArrayData = data.outputValue(out_vertIndicesArray, &returnStatus);
		McheckErr(returnStatus, "Error getting outVertIndicesArrayData Handle\n");
		MFnIntArrayData oviaData;
		MObject vertIndicesObj = oviaData.create(vertIndices);
		outVertIndicesArrayData.set(vertIndicesObj);

		// debug: Checking if the arrays are filled
		int len1 = inVertArray.length();
		int len2 = vertIndices.length();
		int *debug1 = new int[len1];
		int *debug2 = new int[len2];
		inVertArray.get(debug1);
		vertIndices.get(debug2);
		delete[] debug1;
		delete[] debug2;


		data.setClean(plug);
	}

	else
		stat = MS::kUnknownParameter;

	return stat;
}

void* LSComponentNode::creator()
{
	return new LSComponentNode();
}

MStatus LSComponentNode::initialize()
{
	MFnTypedAttribute tAttr;

	in_vertIndicesArray = tAttr.create("in_vertIndicesArray", "iva", MFnData::kIntArray, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode outputMesh attribute\n");
	MAKE_INPUT(tAttr);

	out_vertIndicesArray = tAttr.create("out_vertIndicesArray", "ova", MFnData::kIntArray, &returnStatus);
	McheckErr(returnStatus, "ERROR creating LSTetgenNode outputMesh attribute\n");
	MAKE_OUTPUT(tAttr);

	returnStatus = addAttribute(in_vertIndicesArray);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode outputFilePath attribute\n");
	returnStatus = addAttribute(out_vertIndicesArray);
	McheckErr(returnStatus, "ERROR adding LSTetgenNode outputFilePath attribute\n");

	// set up attribute Affects for outputElementsArray
	returnStatus = attributeAffects(in_vertIndicesArray, out_vertIndicesArray);
	McheckErr(returnStatus,"ERROR in attributeAffects: LSTetgenNode inputMesh - outputMesh\n");


	return MS::kSuccess;
}
