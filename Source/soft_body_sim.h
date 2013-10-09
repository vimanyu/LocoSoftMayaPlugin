
#ifndef SOFT_BODY_SIM_H_INCLUDED
#define SOFT_BODY_SIM_H_INCLUDED


#include <vector>
#include <string>
#include "vega/volumetricMeshLoader.h"
#include "vega/corotationalLinearFEM.h"
#include "vega/corotationalLinearFEMForceModel.h"
#include "vega/generateMassMatrix.h"
#include "vega/implicitBackwardEulerSparse.h"
#include "vega/centralDifferencesSparse.h"
#include "vega/eulerSparse.h"
#include "vega/massSpringSystemFromTetMesh.h"
#include "vega/massSpringSystemForceModel.h"
#include "tetgen.h"

using std::string;

class SoftBodySim
{
public:
    SoftBodySim();
	SoftBodySim(double youngsMod, double poisRatio, double density, 
		double frict, double restitu,double damp, int eleArrayLen, int *eleArray, int vertArrayLen, double *vertArray, int integrationType, int forceModelType);
	SoftBodySim(double youngsMod, double poisRatio, double density, 
		double frict, double restitu,double damp, string infPath, int eleArrayLen, int *eleArray, int vertArrayLen, double *vertArray, int integrationType, int forceModelType);
    SoftBodySim(unsigned int n);
	void setUserForceAttributes(double fMagnitude,const std::vector<double>& direction, const std::vector<int>& fVertexIndices, int fAppT, int fReleaseT, int fIncT, int fStartT,int fStopT);
   	void setContactAttributes(float springK, float dampK);
	virtual ~SoftBodySim();
	float* m_vertices;
	unsigned int numOfVertices;

    void initialize(std::string filename, float dt,const std::vector<int>& constrainedVIndices);
    void update();
	
	double *in_vertices; // these are vertices passed in by LSSolverNode, which are used to make TetMesh (i.e only used in the first frame)
	int *in_elements; // these are indices of the tetrahedra passed in by LSSolverNode, which are used to make TetMesh
	int in_vertices_size;
	int in_elements_size;

protected:

    //unsigned int m_solver_iterations;
	float* m_restVertices;
	double* m_forces;
	double* m_velocities;
	unsigned int numOfFaces;
    // vertices and estimated position.
    std::vector<unsigned int> m_triangle_list;

private:
	//ImplicitBackwardEulerSparse* implicitBackwardEulerSparse;
	IntegratorBaseSparse* integratorBaseSparse;
	ForceModel * forceModel;
	TetMesh * tetMesh;
	SparseMatrix * massMatrix;
	void addExternalForces();
	void addUserForces();
	void clearForces();
	void handleContacts();
    // update the normal per frame for visualization.

	// set up m_triangle_list and numOfFaces
	void setupTrifaces(const tetgenio & out);

	// debug: copy pointlist to m_vertices
	void copyPointlist(const tetgenio &out);

	//void createTetMeshFromFile();
	void createTetMesh();

private:
	double poissonRatio;
	double youngsModulus;
	double objectDensity;
	double restitution;
	double friction;
	double damping;
	string inputFilePath;
	int integrationType;
	int forceModelType;

	double fMagnitude;
	Vec3d fDirection;
	std::vector<int> fVertexIndices;
	int globalTimer;
	int timer;
	bool uForceWait;

	int forceApplicationTime;
	int forceReleasedTime;
	int forceIncrementTime;
	int forceStartTime;
	int forceStopTime;
	float g_penaltyKs;
	float g_penaltyKd;

};

#endif