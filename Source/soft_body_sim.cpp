#include "soft_body_sim.h"
#include "glm.hpp"
#include "gtc/matrix_transform.hpp"
#include <iostream>
#define PARDISO

SoftBodySim::SoftBodySim()
	:numOfFaces(0),
	 numOfVertices(0),
	 m_vertices(0),
	 m_forces(0),
	 m_velocities(0),
	 m_restVertices(0),
	 integratorBaseSparse(0),
	 forceModel(0),
	 tetMesh(0),
	 massMatrix(0),
	 in_vertices(0),
	 in_elements(0),
	 in_vertices_size(0),
	 in_elements_size(0),
	 integrationType(0),
	 forceModelType(0),
	 damping(0),
	 fMagnitude(0.0),
	 fDirection(Vec3d(0,0,0)),
	 timer(0),
	 globalTimer(0),
	 uForceWait(true),
	 forceApplicationTime(100),
	 forceReleasedTime(150),
	 forceIncrementTime(10),
	 forceStartTime(100),
	 forceStopTime(200),
	 g_penaltyKs(1000),
	 g_penaltyKd(50)
{
}

SoftBodySim::SoftBodySim(double youngsMod, double poisRatio, double density, 
						 double frict, double restitu,double damp, string infPath, int eleArrayLen, int *eleArray, 
						 int vertArrayLen, double *vertArray,int iType, int fType)
	:numOfFaces(0),
	 numOfVertices(0),
	 m_vertices(0),
	 m_forces(0),
	 m_velocities(0),
	 m_restVertices(0),
	 integratorBaseSparse(0),
	 forceModel(0),
	 tetMesh(0),
	 massMatrix(0),
	 youngsModulus(youngsMod),
	 poissonRatio(poisRatio),
	 objectDensity(density),
	 friction(frict),
	 restitution(restitu),
	 inputFilePath(infPath),
	 in_vertices(vertArray),
	 in_elements(eleArray),
	 in_vertices_size(vertArrayLen),
	 in_elements_size(eleArrayLen),
	 integrationType(iType),
	 forceModelType(fType),
	 damping(damp),
	 fMagnitude(0.0),
	 fDirection(Vec3d(0,0,0)),
	 timer(0),
	 globalTimer(0),
	 uForceWait(true),
	 forceApplicationTime(50),
	 forceReleasedTime(150),
	 forceIncrementTime(10),
	 forceStartTime(100),
	 forceStopTime(200),
	 g_penaltyKs(1000),
	 g_penaltyKd(50)
{
}

SoftBodySim::SoftBodySim(double youngsMod, double poisRatio, double density, 
						 double frict, double restitu,double damp, int eleArrayLen, int *eleArray, 
						 int vertArrayLen, double *vertArray,int iType, int fType)
	:numOfFaces(0),
	 numOfVertices(0),
	 m_vertices(0),
	 m_forces(0),
	 m_velocities(0),
	 m_restVertices(0),
	 integratorBaseSparse(0),
	 forceModel(0),
	 tetMesh(0),
	 massMatrix(0),
	 youngsModulus(youngsMod),
	 poissonRatio(poisRatio),
	 objectDensity(density),
	 friction(frict),
	 restitution(restitu),
	 in_vertices(vertArray),
	 in_elements(eleArray),
	 in_vertices_size(vertArrayLen),
	 in_elements_size(eleArrayLen),
	 integrationType(iType),
	 forceModelType(fType),
	 damping(damp),
	 timer(0),
	 globalTimer(0),
	 uForceWait(true),
	 forceApplicationTime(50),
	 forceReleasedTime(150),
	 forceIncrementTime(10),
	 forceStartTime(100),
	 forceStopTime(200),
	 g_penaltyKs(1000),
	 g_penaltyKd(50)
{
}

void SoftBodySim::initialize(std::string filename, float dt,const std::vector<int>& constrainedVIndices)
{
//	if (in_elements_size != 0)
//	{
		//std::cout << "Creating tetMesh from vertices and indices array" << std::endl;
		createTetMesh();
//	}
//	else
//		createTetMeshFromFile();

	numOfVertices  = tetMesh->getNumVertices();
	tetMesh->setSingleMaterial(youngsModulus, poissonRatio, objectDensity);

	int r = 3 * numOfVertices; // total number of DOFs
	m_vertices = new float[r];
	m_restVertices = new float[r];
	int idx = 0;
	for(int i=0; i < numOfVertices; ++i)
	{
		Vec3d v = *tetMesh->getVertices()[i];
		m_vertices[idx++] = v[0];
		m_vertices[idx++] = v[1];
		m_vertices[idx++] = v[2];
	}

	idx = 0;
	for(int i=0; i < numOfVertices; ++i)
	{
		Vec3d v = *tetMesh->getVertices()[i];
		m_restVertices[idx++] = v[0];
		m_restVertices[idx++] = v[1];
		m_restVertices[idx++] = v[2];
	}

	m_velocities = new double[r];
	for (int i=0; i<r; i=i++)
	{
		m_velocities[i] = 0.0;
	}

	m_forces = new double[r];
	
	for (int i=0; i<r; i++)
	{
		m_forces[i] = 0;
	}

	//TODO make sure triangle_list is initialized properly
	//m_triangle_list.reserve(numOfFaces);
	//compute_normal();

	if( forceModelType == 0)
	{
		CorotationalLinearFEM* deformableModel = new CorotationalLinearFEM(tetMesh);
		// create the class to connect the deformable model to the integrator
		forceModel = new CorotationalLinearFEMForceModel(deformableModel);
	}
	
	else
	{
		MassSpringSystem* mss;
		MassSpringSystemFromTetMesh::GenerateMassSpringSystem(tetMesh,
		&mss,objectDensity,youngsModulus,damping,0);
		forceModel = new MassSpringSystemForceModel(mss);
	}

	float timestep = dt; // the timestep
	// create consistent (non-lumped) mass matrix
	
	//GenerateMassMatrix::computeMassMatrix(tetMesh, &massMatrix, true);
	// This option only affects PARDISO and SPOOLES solvers, where it is best
	// to keep it at 0, which implies a symmetric, non-PD solve.
	// With CG, this option is ignored.
	int positiveDefiniteSolver = 0;
	// constraining vertices 4, 10, 14 (constrained DOFs are specified 0-indexed):
	//int numConstrainedDOFs = 9;
	//int constrainedDOFs[9] = { 0, 1, 2,3,4,5,12,13,14,15,16,17};

	//int numConstrainedDOFs = 0;
	//int* constrainedDOFs = (int*)0;

	int numConstrainedDOFs = 3*constrainedVIndices.size();
	int* constrainedDOFs = (int*)0;
	if( numConstrainedDOFs != 0)
	{
		constrainedDOFs =  new int[numConstrainedDOFs];
		int j = 0;
		for(int i=0 ; i<constrainedVIndices.size();i++)
		{
			constrainedDOFs[j++] = 3*constrainedVIndices[i];
			constrainedDOFs[j++] = 3*constrainedVIndices[i]+1;
			constrainedDOFs[j++] = 3*constrainedVIndices[i]+2;
		}
	}

	// (tangential) Rayleigh damping
	float dampingMassCoef = 0.0; // "underwater"-like damping (here turned off)
	//float dampingStiffnessCoef = 0.05; // (primarily) high-frequency damping
	float dampingStiffnessCoef = damping;
	// initialize the integrator
	
	massMatrix = SparseMatrix::CreateIdentityMatrix(3*numOfVertices);
	massMatrix->ScalarMultiply(10);
	if( integrationType == 0)
	{
	integratorBaseSparse = new 
	ImplicitBackwardEulerSparse(r, timestep, massMatrix, forceModel,
	positiveDefiniteSolver, numConstrainedDOFs, constrainedDOFs,
	dampingMassCoef, dampingStiffnessCoef);
	}

	else if(integrationType == 1)
	{
	integratorBaseSparse = new 
	ImplicitNewmarkSparse(r, timestep, massMatrix, forceModel,
	positiveDefiniteSolver, numConstrainedDOFs, constrainedDOFs,
	dampingMassCoef, dampingStiffnessCoef);
	}

	else if(integrationType == 2)
	{
	integratorBaseSparse = new 
	CentralDifferencesSparse(r, timestep, massMatrix, forceModel,
	 numConstrainedDOFs, constrainedDOFs,
	dampingMassCoef, dampingStiffnessCoef);
	}

	else
	{
	integratorBaseSparse = new 
	EulerSparse(r, timestep, massMatrix, forceModel,
	0,numConstrainedDOFs, constrainedDOFs,
	dampingMassCoef);
	}
	//massMatrix = SparseMatrix::CreateIdentityMatrix(3*numOfVertices);
}

void SoftBodySim::createTetMesh()
{
	tetMesh = new TetMesh(in_vertices_size / 3.0, in_vertices, in_elements_size / 4.0, in_elements);
}

//void SoftBodySim::createTetMeshFromFile()
//{
//	char* filePath = new char[inputFilePath.size()+1];
//	std::copy(inputFilePath.begin(), inputFilePath.end(), filePath);
//	filePath[inputFilePath.size()] = '\0';
//	tetMesh = new TetMesh(filePath, 0);
//	delete [] filePath;
//}

void SoftBodySim::copyPointlist(const tetgenio &out)
{
	
	m_vertices = new float[out.numberofpoints * 3];
	m_restVertices = new float[out.numberofpoints * 3];
	std::copy(out.pointlist, out.pointlist+3*out.numberofpoints, m_vertices);
	std::copy(out.pointlist, out.pointlist+3*out.numberofpoints, m_restVertices);

	for(int i = 0 ; i < out.numberofpoints * 3 ; i++)
	{
		if( i % 3 == 1)
		{
			m_vertices[i] += 5;
			m_restVertices[i]+=5;
		}
	}
}


void SoftBodySim::setupTrifaces(const tetgenio &out)
{
	numOfFaces = out.numberoftrifaces;
	//m_vertices = new float[out.numberoftrifaces*3];
	
	// temp debug
	//int mvindex = 0;
	for(int i = 0 ; i < out.numberoftrifaces*3 ; i++)
	{
		// NOTE: as required and defined in cube.poly, the node indices start at 1 so we have to subtract 1 here to use the indices as array indices
		int index1 = out.trifacelist[i] - 1;
		m_triangle_list.push_back(index1);
			
		i++;
		int index2 = out.trifacelist[i] - 1;
		m_triangle_list.push_back(index2);
				
		i++;
		int index3 = out.trifacelist[i] - 1;
		m_triangle_list.push_back(index3);
	}
}


void SoftBodySim::addExternalForces()
{
	for (int i=0; i<numOfVertices; i=i++)
	{
		m_forces[3*i] = 0.0;
		m_forces[3*i+1] = -9.8;
		m_forces[3*i+2] = 0.0;
	}
	addUserForces();

	////Add collision forces here too
	//for (int i=1; i<3*numOfVertices; i= i+3)
	//{
	//	if( m_vertices[i] <=0.0)
	//		m_forces[i]+=30;
	//}

	//Collision layer
	int r = 3*numOfVertices;
	double* d = new double[r];
	double* v = new double[r];
	//const float friction = 0.98f;
	//const float restitution = 0.4f;

	integratorBaseSparse->GetqState(d,v);
	

	bool contact = true;

	for (int i=1; i<3*numOfVertices; i= i+3)
	{
		if( m_vertices[i] <=0.05 )
		{
			float dist = m_vertices[i];
			glm::vec3 normal(0,1,0);
			if(m_vertices[i]<= 0.0 && contact)
			{
				glm::vec3 pVelocity =glm::vec3( m_velocities[i-1], m_velocities[i], m_velocities[i+1]);
				glm::vec3 pPosition =glm::vec3( m_vertices[i-1], m_vertices[i], m_vertices[i+1]);
				glm::vec3 originalPosition = glm::vec3(m_restVertices[i-1],m_restVertices[i],m_restVertices[i+1]);

				glm::vec3 vel = pVelocity;
				glm::vec3 vn;
				glm::vec3 vt;
				float norm_fraction =  glm::dot(normal,vel);
				vn = norm_fraction*normal;
				vt = vel- vn;
				vn*= restitution;
				vt*= friction;
				pVelocity = vn+vt;

				//std::cout<<"Velocity before contact:("<<
				//	pVelocity.x<<","<<pVelocity.y<<","<<pVelocity.z<<")"<<std::endl;
				glm::vec3 reflVelocity = pVelocity - 2.0f*( glm::dot(pVelocity,normal))*normal ;

				v[i-1] = reflVelocity.x;
				v[i]   = reflVelocity.y;
				v[i+1] = reflVelocity.z;
				
				//std::cout<<"Velocity after contact:("<<
				//	reflVelocity.x<<","<<reflVelocity.y<<","<<reflVelocity.z<<")"<<std::endl;
				//
				//std::cout<<"Original disp before contact:("<<
				//	d[i-1]<<","<<d[i]<<","<<d[i+1]<<")"<<std::endl;	

				pPosition += fabs(dist)*normal;
				glm::vec3 displacement = pPosition - originalPosition;
				d[i-1]  = displacement.x;
				d[i]    = displacement.y;
				d[i+1]  = displacement.z;

				//std::cout<<"Displacement after contact:("<<
				//	d[i-1]<<","<<d[i]<<","<<d[i+1]<<")"<<std::endl;	
				continue;
			}

			glm::vec3 springForce = g_penaltyKs*(fabs(fabs(dist)-0.01f))*(normal);
			glm::vec3 ptVelocity =glm::vec3( m_velocities[i-1], m_velocities[i], m_velocities[i+1]);
			glm::vec3 dampForce = -g_penaltyKd*( glm::dot(ptVelocity,normal) )*normal;
			glm::vec3 penaltyForce = springForce + dampForce;
			m_forces[i-1] += penaltyForce.x;
			m_forces[i] += penaltyForce.y;
			m_forces[i+1] += penaltyForce.z;
		}

	}

		integratorBaseSparse->SetState(d,v);
		delete[] d;
		delete[] v;

}

void SoftBodySim::clearForces()
{
	for (int i=0; i<numOfVertices; i=i++)
	{
		m_forces[3*i] = 0.0;
		m_forces[3*i+1] = 0.0;
		m_forces[3*i+2] = 0.0;
	}
}


void SoftBodySim::update()
{

	int r = 3*numOfVertices;
	
	integratorBaseSparse->SetExternalForcesToZero();
	clearForces();
	addExternalForces();

	//for(int i = 0 ; i < 3 * numOfVertices ; i++)
	//{
	//	if( i % 3 == 1)
	//		std::cout << m_vertices[i] << std::endl;
	//}

	//std::cout << "end of iteration" << std::endl;
	//double * f = new double[r];
	//for(int j=0; j<r; j++)
	//{
	//	if( j % 3 == 1)
	//		f[j] = -9.8;
	//	else
	//		f[j] = 0;
	//}
	
	integratorBaseSparse->SetExternalForces(m_forces);

	// important: must always clear forces, as they remain in effect unless changed
	integratorBaseSparse->DoTimestep();

	double * u = new double[r];
	//double* acc = new double[r];
	integratorBaseSparse->GetqState(u,m_velocities);
	for(int i=0; i< 3 * numOfVertices; i++)
	{
		m_vertices[i]=m_restVertices[i] +  u[i];
	}
	//std::cout<<"OUR CALCULATION"<<std::endl;
	//for(int i=0; i<numOfVertices; i++)
	//{
	//	std::cout<< m_vertices[3*i]<<","<<m_vertices[3*i+1]<<","<<m_vertices[3*i+2]<<std::endl;
	//}
	clearForces();
	
	/*std::cout<<"TOTAL MASS: "<<integratorBaseSparse->GetTotalMass()<<std::endl;

	std::cout<< "Acceleration: "<<std::endl;
	for(int i = 0; i<3*numOfVertices; i=i+3)
	{
		std::cout<< acc[i+1]<<std::endl;
	}*/
	
	//handleContacts();
	// compute normals here
	//compute_normal();
	//double* fint = new double[r];
	//forceModel->GetInternalForce(u,fint);
	//std::cout<< "Forces: "<<std::endl;
	//for(int i = 0; i<3*numOfVertices; i=i+3)
	//{
	//	std::cout<< fint[i+1]<<std::endl;
	//}

	//delete[] u;
}

void SoftBodySim::handleContacts()
{
	int r = 3*numOfVertices;
	double * u = new double[r];
	integratorBaseSparse->GetqState(u);
	for (int i=0; i<r; i= i+3)
	{
		if (m_vertices[i+1] <=0.0)
		{
			m_velocities[i+1] = -m_velocities[i+1];
			u[i+1]+=  0.0 - m_vertices[i+1];
		}

	}

	double * v = new double[r];
	//std::copy(m_vertices,m_vertices+r, u);
	std::copy(m_velocities, m_velocities+r, v);
	integratorBaseSparse->SetqState(u,v);

}



SoftBodySim::~SoftBodySim()
{
	delete[] m_velocities;
	delete[] m_forces;
	delete[] m_vertices;
	delete massMatrix;
	delete tetMesh;
	delete forceModel;
	delete integratorBaseSparse;
	delete[] in_vertices;
	delete[] in_elements;
}

void SoftBodySim::setUserForceAttributes(double fMag, const std::vector<double>& dir, const std::vector<int>& fvIndices, 
										 int fAppT, int fReleaseT, int fIncT, int fStartT,int fStopT)
{
	fMagnitude = fMag;
	fDirection = Vec3d(dir[0],dir[1],dir[2]);
	fDirection.normalize();
	fVertexIndices = fvIndices;
	forceApplicationTime = fAppT;
	forceReleasedTime = fReleaseT;
	forceIncrementTime = fIncT;
	forceStartTime = fStartT;
	forceStopTime = fStopT;

	//fMagnitude = 10;
	//fFrequency = 10;
	//fDirection = Vec3d(0,-1,0);
	//fVertexIndices.push_back(3);
	//fVertexIndices.push_back(5);
}

void SoftBodySim::addUserForces()
{
	Vec3d userForce = fMagnitude*fDirection;
	//forceApplicationTime = 100;
	//forceReleasedTime = 150;
	//forceIncrementTime = 10;

	timer++;
	globalTimer++;
	if( globalTimer>=forceStartTime && globalTimer <= forceStopTime)
		uForceWait = false;

	else
		uForceWait = true;
	
	for(int i=0; i<fVertexIndices.size(); i++)
	{
		if(!uForceWait)
		{

			int segment = (timer-forceStartTime)/forceIncrementTime;

			if (segment*forceIncrementTime <= forceApplicationTime)
			{
				m_forces[3*fVertexIndices[i]]+= segment*userForce[0];
				m_forces[3*fVertexIndices[i]+1]+= segment*userForce[1];
				m_forces[3*fVertexIndices[i]+2]+= segment*userForce[2];
			}

			else if (segment*forceIncrementTime > forceApplicationTime+forceReleasedTime)
			{
				timer = forceStartTime;
			}


			//if(timer<100+ fFrequency/4)
			//{
			//	m_forces[3*fVertexIndices[i]]+= userForce[0];
			//	m_forces[3*fVertexIndices[i]+1]+= userForce[1];
			//	m_forces[3*fVertexIndices[i]+2]+= userForce[2];
			//}
			//else if(timer<100+ fFrequency/2)
			//{
			//	m_forces[3*fVertexIndices[i]]+= 2*userForce[0];
			//	m_forces[3*fVertexIndices[i]+1]+= 2*userForce[1];
			//	m_forces[3*fVertexIndices[i]+2]+= 2*userForce[2];
			//}

			//else if(timer<100+ fFrequency)
			//{
			//	m_forces[3*fVertexIndices[i]]+= 4*userForce[0];
			//	m_forces[3*fVertexIndices[i]+1]+= 4*userForce[1];
			//	m_forces[3*fVertexIndices[i]+2]+= 4*userForce[2];
			//}

			//else if(timer > 2*fFrequency+100)
			//{
			//	timer = 100;
			//}
		}
	}

}

void SoftBodySim::setContactAttributes(float springK, float dampK)
{
	g_penaltyKd = dampK;
	g_penaltyKs = springK;
}