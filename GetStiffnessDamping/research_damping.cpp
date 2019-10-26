#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <utility>
#include <map>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <Eigen/Dense>

using namespace OpenRAVE;
using namespace std;
using namespace Eigen;

vector<int> Getactivejoint(RobotBasePtr robot)
{
	vector<int> activejoint = {robot->GetJoint("Joint0")->GetDOFIndex(),robot->GetJoint("Joint1")->GetDOFIndex(),
	robot->GetJoint("Joint2")->GetDOFIndex(),robot->GetJoint("Joint3")->GetDOFIndex()};
	return activejoint;
}

vector<int> vector_arange(int dof)
{	
	vector<int> temp;
	for(int i=0; i<dof; i++)
	{
		temp.push_back(i);
	}
	return temp;
}

int main(int argc, char ** argv)
{

    string scenefilename = "/home/pengchang/openrave/test/research/env.xml";
    string viewername = "qtcoin"; // qtcoin

    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment

	
    penv->Load(scenefilename); // load the scene
 
    vector<RobotBasePtr> vrobots;
    penv->GetRobots(vrobots);
    RobotBasePtr probot = vrobots.at(0); //get robot probot

	// int current_active_dof = probot->GetActiveDOF();
	// probot->SetActiveDOFs(vector_arange(current_active_dof),DOF_Transform);

	//get the active joint index in a vector
	vector<int> activejoint = Getactivejoint(probot);

	//load the joint position velocity acceleration
	double time1 = 1.6473;
	double time2 = 1.7393;
	double time3 = 1.7666;
	vector<double> position1 = {1.1598, 0.783581, 0.562371, 0.290721};
 	vector<double> position2 = {1.14072, 0.754111, 0.51993, 0.238344};
	vector<double> position3 = {1.13855, 0.739258, 0.505685, 0.220465};

	double vel11 = (position2[0]-position1[0])/(time2-time1);
	double vel12 = (position2[1]-position1[1])/(time2-time1);
	double vel13 = (position2[2]-position1[2])/(time2-time1);
	double vel14 = (position2[3]-position1[3])/(time2-time1); 
	vector<double> velocity1 = {vel11,vel12,vel13,vel14};

	double vel21 = (position3[0]-position2[0])/(time3-time2);
	double vel22 = (position3[1]-position2[1])/(time3-time2);
	double vel23 = (position3[2]-position2[2])/(time3-time2);
	double vel24 = (position3[3]-position2[3])/(time3-time2);
	vector<double> velocity2 = {vel21,vel22,vel23,vel24};

	double accel1 = (vel21-vel11)/(time2-time1);
	double accel2 = (vel22-vel12)/(time2-time1);
	double accel3 = (vel23-vel13)/(time2-time1);
	double accel4 = (vel24-vel14)/(time2-time1);
	vector<double> acceleration = {accel1,accel2,accel3,accel4};

	for (auto accelvalue:acceleration) {
		cout << accelvalue << endl;
	}

	vector<double> Position = {position1[0],(position1[1]-position1[0]),(position1[2]-position1[1]),(position1[3]-position1[2])};
	vector<double> Velocity = {vel11,vel12,vel13,vel14};
	vector<double> Acceleration = {accel1,accel2,accel3,accel4};

	//load external force
	geometry::RaveVector<double> force = {0.0,-1.96,0.0}; // ForceTorqueMap asks for geometry::RaveVector
	geometry::RaveVector<double> torque = {0.0,0.0,0.0};
	std::pair<geometry::RaveVector<double>,geometry::RaveVector<double>> external_force (force,torque);
		//creating external force map
	KinBody::ForceTorqueMap externalforce_map; // std::map<int, std::pair<geometry::RaveVector<double>,geometry::RaveVector<double>>> externalforce_map; (also work)
		//adding the force to link_4
	externalforce_map[4] = external_force; //externalforce_map.insert(std::pair<int, std::pair<geometry::RaveVector<double>,geometry::RaveVector<double>>>(4,external_force)); (also work)

	//set the joint values
	probot->SetDOFValues(Position, 1, activejoint);
	probot->SetDOFVelocities(Velocity, 1, activejoint);

	//calculate the jacobian

	//calculate the inverse dynamics
	vector<double> Torque_gravity;
	probot->ComputeInverseDynamics(Torque_gravity, Acceleration,externalforce_map);
		// display inverse dynamics
		// for (size_t i=0; i<Torque_gravity.size(); i++){
		// 	std::cout << Torque_gravity.size() <<std::endl;
		// 	std::cout << Torque_gravity[i] << std::endl;
		// }
	cout << "Torque: " << endl;
	for (auto torque:Torque_gravity) {
		cout << torque << endl;
	}

	//get stiffness and damping 

		// // Stiffness: 
		// 0.762047
		// 0.757427
		// 1.11907
		// 0.828156
	vector<double> Stiffness = {0.762047, 0.757427, 1.11907, 0.828156};
	vector<double> Damping;
	
	Damping.push_back((Torque_gravity[0]-Stiffness[0]*(Position[0]-1.5708))/Velocity[0]);
	for (size_t i=1; i<Torque_gravity.size(); i++){
		Damping.push_back((Torque_gravity[i]-Stiffness[i]*Position[i])/Velocity[i]);
	}

	cout << "Damping: " << endl;
	for (auto damp:Damping) {
		cout << damp <<endl;
	}



	//set the viewer
	ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
	penv->Add(viewer);
	bool showgui = true;
	viewer->main(showgui);

	return 0;
}