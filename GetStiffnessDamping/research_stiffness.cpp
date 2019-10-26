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
	double record1 = 0.84107; // need more data
	double record2 = 0.193146;
	double record3 = -0.159449;
	double record4 = -0.519068;
	double joint1 = record1;
	double joint2 = record2-record1;
	double joint3 = record3-record2;
	double joint4 = record4-record3;
	vector<double> Position = {joint1,joint2,joint3,joint4};
	vector<double> Velocity = {0.0,0.0,0.0,0.0};
	vector<double> Acceleration = {0.0,0.0,0.0,0.0};

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
	vector<double> Stiffness;
	// thought the first joint should refer to the y axis (upwards)
	Stiffness.push_back((double)Torque_gravity[0]/(double)(Position[0]-1.5708));
	for (size_t i=1; i<Torque_gravity.size(); i++){
		Stiffness.push_back((double)Torque_gravity[i]/(double)Position[i]);
	}
		// display stiffness
		// for (int i=0; i<4; i++){
		// 	std::cout << Stiffness[i] << std::endl;
		// }
	cout << "Stiffness: " << endl;
	for (auto stiff:Stiffness) {
		cout << stiff <<endl;
	}

// // Stiffness: 
// 0.762047
// 0.757427
// 1.11907
// 0.828156

	//set the viewer
	ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
	penv->Add(viewer);
	bool showgui = true;
	viewer->main(showgui);

	return 0;
}