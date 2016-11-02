/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Saulo Cardoso */

#include "header.h"
#include "planner.h"

using namespace std;
using namespace Eigen;

// Verifica se o Estado colide com o obstaculo
bool isStateValid(const ob::SpaceInformation *si, const ob::State *state)
{
    const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
    double x = s->getX(), y = s->getY();

    // Um obstaculo com o formato de uma circunferencia de raio 2.5 centrada em (5, 7.5) com tolerancia de 1 (+1)
    if ((((x-5)*(x-5)) + ((y-7.5)*(y-7.5))) <= (6.25)+1)
    {
       return 0;  //Retorna zero caso as coordenadas toquem o obstaculo
    }
    else if (si->satisfiesBounds(s))
    {
        return 1;
    }
}

bool areCoordinatesValid(double x,double y)
{
    if (((((x-5)*(x-5)) + ((y-7.5)*(y-7.5))) <= (6.25)+1) || (((x>20)||(x<0)) || ((y>20) || (y<0))))
    {
       return 0;        //Retorna zero caso as coordenadas toquem o obstaculo ou estejam foram dos limites do plano
    }
    else
    {
        return 1;
    }
}

double getArcLenght (Arc arc) // Retorna o comprimento do arco
{
    double lenght = 0;
    lenght = fabs(2 * arc.phi * (1/arc.radius)); 
    return lenght;
}

void PathNode::getArc (PathNode inicial,PathNode final)     // Calcula o arco entre dois pontos
{
    typedef Matrix<float,3,1> Matrix2f;                     //Usa Eigen para calcular matrizes
    Arc arco;
    Matrix2f start(inicial.dof[0],inicial.dof[1],inicial.dof[2]);
    Matrix2f end(final.dof[0],final.dof[1],0);
    Matrix2f center;

    // double thetaA = inicial.dof[2];        //Orientacao inicial
    //double thetaA = (M_PI)/2;
    double thetaA = .5*boost::math::constants::pi<double>();
    double thetaB;
    double phi;         //Bearing to end point
    double r;           //Radius of the arc

   
    phi = atan2(end(1,0)-start(1,0),end(0,0)-start(0,0));
   
    phi = atan2(sin(phi-thetaA),cos(phi-thetaA));

    r = (2*sin(phi))/sqrt(pow(start(0,0)-end(0,0),2)+pow(start(1,0)-end(1,0),2));
    arco.phi=phi;
    arco.radius=r;
   double comprimento = getArcLenght(arco);
   center(0,0) = -r*sin(thetaA) + start(0,0);
   center(1,0) = r*cos(thetaA) + start(1,0);
   thetaB=(2*phi)+thetaA;
   ofstream myfile;

   myfile.open("arc.txt",std::ios_base::app);
   
   //cout<<"Raio que conecta: "<<1/r<<"Ângulo: "<<phi<<" e comprimento: "<< comprimento<<" ,orientação final: "<<thetaB<< " e centro: "<<center(0,0)<<" "<<center(1,0)<<endl;

    myfile.close();
} 

std::fstream& GotoLine(std::fstream& file, unsigned int num)
{
    file.seekg(std::ios::beg);
    for(int i=0; i < num - 1; ++i)
    {
       file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}

void PathNode::setValues (PathNode* PathNodeo, float values[])
{
    PathNodeo->dof[0] = values[0];
    PathNodeo->dof[1] = values[1];
    PathNodeo->dof[2] = values[2];
}

bool calcula_Arco(float Xa, float Ya, float Xb, float Yb)
{
    float phi = atan2((Yb-Ya),(Xb-Xa));
    float thetaA = (.5*boost::math::constants::pi<double>());
     phi = atan2(sin(phi-thetaA),cos(phi-thetaA));
    //cout<< "Phi calculado: "<<phi <<endl;
    float k;
    k =  (2*sin(phi))/sqrt(pow(Yb-Ya,2)+pow(Xb-Xa,2));
    float r = (1/k);
    //cout<< "Raio calculado: "<<r<<endl;
    if (abs(r)>4){
        return 0;
    }
    else{
        return 1;
    }
    return 0;
}

void ReadfromFile (const char*infile)
{
    /*cout <<in<<endl;
    std::fstream infile(in);*/

    std::fstream in(infile);
    cout<<"            Abrindo o arquivo "<<infile<<" para leitura."<<endl;
    std::string line,line2;
    float dofi[3];
    float dofe[3];
    int i=0; 

    PathNode first;
    PathNode last;
  
    while(!in.eof())
    {
        GotoLine(in,(i+1));
        //cout<<"Número da primeira linha: "<<(i+1)<<endl;
        getline(in, line);
        std::istringstream iss(line);

        GotoLine(in,(i+2));
        //cout<<"Número da segunda linha: "<<(i+2)<<endl;
        getline(in, line2);
        std::istringstream iss2(line2);
        /*if (!(iss >> x >> y)) 
        { 
        cout<<"erro"<<endl;
        break; 
        } // error*/
        iss >> dofi[0] >> dofi[1] >> dofi[2];
        iss2 >> dofe[0] >> dofe[1] >> dofe[2];
        if ((!line.empty())&&(!line2.empty()))
        {
            //PathNode::setValues(&first,dofi[]);

            first.dof[0] = dofi[0];
            first.dof[1] = dofi[1];
            first.dof[2] = 0;
            last.dof[0] = dofe[0];
            last.dof[1] = dofe[1];

            first.getArc(first,last);
        }
        i++;
    }
}

void plan(ob::StateSpacePtr space)
{
    //  Define a simple setup class
    og::SimpleSetup ss(space);

    ob::SpaceInformation spi(space);
    //  Set start and goal states
    ob::ScopedState<> start(space), goal(space);
    start[0] = 5;start[1] = 0.;
    goal[0] = 5; goal[1] = 13;
    ss.setStartAndGoalStates(start, goal);

    //  Set boundaries for the map
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(20);   
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    //  Return the instance of the used state space.
    ob::SpaceInformationPtr si(ss.getSpaceInformation());

    //  Set motion validity checking function for this space
    ob::MotionValidatorPtr mv (new myMotionValidator(si));
    ss.getSpaceInformation ()->setMotionValidator (mv);

    //  Set state validity checking function for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, si.get(), _1));

    //  File to save start and goal states
    ofstream definition_file;
    definition_file.open("start_goal.txt",std::ios_base::app);
    definition_file << goal[0] << " " << goal[1] << endl;
    definition_file << start[0] <<" " << start[1] << " " << (.5*boost::math::constants::pi<double>()) << endl;
   

    //  This call is optional to get more output information
    //  Set the resolution at which state validity needs to be verified in order for a motion between
    //  two states to be considered valid. This value is specified as a fraction of the space's extent.
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.05);

    cout << spi.getMaximumExtent() << endl;
    cout << spi.getStateValidityCheckingResolution() << endl;

    //  Defines the planner
    og::RRT *rrt = new og::RRT(si);

    //  This parameter greatly influences the runtime of the algorithm.
    //  It represents the maximum length of a motion to be added in the tree of motions.
    rrt->setRange(12);
    ss.setPlanner (ob::PlannerPtr(rrt));

    si->setup();

    //  Prints out the setup preferences
    ss.print(std::cout);

    //  Sets the setup and tries to to solve the problem within 1 second of planning time
    //  ss.solve() will call ss.setup() automatically
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();

        //  Creates a path instance
        og::PathGeometric path = ss.getSolutionPath();
        path.printAsMatrix(std::cout);

        //  Saves the path to a file
        ofstream path_file;
        path_file.open ("path.txt");
        path.printAsMatrix(path_file);
        path_file.close();
        ReadfromFile("path.txt");
    }
    else
        std::cout << "No solution found!" << std::endl;
}

int main(int argc, char* argv[])
{
    // Opens the file start_goal.txt for cleaning
    ofstream myfile;
    myfile.open("start_goal.txt");
    
    try
    {
        po::options_description desc("Options");
        desc.add_options()
        ("plan", "solve")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc,
        po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
        po::notify(vm);

        ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());

        if (vm.count("plan"))
        {
            plan(space);
        }
    }
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exceção de tipo desconhecido!\n";
    }

    return 0;
}
