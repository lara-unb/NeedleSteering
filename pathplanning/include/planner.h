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

#ifndef _PLANNER_
#define _PLANNER_

class PathNode
{
public: 
    float dof[3];                       // x,y e theta
    void getArc(PathNode,PathNode);     // Get arc entre dois estados
    void setValues (PathNode*,float[]); // Define coordenadas
};

class Arc                               // Atributos do Arco
{
public: 
    double radius;
    double phi;
    double *start;
    double *end;
    double *center;
};

bool isStateValid(const ob::SpaceInformation *si, const ob::State *state);
bool areCoordinatesValid(double x,double y);
double getArcLenght (Arc arc);
std::fstream& GotoLine(std::fstream& file, unsigned int num);
bool calcula_Arco(float Xa, float Ya, float Xb, float Yb);
void ReadfromFile (const char*infile);
void plan(ob::StateSpacePtr space);


//Implementa classe que calcula e valida arco para qualquer planejador utilizado pela OMPL.
class myMotionValidator : public ob::MotionValidator
{
public:
    // brief Constructor
    myMotionValidator(ob::SpaceInformation *si) : ob::MotionValidator(si)
    {
    }

    // brief Constructor
    myMotionValidator(const ob::SpaceInformationPtr &si) : ob::MotionValidator(si)
    {
    }

    ~myMotionValidator()
    {
    }

    //brief Check if the path between two states (from \e s1 to \e s2) is valid. This function assumes  s1 is valid.

    bool checkMotion(const ob::State *s1, const ob::State *s2) const
    {
       const ob::SE2StateSpace::StateType *s_start = s1->as<ob::SE2StateSpace::StateType>();
       float X1 = s_start->getX();
       float Y1 = s_start->getY();
       const ob::SE2StateSpace::StateType *s_end = s2->as<ob::SE2StateSpace::StateType>();
       float X2 = s_end->getX();
       float Y2 = s_end->getY();
       int arco_valido;
       arco_valido = calcula_Arco(X1,Y1,X2,Y2);

       if (((areCoordinatesValid(X1,Y1))&&(areCoordinatesValid(X2,Y2)))&&(arco_valido==0))
        {
            //cout<<"O caminho entre:"<< X1 << ","<<Y1<<" e " <<X2 <<","<<Y2<<  " é válido" <<endl;
            return 1;
        }
        else
        {
            //cout<<"O caminho entre:"<< X1 << ","<<Y1<<" e " <<X2 <<","<<Y2<<  " não é válido" <<endl;
            return 0;
        }
    }


    bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State*, double> &lastValid) const
    {

    }

protected:
    //brief The instance of space information this state validity checker operates on
    ob::SpaceInformation    *si_;

    //brief Number of valid segments
    mutable unsigned int valid_;

    //brief Number of invalid segments
    mutable unsigned int invalid_;
};

#endif
