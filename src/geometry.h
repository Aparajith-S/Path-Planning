/// \file geometry.h
/// \brief 2 dimensional geometry vector and bounding box class definitions
/// \author s.aparajith@live.com
/// \date 08.06.2021
/// \copyright None reserved. MIT license
#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <cmath>
#include <vector>
#include <limits>
namespace path_planner{
struct vector2d
{
    vector2d():vector2d(0.0,0.0){}
    vector2d(double x_, double y_):x(x_),y(y_){}
    vector2d(const vector2d&)=default;
    vector2d& operator=(vector2d const & other)=default;
    /// \brief compute cartesian distance
    /// \return distance
    double dist2d(vector2d p2) const
    {return sqrt(pow(this->x-p2.x,2.0)+pow(this->y-p2.y,2.0));}
    
    ///\brief add two vectors
    vector2d& operator+=(vector2d& rhs)
    {this->x+=rhs.x;
    this->y+=rhs.y;
     return *this;}
    
    ///\brief subtract two vectors
    vector2d& operator-=(vector2d& rhs){
    this->x-=rhs.x;
    this->y-=rhs.y;
    return *this;}
    
    /// \brief compute the dot product of two vectors
    /// \return dot product
    double dot(vector2d const &that)const{
        return this->x*that.x+this->y*that.y;
    }
    double x;
    double y;
};

/// \class bounding box rectangle
class rectangle
{ public:
    rectangle()=delete;
    /// \brief creates a bounding box with width and length.
    explicit rectangle(double length,double width):
    vertexBL(-length*0.5,width*0.5),
    vertexTL(length*0.5,width*0.5),
    vertexTR(length*0.5,-width*0.5),
    vertexBR(-length*0.5,-width*0.5),
    orientation(0.0)
    {}
    virtual ~rectangle(){}
    rectangle(rectangle const &)=default;

    /// \brief rotates along a longitudinal axis
    void rotateLongAxis(double theta)
    {
        double costheta=cos(theta);
        double sintheta=sin(theta);
        vector2d temp;
        temp.x = (costheta*this->vertexBL.x)-(sintheta*this->vertexBL.y);
        temp.y = (sintheta*this->vertexBL.x)+(costheta*this->vertexBL.y);
        this->vertexBL.x=temp.x;
        this->vertexBL.y=temp.y;
        temp.x  = (costheta*this->vertexTL.x)-(sintheta*this->vertexTL.y);
        temp.y  = (sintheta*this->vertexTL.x)+(costheta*this->vertexTL.y);
        this->vertexTL.x=temp.x;
        this->vertexTL.y=temp.y;
        temp.x = (costheta*this->vertexTR.x)-(sintheta*this->vertexTR.y);
        temp.y = (sintheta*this->vertexTR.x)+(costheta*this->vertexTR.y);
        this->vertexTR.x = temp.x;
        this->vertexTR.y = temp.y;
        temp.x = (costheta*this->vertexBR.x)-(sintheta*this->vertexBR.y);
        temp.y = (sintheta*this->vertexBR.x)+(costheta*this->vertexBR.y);
        this->vertexBR.x = temp.x;
        this->vertexBR.y = temp.y;
        this->orientation=theta;
    }

    /// \brief translates the rectangle using the suplied coordinates from its center.
    void translate(vector2d T){
        vertexBL+=T;
        vertexTL+=T;
        vertexTR+=T;
        vertexBR+=T;
    }

    /// \brief : using (SAT)separating axis theorem find overlapping rectangles
    /// \return : returns true if bounding boxes overlap.
    /// \see theory : https://gamedevelopment.tutsplus.com/tutorials/collision-detection-using-the-separating-axis-theorem--gamedev-169
    bool isSATOverlap(rectangle const & that)
    {
        std::vector<vector2d> Axes = findPrincipalAxes(that);
        std::vector<vector2d> thisvecs=this->vertices();
        std::vector<vector2d> thatvecs=that.vertices();
        for(auto const & PA : Axes )
        {
            double min_0 = std::numeric_limits<double>::max();
            double max_0 = -std::numeric_limits<double>::max();
            double min_1 = std::numeric_limits<double>::max();
            double max_1 = -std::numeric_limits<double>::max();
            bool overlap=false;
            for(auto const & vert : thisvecs)
            {
                double proj_0 = PA.dot(vert); 
                max_0=std::max(proj_0,max_0);
                min_0=std::min(proj_0,min_0);
            }
            for(auto const & vert : thatvecs)
            {
                double proj_1 = PA.dot(vert); 
                max_1=std::max(proj_1,max_1);
                min_1=std::min(proj_1,min_1);
            }
            overlap |= (min_1 >= min_0 && min_1 < max_0);
            overlap |= (max_1 >= min_0 && max_1 < max_0);
            overlap |= (min_0 >= min_1 && min_0 < max_1);
            overlap |= (max_0 >= min_1 && max_0 < max_1);
            if(overlap==false)
            {return false;}
        }
        return true;
    }

    /// \brief gets the vertices of the rectangle in the cartesian coordinate system
    /// \return vector of coordinates from bottom left to bottom right in clockwise 
    std::vector<vector2d> vertices()const
    {
        std::vector<vector2d> verts;
        verts.push_back(this->vertexBL);
        verts.push_back(this->vertexTL);
        verts.push_back(this->vertexTR);
        verts.push_back(this->vertexBR);
        return verts;
    }
  private:
    /// \brief finds principal axes for itself and the other rectangle to be used with SAT
    std::vector<vector2d> findPrincipalAxes(rectangle const & that)
    {
        double costheta=cos(this->orientation);
        double sintheta=sin(this->orientation);
        double costheta_1=cos(that.orientation);
        double sintheta_1=sin(that.orientation);
        std::vector<vector2d> axes = std::vector<vector2d>();
        axes.push_back(vector2d(costheta,sintheta));
        axes.push_back(vector2d(-sintheta,costheta));
        axes.push_back(vector2d(costheta_1,sintheta_1));
        axes.push_back(vector2d(-sintheta_1,costheta_1));
        return axes;
    }
    vector2d vertexBL;
    vector2d vertexTL;
    vector2d vertexTR;
    vector2d vertexBR;
    double orientation;
};
}

#endif