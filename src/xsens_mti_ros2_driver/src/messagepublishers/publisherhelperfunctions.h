//  Copyright (c) 2003-2024 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.


#ifndef PUBLISHER_HELPER_FUNCTION_H
#define PUBLISHER_HELPER_FUNCTION_H

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <xstypes/xsquaternion.h>

// Rotate a vector from the fixed (world) frame into the sensor body
// frame using the body->world orientation quaternion.
// Computes v_body = q^-1 * v_world * q in closed form
// (v + 2*qv_inv x (qv_inv x v + w*v), where qv_inv = -qv).
inline void rotateWorldToBody(const XsQuaternion &q,
                              double wx, double wy, double wz,
                              double &bx, double &by, double &bz)
{
    const double qw = q.w();
    const double qx = q.x();
    const double qy = q.y();
    const double qz = q.z();

    const double tx = 2.0 * (-qy * wz + qz * wy);
    const double ty = 2.0 * (-qz * wx + qx * wz);
    const double tz = 2.0 * (-qx * wy + qy * wx);

    bx = wx + qw * tx + (-qy * tz + qz * ty);
    by = wy + qw * ty + (-qz * tx + qx * tz);
    bz = wz + qw * tz + (-qx * ty + qy * tx);
}

class PublisherHelperFunctions
{
public:
    PublisherHelperFunctions(/* args */);
    ~PublisherHelperFunctions();

    void variance_from_stddev_param(std::string param, double *variance_out, rclcpp::Node::SharedPtr node_handle)
    {
        std::vector<double> stddev;
        if (node_handle->get_parameter(param, stddev))
        {
            if (stddev.size() == 3)
            {
                auto squared = [](double x) { return x * x; };
                std::transform(stddev.begin(), stddev.end(), variance_out, squared);
            }
            else
            {
                RCLCPP_WARN(node_handle->get_logger(), "Wrong size of param: %s, must be of size 3", param.c_str());
            }
        }
        else
        {
            memset(variance_out, 0, 3 * sizeof(double));
        }
    }

};

PublisherHelperFunctions::PublisherHelperFunctions(/* args */)
{
}

PublisherHelperFunctions::~PublisherHelperFunctions()
{
}

#endif

