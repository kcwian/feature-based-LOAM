//
// Created by mnowicki on 12.12.18.
//

#ifndef LOAM_VELODYNE_LIEALEGEBRA_H
#define LOAM_VELODYNE_LIEALEGEBRA_H

#include <Eigen/Eigen>

namespace Eigen {
    typedef Eigen::Matrix<float, 6, 1> Vector6d;
}

class LieAlgebra {
public:
    static Eigen::Matrix3d exp(const Eigen::Vector3d &omega) {
        double theta = omega.norm();
        Eigen::Matrix3d Omega = skew(omega);

        Eigen::Matrix3d R;
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;

        if (theta<0.00001)
        {
            double theta2 = theta*theta;
            double theta4 = theta2*theta2;
            a = 1.0 - theta2 / 6.0 + theta4 / 120.0;
            b = 0.5 - theta2 / 24.0 + theta4 / 720.0;
        }
        else
        {
            a = sin(theta) / theta;
            b = (1.0 - cos(theta))/(theta*theta);
        }

        Eigen::Matrix3d Omega2 = Omega*Omega;

        R = (Eigen::Matrix3d::Identity()
             + a*Omega
             + b*Omega2);

        return R;
    }

    static Eigen::Matrix4d exp(const Eigen::Vector6d &u) {
        Eigen::Vector3d omega;
        for (int i=0; i<3; i++)
            omega[i]=u[i];
        Eigen::Vector3d upsilon;
        for (int i=0; i<3; i++)
            upsilon[i]=u[i+3];

        double theta = omega.norm();
        Eigen::Matrix3d Omega = skew(omega);

        Eigen::Matrix3d R;
        Eigen::Matrix3d V;
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;

        if (theta<0.00001)
        {
            double theta2 = theta*theta;
            double theta4 = theta2*theta2;
            a = 1.0 - theta2 / 6.0 + theta4 / 120.0;
            b = 0.5 - theta2 / 24.0 + theta4 / 720.0;
            c = 1.0 / 6.0 - theta2 / 120.0 + theta4 / 5040;
        }
        else
        {
            a = sin(theta) / theta;
            b = (1.0 - cos(theta))/(theta*theta);
            c = (theta - sin(theta))/(theta*theta*theta);
        }

        Eigen::Matrix3d Omega2 = Omega*Omega;

        R = (Eigen::Matrix3d::Identity()
             + a*Omega
             + b*Omega2);

        V = (Eigen::Matrix3d::Identity()
             + b*Omega
             + c*Omega2);

        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        ret.block<3, 3>(0, 0) = R;
        ret.block<3, 1>(0, 3) = V*upsilon;

        return ret;
    }

    static Eigen::Vector6d log(const Eigen::Matrix4d &m) {
        Eigen::Vector6d res;
        Eigen::Matrix3d _R = m.block<3, 3>(0, 0);
        Eigen::Vector3d _t = m.block<3, 1>(0, 3);
        double d =  0.5*(_R(0,0)+_R(1,1)+_R(2,2)-1);
        Eigen::Vector3d omega;
        Eigen::Vector3d upsilon;


        Eigen::Vector3d dR = deltaR(_R);
        Eigen::Matrix3d V_inv;

        if (d>0.99999)
        {

            omega=0.5*dR;
            Eigen::Matrix3d Omega = skew(omega);
            V_inv = Eigen::Matrix3d::Identity()- 0.5*Omega + (1./12.)*(Omega*Omega);
        }
        else
        {
            double theta = acos(d);
            omega = theta/(2*sqrt(1-d*d))*dR;
            Eigen::Matrix3d Omega = skew(omega);
            V_inv = ( Eigen::Matrix3d::Identity() - 0.5*Omega
                      + ( 1-theta/(2*tan(theta/2)))/(theta*theta)*(Omega*Omega) );
        }

        upsilon = V_inv*_t;
        for (int i=0; i<3;i++){
            res[i]=omega[i];
        }
        for (int i=0; i<3;i++){
            res[i+3]=upsilon[i];
        }

        return res;
    }

    static Eigen::Vector3d log(const Eigen::Matrix3d &R) {
        Eigen::Vector3d dR = deltaR(R);
        return 0.5 * dR;
    }

    static Eigen::Vector3d deltaR(const Eigen::Matrix3d &R) {
        Eigen::Vector3d v;
        v(0)=R(2,1)-R(1,2);
        v(1)=R(0,2)-R(2,0);
        v(2)=R(1,0)-R(0,1);
        return v;
    }

    static Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
        Eigen::Matrix3d ret;
        ret <<	0,		-v(2),	v(1),
                v(2),	0,		-v(0),
                -v(1),	v(0),	0;
        return ret;
    }

    static Eigen::Matrix4d inv(const Eigen::Matrix4d &mat) {
        Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
        inv.block<3, 3>(0, 0) = mat.block<3, 3>(0, 0).transpose();
        inv.block<3, 1>(0, 3) = -inv.block<3, 3>(0, 0) * mat.block<3, 1>(0, 3);

        return inv;
    }

    static double sinc(double x) {
        if(fabs(x) < 1e-5){
            // using Taylor expansion
            double x2 = x*x;
            return 1.0 - x2/6.0 + x2*x2/120.0;
        }
        else {
            return sin(x)/x;
        }
    }


    static Eigen::Matrix3d rotateByY(Eigen::Matrix3d matrix, Eigen::Vector3d axis) {
        axis = matrix.inverse() * axis;

        // Getting log-map parameters
        double theta = acos(fabs(axis(2)));
        double sincVal = sinc(theta);
        Eigen::Vector3d vec = Eigen::Vector3d::Zero();
        vec(1) = axis(0) / sincVal;

        Eigen::Matrix3d correctionMatrix = LieAlgebra::exp(vec);

        return correctionMatrix;
    }

};

#endif //LOAM_VELODYNE_LIEALEGEBRA_H
