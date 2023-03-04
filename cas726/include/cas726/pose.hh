#ifndef CAS726_POSE_HH
#define CAS726_POSE_HH

#include <cmath>

namespace cas726 {
/** Helper class for dealing with 2D poses */
    class Pose2 {
    public:
        float x,y,theta;
        Pose2():x(0),y(0),theta(0) { }
        Pose2(float _x, float _y, float _theta): x(_x),y(_y),theta(_theta) {}

        Pose2(const Pose2& other) {
            x=other.x;
            y=other.y;
            theta=other.theta;
        }

        //operator overloading
        Pose2 operator+(Pose2 &other) {
            Pose2 ps;
            ps.x = x+other.x;
            ps.y = y+other.y;
            ps.theta = theta+other.theta;
            ps.normalize();
            return ps;
        }
        Pose2 operator-(Pose2 &other) {
            Pose2 ps;
            ps.x = x-other.x;
            ps.y = y-other.y;
            ps.theta = theta-other.theta;
            ps.normalize();
            return ps;
        }
        Pose2& operator=(Pose2 &other) {
            this->x = other.x;
            this->y = other.y;
            this->theta = other.theta;
            return *this;
        }

        //normalizes theta between -PI and PI
        void normalize() {
            theta = atan2f(sin(theta),cos(theta));
        }

        //helper function, useful outside. computes the difference between two angles
        static float angle_diff(float &theta_a, float &theta_b) {
            float t = theta_a - theta_b;	      
            //normalize difference
            return fabsf(atan2f(sin(t),cos(t)));
        }

        static float normalize(const float &a) {
            return atan2f(sin(a),cos(a));
        }
    };
}
#endif
