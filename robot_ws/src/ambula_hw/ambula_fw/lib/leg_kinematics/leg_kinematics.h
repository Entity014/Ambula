#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

#include <Arduino.h>
#include <math.h>

class LegKinematics
{
public:
    struct link_param
    {
        float L1;
        float L2;
        float L3; // wheel radius / toe offset
        float hip_x;
        float hip_y;
        float hip_z;
        float align_y;   // rad
        float waist_off; // rad
        float hip_off;   // rad
        float knee_off;  // rad
    };

    struct joint_state
    {
        float waist; // rad
        float hip;   // rad
        float knee;  // rad
    };

    struct foot_point
    {
        float x;
        float y;
        float z;
    };

    explicit LegKinematics(const link_param &p)
        : p_(p) {}

    foot_point getFootPosition(const joint_state &js) const
    {
        return calFootPosition(js);
    }

protected:
    foot_point calFootPosition(const joint_state &js) const
    {
        // apply offsets
        const float q1 = js.waist + p_.waist_off;
        const float q2 = js.hip + p_.hip_off;
        const float q3 = js.knee + p_.knee_off;

        const float c1 = cosf(q1), s1 = sinf(q1);
        const float ca = cosf(p_.align_y), sa = sinf(p_.align_y);

        // v_plane = Ry(q2)*[L1,0,0] + Ry(q2+q3)*[(L2+L3),0,0]
        const float t23 = q2 + q3;

        const float vx = p_.L1 * cosf(q2) + (p_.L2 + p_.L3) * cosf(t23);
        const float vz = -p_.L1 * sinf(q2) - (p_.L2 + p_.L3) * sinf(t23);

        // R_pre = Ry(align)*Rz(q1), and v_plane has y=0
        const float x_rot = ca * (vx * c1) + sa * (vz);
        const float y_rot = vx * s1;
        const float z_rot = -sa * (vx * c1) + ca * (vz);

        foot_point fp;
        fp.x = p_.hip_x + x_rot;
        fp.y = p_.hip_y + y_rot;
        fp.z = p_.hip_z + z_rot;
        return fp;
    }

private:
    link_param p_;
};

#endif
