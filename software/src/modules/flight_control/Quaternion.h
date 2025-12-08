#pragma once
#include <cmath>

struct Quaternion {
    float w;
    float x;
    float y;
    float z;

    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternion fromArray(const float q[4]) {
        return Quaternion(q[0], q[1], q[2], q[3]);
    }

    void toArray(float q[4]) const {
        q[0] = w; q[1] = x; q[2] = y; q[3] = z;
    }

    // Quaternion-Multiplikation: this * other
    Quaternion operator*(const Quaternion& o) const {
        return Quaternion(
            w*o.w - x*o.x - y*o.y - z*o.z,
            w*o.x + x*o.w + y*o.z - z*o.y,
            w*o.y - x*o.z + y*o.w + z*o.x,
            w*o.z + x*o.y - y*o.x + z*o.w
        );
    }

    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    void normalize() {
        float n2 = w*w + x*x + y*y + z*z;
        if (n2 <= 0.0f) {
            w = 1.0f; x = y = z = 0.0f;
            return;
        }
        float inv = 1.0f / std::sqrt(n2);
        w *= inv; x *= inv; y *= inv; z *= inv;
    }

    // Aus Roll, Pitch, Yaw (rad) Quaternion erzeugen (Z-Y-X Konvention: yaw, pitch, roll)
    static Quaternion fromEuler(float roll, float pitch, float yaw) {
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);

        Quaternion q;
        q.w = cy*cp*cr + sy*cp*sr * -1.0f + cy*sp*sr * 0.0f + sy*sp*cr * 0.0f; // wir machen’s sauberer unten
        // besser direkt Standardform:
        q.w = cy*cp*cr + sy*sp*sr;
        q.x = cy*cp*sr - sy*sp*cr;
        q.y = sy*cp*sr + cy*sp*cr;
        q.z = sy*cp*cr - cy*sp*sr;

        q.normalize();
        return q;
    }

    // Euler aus Quaternion (für Debug / Altitude / Yaw-Handling)
    void toEuler(float& roll, float& pitch, float& yaw) const {
        // Roll (x)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y)
        float sinp = 2.0f * (w * y - z * x);
        if (std::fabs(sinp) >= 1.0f)
            pitch = std::copysign(3.14159265358979f / 2.0f, sinp); // ±90°
        else
            pitch = std::asin(sinp);

        // Yaw (z)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
};
