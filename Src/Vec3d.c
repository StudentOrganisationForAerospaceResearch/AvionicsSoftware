#include "Vec3d.h";

double vectorMagnitude(struct Vec3d vector) {
    return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

struct Vec3d vectorZero() {
    struct Vec3d rt;
    rt.x = 0;
    rt.y = 0;
    rt.z = 0;
    rt.vb = 1;
    return rt;
}

struct Vec3d vectorAdd(struct Vec3d v1, struct Vec3d v2) {
    struct Vec3d rt;
    rt.x = v1.x + v2.x;
    rt.y = v1.y + v2.y;
    rt.z = v1.z + v2.z;
    rt.vb = 1;
    return rt;
}

struct Vec3d vectorSub(struct Vec3d v1, struct Vec3d v2) {
    struct Vec3d rt;
    rt.x = v1.x - v2.x;
    rt.y = v1.y - v2.y;
    rt.z = v1.z - v2.z;
    rt.vb = 1;
    return rt;
}

struct Vec3d vectorMult(struct Vec3d v1, struct Vec3d v2) {
    struct Vec3d rt;
    rt.x = v1.x * v2.x;
    rt.y = v1.y * v2.y;
    rt.z = v1.z * v2.z;
    rt.vb = 1;
    return rt;
}

struct Vec3d vectorMultScalar(double scalar, struct Vec3d v) {
    struct Vec3d rt;
    rt.x = v.x * scalar;
    rt.y = v.y * scalar;
    rt.z = v.z * scalar;
    rt.vb = 1;
    return rt;
}

struct Vec3d vectorUniformScale(double scalar, struct Vec3d v) {
    struct Vec3d rt;
    rt.x = v.x + scalar;
    rt.y = v.y + scalar;
    rt.z = v.z + scalar;
    rt.vb = 1;
    return rt;
}
