#pragma once

#include <math.h>

struct Vec3d
{
    double x;
    double y;
    double z;

    // TODO is there a better way to do this????
    // valid bit set to -1 if the vector is not valid
    int vb;
};

double vectorMagnitude(struct Vec3d vector);

struct Vec3d vectorZero();

struct Vec3d vectorAdd(struct Vec3d v1, struct Vec3d v2);

struct Vec3d vectorSub(struct Vec3d v1, struct Vec3d v2);

struct Vec3d vectorMult(struct Vec3d v1, struct Vec3d v2);

struct Vec3d vectorMultScalar(double scalar, struct Vec3d v);

struct Vec3d vectorUniformScale(double scalar, struct Vec3d v);

