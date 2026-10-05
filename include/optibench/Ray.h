#pragma once
#include "Vec2.h"

namespace ob {
    // data component of a ray
    struct Ray {
        // 2D point storing the genesis of the ray
        Vec2 origin;

        // 2D vector showing the direction of the ray
        Vec2 direction;
    };


    // data component tracking ray collisions
    struct Hit {
        // true if the ray collides with an object
        bool hit = false;

        // distance from ray's starting point to point of collision
        double t = 0.0;

        // 2D coordinates of the exact point of collision
        Vec2 point;

        // 2D vector pointing straight outward from the surface of the collision object
        Vec2 normal;
    };
}