//  Created by Igor Kroitor on 29/12/15.

#include <stdio.h>

//-----------------------------------------------------------------------------
// Gilbert-Johnson-Keerthi (GJK) collision detection algorithm in 2D
// http://mollyrocket.com/849
// http://www.dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/
//-----------------------------------------------------------------------------

struct _vec2 { float x; float y; };
typedef struct _vec2 vec2;

//-----------------------------------------------------------------------------

vec2 subtract (vec2 a, vec2 b) { a.x -= b.x; a.y -= b.y; return a; }
vec2 negate (vec2 v) { v.x = -v.x; v.y = -v.y; return v; }
vec2 perpendicular (vec2 v) { vec2 p = { v.y, -v.x }; return p; }
float dotProduct (vec2 a, vec2 b) { return a.x * b.x + a.y * b.y; }
float lengthSquared (vec2 v) { return v.x * v.x + v.y * v.y; }

//-----------------------------------------------------------------------------

vec2 tripleProduct (vec2 a, vec2 b, vec2 c) {
    
    vec2 r;
    
    float ac = a.x * c.x + a.y * c.y; // perform a.dot(c)
    float bc = b.x * c.x + b.y * c.y; // perform b.dot(c)
    
    // perform b * a.dot(c) - a * b.dot(c)
    r.x = b.x * ac - a.x * bc;
    r.y = b.y * ac - a.y * bc;
    return r;
}

//-----------------------------------------------------------------------------

vec2 averagePoint (const vec2 * vertices, size_t count) {
    vec2 avg = { 0.f, 0.f };
    for (size_t i = 0; i < count; i++) {
        avg.x += vertices[i].x;
        avg.y += vertices[i].y;
    }
    avg.x /= count;
    avg.y /= count;
    return avg;
}

//-----------------------------------------------------------------------------

size_t indexOfFurthestPoint (const vec2 * vertices, size_t count, vec2 d) {
    
    float maxProduct = dotProduct (d, vertices[0]);
    size_t index = 0;
    for (size_t i = 0; i < count; i++) {
        float product = dotProduct (d, vertices[i]);
        if (product > maxProduct) {
            maxProduct = product;
            index = i;
        }
    }
    return index;
}

//-----------------------------------------------------------------------------

vec2 support (const vec2 * vertices1, size_t count1,
              const vec2 * vertices2, size_t count2, vec2 d) {

    size_t i = indexOfFurthestPoint (vertices1, count1, d);
    size_t j = indexOfFurthestPoint (vertices2, count2, negate (d));
    return subtract (vertices1[i], vertices2[j]);
}

//-----------------------------------------------------------------------------

int gjk (const vec2 * vertices1, size_t count1,
         const vec2 * vertices2, size_t count2) {
    
    size_t index = 0; // index of current vertex of simplex
    vec2 a, b, c, d, ao, ab, ac, abperp, acperp, simplex[3];
    
    vec2 position1 = averagePoint (vertices1, count1); // rough average
    vec2 position2 = averagePoint (vertices2, count2);

    // initial direction from the center of 1st body to the center of 2nd body
    d = subtract (position1, position2);
    
    // if initial direction is zero – set it to any arbitrary axis (we choose X)
    if ((d.x == 0) && (d.y == 0))
        d.x = 1.f;
    
    // set the first support as initial point of the new simplex
    a = simplex[0] = support (vertices1, count1, vertices2, count2, d);
    
    if (dotProduct (a, d) <= 0)
        return 0; // no collision
    
    d = negate (d); // we will be searching in the opposite direction next
    
    while (1) {
        
        a = simplex[++index] = support (vertices1, count1, vertices2, count2, d);
        
        if (dotProduct (a, d) <= 0)
            return 0; // no collision
        
        ao = negate (a); // from point A to Origin is just negative A
        
        // simplex has 2 points (a line segment, not a triangle yet)
        if (index < 2) {
            b = simplex[0];
            ab = subtract (b, a); // from point A to B
            d = tripleProduct (ab, ao, ab); // normal to AB
            if (lengthSquared (d) == 0)
                d = perpendicular (ab);
            continue; // skip to next iteration
        }
        
        b = simplex[1];
        c = simplex[0];
        ab = subtract (b, a); // from point A to B
        ac = subtract (c, a); // from point A to C
        
        acperp = tripleProduct (ab, ac, ac);
        
        if (dotProduct (acperp, ao) >= 0) {
            
            d = acperp; // new direction is normal to AC
            simplex[1] = simplex[2]; // swap element in the middle (point B)
            --index;
            
        } else {
            
            abperp = tripleProduct (ac, ab, ab);
            
            if (dotProduct (abperp, ao) < 0)
                return 1; // collision
            
            simplex[0] = simplex[1]; // swap first element (point C)
            simplex[1] = simplex[2]; // swap element in the middle (point B)
            --index;

            d = abperp; // new direction is normal to AB
        }
    }
    
    return 0;
}

//-----------------------------------------------------------------------------

int main(int argc, const char * argv[]) {
    
    vec2 vertices1[] = {
        { 4, 11 },
        { 4, 5 },
        { 9, 9 },
    };
    
    vec2 vertices2[] = {
        { 5, 7 },
        { 7, 3 },
        { 10, 2 },
        { 12, 7 },
    };

    size_t count1 = sizeof (vertices1) / sizeof (vec2); // == 3
    size_t count2 = sizeof (vertices2) / sizeof (vec2); // == 4
    
    int collisionDetected = gjk (vertices1, count1, vertices2, count2);
    
    printf (collisionDetected ? "Bodies collide!\n" : "No collision\n");
    return 0;
}
