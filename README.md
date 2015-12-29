# gjk.c – Gilbert-Johnson-Keerthi in plain C
This is a rough but fast implementation of GJK collision detection algorithm in plain C. It is in 2D for now, full 3D version is upcoming... This 2D-version uses Minkowski sums and builds a triangle-simplex in Minkowski space to tell if two arbitrary polygons are colliding. 3D-version will be roughly the same, but will build a tetrahedron-simplex inside a 3-dimensional Minkowski space. It currently only tells if there is a collision or not. C code for distance and contact points coming soon.

## Disclaimer
Fuck all licenses and fuck copyright. I made it for my learning purposes, it's public knowledge and it's absolutely free for any usage.

## Usage example
```
int main(int argc, const char * argv[]) {
    
    vec2 vertices1[] = {
        { 4, 11 },
        { 4, 5 },
        { 9, 5 },
    };
    
    vec2 vertices2[] = {
        { 7, 7 },
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
```
## References (must see)
Most of the info (along with reference implementation) was taken from dyn4j:

1. http://www.dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/
2. http://mollyrocket.com/849
3. https://github.com/wnbittle/dyn4j
