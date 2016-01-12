# gjk.c – Gilbert-Johnson-Keerthi in plain C
This is a rough but fast implementation of GJK collision detection algorithm in plain C. Just one C file, less then 200 lines, no dependencies. It is in 2D for now, full 3D version is upcoming... This 2D-version uses Minkowski sums and builds a triangle-simplex in Minkowski space to tell if two arbitrary convex polygons are colliding. 3D-version will be roughly the same, but will build a tetrahedron-simplex inside a 3-dimensional Minkowski space. It currently only tells if there is a collision or not. C-code for distance and contact points coming soon.

## Disclaimer
Fuck all licenses and copyright. I made it for learning purposes, this is public knowledge and it's absolutely free for any usage.

## Usage example
This is an illustration of the example case from [dyn4j](http://www.dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/).

![Example case from dyn4j](http://www.dyn4j.org/wp-content/uploads/2010/04/gjk-figure1.png "Example case from dyn4j")

The two tested polygons are defined as arrays of plain C vector struct type. This implementation of GJK doesn't really care about the order of the vertices in the arrays, as it treats all sets of points as *convex polygons*.

```c
struct _vec2 { float x; float y; };
typedef struct _vec2 vec2;

...

int main(int argc, const char * argv[]) {
    
    // test case from dyn4j

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
```
## References (must see)
Most of the info (along with reference implementation) was taken from dyn4j. It has a very thorough explanation of GJK and it is definitely a must visit for those who need to understand the intricacies of the algorithm.

1. http://www.dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/ GJK description (+ a lot of other useful articles)
2. http://mollyrocket.com/849 Awesome old-school GJK / Minkowski space video
3. https://github.com/wnbittle/dyn4j Quality source code for reference
