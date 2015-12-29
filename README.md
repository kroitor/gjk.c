# gjk.c – Gilbert-Johnson-Keerthi in plain C
This is a rough but fast implementation of GJK collision detection algorithm in plain C. It is in 2D for now, full 3D version is upcoming (soon)... It uses Minkowski sums and builds a simplex (a triangle) in Minkowski space to tell if two arbitrary polygons are colliding. 

## Disclaimer
Fuck all licenses and licensees. I made it for my learning purposes and it's absolutely free for absolutely any usage.

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
