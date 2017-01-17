# gjk.c – Gilbert-Johnson-Keerthi In Plain C
This is a rough but fast implementation of GJK collision detection algorithm in plain C. Just one C file, less then 200 lines, no dependencies. It is in 2D for now, full 3D version is upcoming... This 2D-version uses Minkowski sums and builds a triangle-simplex in Minkowski space to tell if two arbitrary convex polygons are colliding. 3D-version will be roughly the same, but will build a tetrahedron-simplex inside a 3-dimensional Minkowski space. It currently only tells if there is a collision or not. C-code for distance and contact points coming soon.

## Disclaimer
Fuck licenses and copyright. I made it for learning purposes, this is public knowledge and it's absolutely free for any usage.

## Usage Example
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
## How It Works

The goal of this explanation is to help people visualize the logic of GJK. To explain how it works we have to oversimplify certain things. There's no complicated math except basic arithmetic and a little bit of vector math, nothing more, so that a 3rd-grader could understand it.

At the very top level the GJK algorithm tells if two arbitrary shapes are intersecting (colliding) or not. A collision occurs when two shapes try to occupy the same points in space at the same time. The space can be of any nature. It might be your in-game world simulation, or a calculation on a table of statistical data, or a built-in navigation system for a robot or any other application you can imagine. You can use it for calculating collisions of solid bodies and numeric intersections of any kind. You can have as many dimensions as you want, the amount of dimensions does not really matter, the logic is the same for 1D, 2D, 3D, etc... With GJK you can even calculate collisions in 4D if you're able to comprehend this. The algorithm itself does not require any hard math whatsoever, it's actually very intuitive and easy. It takes an arithmetic difference of two shapes by subtracting all points of one shape from all points of another shape. If two shapes share a common point, subtracting that point from itself results in a difference of zero. So, if zero is found in the result then there's a collision. In fact, the algorithm does not have to calculate all differences for all possible pairs of points, only a small subset of significant points is examined, therefore it works very fast while being very accurate.

In order to understand GJK one has to build an imaginary visualization of what is going on under the hood. Once you see the picture in your head, you can implement it and even tweak it for your needs. 

### A 1D intro

Let's start with a naive example of computing a shape difference in one dimension. A segment of a number line is an example of a 1D-shape. Imagine we have two segments on the number line: segment `[1,3]` and segment `[2,4]`:

![Segment [1,3] on the number line](https://cloud.githubusercontent.com/assets/1294454/21998716/8f47b72e-dc47-11e6-836e-06d523a84105.jpg "Segment [1,3] on the number line")
![Segment [2,4] on the number line](https://cloud.githubusercontent.com/assets/1294454/21998717/8f487718-dc47-11e6-88f2-57bf590a0ee4.jpg "Segment [2,4] on the number line")

Zero is our point of reference on the number line, so we call it *the Origin*. Easy enough.
It is obvious that our segments occupy some common region of our 1D-space, so they must be intersecting or colliding, you can tell that just by looking at the representation of both segments in same 1D-space (on the same line). Let's confirm arithmetically that these segments indeed intersect. We do that by subtracting all points of segment `[2,3]` from all points of segment `[1,3]` to see what we get on a number line.
```
1 - 2 = -1
1 - 3 = -2
1 - 4 = -3

2 - 2 =  0
2 - 3 = -1
2 - 4 = -2

3 - 2 =  1
3 - 3 =  0
3 - 4 = -1
```
We got a lot of numbers, many of them more than once. The resulting set of points (numbers) is larger than each of the initial sets of points of two shapes. Let's plot these resulting points in our 1D-space and look at the shape of resulting segment on the number line:

![Segment [-3,1] on the number line](https://cloud.githubusercontent.com/assets/1294454/21998715/8f465a96-dc47-11e6-9aa5-e30f59453685.jpg "Segment [-3,1] on the number line")

So after all we got resulting segment `[-3,1]` which covers points `-3`, `-2`, `-1`, `0` and `1`. Because two initial shapes had some points in common the resulting segment contains a zero. This comes from a simple fact, that when you subtract a point (which is a number) from itself you inevitably end up with a zero. Note, that our initial segments had points 2 and 3 in common. When we subtracted 2 from 2 we got 0. When we subtracted 3 from 3 we also got a zero. This is quite obvious. So, if two shapes have at least one common point, because you subtract that point from itself, the resulting set must contain zero (the Origin) at least once. This is the key of GJK which says: if the Origin is contained inside the resulting set then initial shapes must have collided or kissed at least. Once you get it, you can then apply it to any number of dimensions.

Now let's take a look at a counter-example, say we have two segments `[-2,-1]` and `[1,3]`:
![Segment [-2,-1] on the number line](https://cloud.githubusercontent.com/assets/1294454/21998714/8f462616-dc47-11e6-9eec-b13454c7a67a.jpg "Segment [-2,-1] on the number line")
![Segment [1,3] on the number line](https://cloud.githubusercontent.com/assets/1294454/21998716/8f47b72e-dc47-11e6-836e-06d523a84105.jpg "Segment [1,3] on the number line")

We can visually ensure that segment `[-2,-1]` occupies a different region of number line than that of segment `[1,3]`. There's a gap  between our initial shapes, so these two shapes do not intersect in our 1D-space, therefore there's no collision. Let's prove that arithmetically by subtracting all points of any of the two segments from all points of the other.
```
1 - (-1) = 1 + 1 = 2
1 - (-2) = 1 + 2 = 3

2 - (-1) = 2 + 1 = 3
2 - (-2) = 2 + 2 = 4

3 - (-1) = 3 + 1 = 4
3 - (-2) = 3 + 2 = 5 
```
And we again draw the resulting segment on a number line in our imaginary 1D-space:

![Segment [2,5] on the number line](https://cloud.githubusercontent.com/assets/1294454/21998713/8f45117c-dc47-11e6-8e2a-a759b36a559d.jpg "Segment [2,5] on the number line")

We got another bigger segment `[2,5]` which represents a difference of all the points of two initial segments but this time it does not contain the Origin. That is, the resulting set of points does not include zero, because initial segments did not have any points in common so they indeed occupy different regions of our number line and don't intersect.

Now, if our initial shapes were too big (long initial segments) we would have to calculate too many differences from too many pairs of points. But it's actually easy to see, that we only need to calculate the difference between the endpoints of two segments, ignoring all the inside points of both segments.

Consider two segments `[10,20]` and `[5,40]`:

![Segment [10,20] on the number line](https://cloud.githubusercontent.com/assets/1294454/21999180/deb68374-dc49-11e6-9ef4-ea631b35178c.jpg "Segment [10,20] on the number line")
![Segment [5,40] on the number line](https://cloud.githubusercontent.com/assets/1294454/21999179/de9cceac-dc49-11e6-8ddf-656ec1829c43.jpg "Segment [5,40] on the number line")

Now subtract their four endpoints from each other:
```
10 - 5  =   5
10 - 40 = -30

20 - 5  =  15
20 - 40 = -20
```
The resulting segment `[-30,15]` would look like this:

![Segment [-30,15] on the number line](https://cloud.githubusercontent.com/assets/1294454/21999267/39333c70-dc4a-11e6-91ec-12a08ebb6f34.jpg "Segment [-30,15] on the number line")

Notice that when we subtract the *opposite points* of initial shapes from one another, the resulting difference number lands either to the left or to the right of the Origin (which is zero) on the number line. Say, we take the leftmost point of segment `[5,40]` (number `5`) and subtract it from the rightmost point of segment `[10,20]` (number `20`) the resulting number `20 - 5 = 15` is positive, so it lands to the right of zero on the number line. Then we take the rightmost point of segment `[5,40]` (number `40` this time) and subtract it from the leftmost point of segment `[10,20]` (number `10`) the resulting number `10 - 40 = -30` is negative, so it lands to the left of zero on the number line which is exactly opposite to the first positive difference point `15`. We say these points are *opposite* directionwise. Therefore the Origin is between two resulting points *enclosing* it.

We ignored all insignificant internal points and only took the endpoints of initial segments into account thus reducing our calculation to four basic arithmetic operations (subtractions). We did that by switching to a simpler representation of a segment (only two endpoints instead of all points contained inside an initial segment). A simpler representation of the difference of two shapes is called a *simplex*. It literally means 'the simplest possible'. The simplest possible *thing* on a one-dimensional number-line is a number (a point in 1D-space). A segment is the simplest possible shape which is sufficient to contain multiple points of a number line. Even if one segment covers the other segment in its entirety (a segment fully contains another segment) – you can still detect an intersection of them in space. And it does not matter which one you're subtracting from, the resulting set will still contain the Origin at zero.

GJK also works if two initial shapes don't intersect but just barely touch. Say, we have two shapes, segment `[1,2]` and segment `[2,3]`:

![Segment [1,2] on the number line](https://cloud.githubusercontent.com/assets/1294454/21999597/fbec71fe-dc4b-11e6-8afa-371f87457339.jpg "Segment [1,2] on the number line")
![Segment [2,3] on the number line](https://cloud.githubusercontent.com/assets/1294454/21999596/fbebff44-dc4b-11e6-811a-f832ab3eb9a4.jpg "Segment [2,3] on the number line")

These two segments only have one point in common. Subtracting their endpoints from each other gives:
```
1 - 2 = -1
1 - 3 = -2

2 - 2 =  0
2 - 3 = -1
```
And the resulting difference looks like:

![Segment [-2,0] on the number line](https://cloud.githubusercontent.com/assets/1294454/21999595/fbe9927c-dc4b-11e6-8521-816589df8552.jpg "Segment [-2,0] on the number line")

Notice, that in case of only one common point the Origin is not inside resulting segment, but is actually one of its endpoints (the rightmost in this example). When your resulting segment has the Origin at one of its endpoints that means that your initial shapes do not collide, but merely touch at a single point (two segments have only one common point of intersection).

What GJK really says is: if you're able to build a simplex that contains (includes) the Origin then your shapes have at least one or more points of intersection (occupy same points in space).

### Moving On To 2D

Now let's take a look at the picture in 2D. Our 2D-space is now an xy-plane (which is represented by two orthogonal number lines instead of a single number line). Every point in our 2D-space now has two xy-coordinates instead of one number, that is, each point is now a 2D-vector. Suppose we have two basic 2D-shapes – a rectangle `ABCD` intersecting a triangle `EFG` on a plane.

![Rectangle ABCD and triangle EFG on 2D xy-plane](https://cloud.githubusercontent.com/assets/1294454/22000794/450f715a-dc52-11e6-910f-4b548ab97c2d.jpg "Rectangle ABCD and triangle EFG on 2D xy-plane")

These shapes are represented by the following sets of points (2D-vectors, which are pairs of xy-coordinates):
```
A (1, 3)
B (5, 3)
C (5, 1)
D (1, 1)

E (2, 4)
F (4, 4)
G (3, 2)
```
Now, because we have much more numbers here, the arithmetic becomes a little more involved, but its still very easy – we literally subtract all points of one shape from all points of another shape one by one.
```
A - E = (1 - 2, 3 - 4) = (-1, -1)
A - F = (1 - 4, 3 - 4) = (-2, -1)
A - G = (1 - 3, 3 - 2) = (-2,  1)

B - E = (5 - 2, 3 - 4) = ( 3, -1)
B - F = (5 - 4, 3 - 4) = ( 1, -1)
B - G = (5 - 3, 3 - 2) = ( 2,  1)

C - E = (5 - 2, 1 - 4) = ( 3, -3)
C - F = (5 - 4, 1 - 4) = ( 1, -3)
C - G = (5 - 3, 1 - 2) = ( 2, -1)

D - E = (1 - 2, 1 - 4) = (-1, -3)
D - F = (1 - 4, 1 - 4) = (-3, -3)
D - G = (1 - 3, 1 - 2) = (-2, -1)
```
After plotting all of 12 resulting points in our 2D-space and connecting the *outermost* points with lines, we get the following difference shape:

![Rectangle ABCD minus triangle EFG on 2D xy-plane](https://cloud.githubusercontent.com/assets/1294454/22000793/450e7e80-dc52-11e6-972c-4843338c3fad.jpg "Rectangle ABCD minus triangle EFG on 2D xy-plane")

GJK says that if we're able to enclose the Origin within the resulting shape, then two initial shapes must have collided.
We immediately see, that this shape actually contains the Origin. Therefore we can visually confirm that our initial rectangle `ABCD` indeed intersects our initial triangle `EFG`.

### Building The Simplex

Now, remember, in 1D to determine whether the resulting segment contains the Origin we check if a primitive inequality `leftEndpoint < Origin < rightEndpoint` holds. So we could think of it as if the segment *surrounds* the Origin from all sides of 1D space (from both left and right, as there are only 2 relative sides in one dimension on our sketch). In 1D for an object to be able to surround any point (the Origin is the zero point on a number line), that object must itself consist of at least two of its own points (in other words, it must be a segment defined by its two points on a number line). Therefore the 1D-version is trying to build a simplex of two endpoints (a segment on a number line). And then it checks whether the Origin is contained within the resulting segment.

In 2D the process is similar. We can immediately tell if the Origin is inside the resulting polygon shape just by a brief visual inspection. This is done by the natural wiring of human brains. But a machine cannot distinguish *inside* from *outside* unless you teach it how to do that. It's a lot easier for a machine to see if a point is contained inside a simple shape like a triangle or a circle,  rather than a more complex shape like a general polygon with N vertices or some irregular shape. Moreover, the algorithm has to do that in a fast but accurate way in order to be suitable for general-purpose collision detection.

And this is where the true power of simplicity of GJK comes into play. Think this way: you need at least two points in 1D to surround the Origin. But in 2D two points are not enough to surround anything on a plane. Two points define a single straight line on a plane, but a straight line cannot enclose anything, because it's a line and it is straight. Therefore in 2D you need at least three points connected by segments, that is a triangle, to be able to enclose at least some area of the plane. So the simplest possible shape that can *enclose* something in 2D is a triangle. Now our task of arithmetically enclosing the Origin within our resulting polygon shape simplifies a little bit. We have to find such three points from our resulting set of points, that make a triangle that encloses the Origin. GJK can build a triangle and test if a certain point lies within that triangle, so we just made this task solvable by a machine.

![Simplices in various spaces](https://cloud.githubusercontent.com/assets/1294454/22039058/c30ea898-dd0e-11e6-8e15-62a5b612036d.jpg "Simplices in various spaces")

After we subtracted our initial shapes one from another we got a resulting set of all of the points of a new shape that represents the difference of the initial shapes. The most straightforward way to build such an Origin-enclosing triangle from a given set of points is to start taking triples of points (combinations of three points) to see if they form a triangle with the Origin inside it. If a triple of points makes such a triangle, then we can conclude that the difference of two shapes contains the Origin, so initial shapes must have collided or intersected. If not, we take some other triple up until we run out of points. If none of the triples did enclose the Origin, then no such triangle was found and there was no collision.

So, if we randomly select any three of our points and connect them with line segments, we would probably end up with a triangle similar to one of the following:

![2-Simplex on a coordinate plane](https://cloud.githubusercontent.com/assets/1294454/22035222/38bca25c-dd00-11e6-9236-c93f8ba6d050.jpg "2-Simplex on a coordinate plane")
![2-Simplex on a coordinate plane](https://cloud.githubusercontent.com/assets/1294454/22035223/38d78054-dd00-11e6-99e3-e2e0c40dfe2c.jpg "2-Simplex on a coordinate plane")
![2-Simplex on a coordinate plane](https://cloud.githubusercontent.com/assets/1294454/22035224/38e5afda-dd00-11e6-903b-69b98087e5da.jpg "2-Simplex on a coordinate plane")
![2-Simplex on a coordinate plane](https://cloud.githubusercontent.com/assets/1294454/22035443/12889bb2-dd01-11e6-9d75-234dbf351722.jpg "2-Simplex on a coordinate plane")

WORK IN PROGRESS, to be continued soon... )

### The Support Function

#### A Word On Math

To calculate the difference between any two numbers we subtract any of them from the other like this: `A - B = C`. But in general when dealing with 2D or 3D coordinate vectors (which is usually the case in many applications) the order of subtraction actually does matter. A more accurate way of representing the difference of two vectors is to take the first vector and sum it with a negated version of the second vector, so that `B = -B` and then `A + B = C`. Because of this fact the Minkowski support function is often defined as a sum of two vectors where the second vector is negated, this is why the function is called *Minkowski addition* or *Minkwoski sum* and you will probably never hear of *Minkowski subtraction*.

WORK IN PROGRESS, to be continued soon... )

### Adding 3rd dimension

A careful reader might have already noticed a pattern in how the algorithm actually works.

For a single dimension (1D number line) we need a support function and a 1-simplex of two points to enclose the Origin. If we can find such two points then our shapes do intersect indeed. If we cannot find such two points then our initial shapes must have some distance (non-zero difference) between them. For two dimensions (2D coordinate plane) we need a support function and a 2-simplex of three points (a triangle) to enclose the Origin. To be able to enclose the Origin in three dimensions (3D space) we need a support function and a 3-simplex of four points (a tetrahedron).

WORK IN PROGRESS, to be continued soon... )

## References (must see)
Most of the info (along with reference implementation) was taken from dyn4j. It has a very thorough explanation of GJK and it is definitely a must visit for those who need to understand the intricacies of the algorithm.

1. http://www.dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/ GJK description (+ a lot of other useful articles)
2. http://mollyrocket.com/849 Awesome old-school GJK / Minkowski space video
3. https://github.com/wnbittle/dyn4j Quality source code for reference

## P.S. 3D-version coming soon...
![3D-version of GJK in plain C coming soon...](http://s21.postimg.org/da9txc3uv/Screen_Shot_2016_01_13_at_09_13_12.jpg "3D-version of GJK in plain C coming soon...")
