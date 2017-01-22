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

The goal of this explanation is to help people visualize the logic of GJK. To explain it we have to oversimplify certain things. There's no complicated math except basic arithmetic and a little bit of vector math, nothing more, so that a 3rd-grader could understand it. It is actually not very difficult to have GJK algorithm explained properly in an understandable form.

At the very top level GJK tells if two arbitrary shapes are intersecting (colliding) or not. The algorithm is used to calculate the depth of intersection (collision distance). A collision occurs when two shapes try to occupy the same points in space at the same time. The space can be of any nature. It might be your in-game world simulation, or a calculation on a table of statistical data, or a built-in navigation system for a robot or any other application you can imagine. You can use it for calculating collisions of solid bodies and numeric intersections of any kind. You can have as many dimensions as you want, the amount of dimensions does not really matter, the logic is the same for 1D, 2D, 3D, etc... With GJK you can even calculate collisions in 4D if you're able to comprehend this. 

The algorithm itself does not require any hard math whatsoever, it's actually very intuitive and easy. It takes an arithmetic difference of two shapes by subtracting all points of one shape from all points of another shape. If two shapes share a common point, subtracting that point from itself results in a difference of zero. So, if zero is found in the result then there's a collision. In fact, the algorithm does not have to calculate all differences for all possible pairs of points, only a small subset of significant points is examined, therefore it works very fast while being very accurate.

In order to understand GJK one has to build an imaginary visualization of what is going on under the hood. Once you see the picture in your head, you can implement it and even tweak it for your needs. 

### A 1D Intro

Let's start with a naive example of computing a difference of two shapes in one dimension. A segment of a number line is an example of 1D-shape. Imagine we have two segments on the number line: segment `[1,3]` and segment `[2,4]`:

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
We got a lot of numbers, many of them more than once. The resulting set of points (numbers) is larger than each of our initial sets of points of two shapes. Let's plot these resulting points in our 1D-space and look at the shape of resulting segment on the number line:

![Segment [-3,1] on the number line](https://cloud.githubusercontent.com/assets/1294454/21998715/8f465a96-dc47-11e6-9aa5-e30f59453685.jpg "Segment [-3,1] on the number line")

So after all we have resulting segment `[-3,1]` which covers points `-3`, `-2`, `-1`, `0` and `1`. Because two initial shapes had some points in common the resulting segment contains a zero. This comes from a simple fact, that when you subtract a point (which is a number) from itself you inevitably end up with a zero. Note, that our initial segments had points 2 and 3 in common. When we subtracted 2 from 2 we got 0. When we subtracted 3 from 3 we also got a zero. This is quite obvious. So, if two shapes have at least one common point, because you subtract that point from itself, the resulting set must contain zero (the Origin) at least once. This is the key of GJK which says: if the Origin is contained inside the resulting set then initial shapes must have collided or kissed at least. Once you get it, you can then apply it to any number of dimensions.

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

Now, if our initial shapes were too big (long initial segments) we would have to calculate too many differences from too many pairs of points. But it's actually easy to see, that we only need to calculate the intersection between the endpoints of two segments, ignoring all the inside points of both segments.

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

We ignored all insignificant internal points and only took the endpoints of initial segments into account thus reducing our calculation to four basic arithmetic operations (subtractions). We did that by switching to a simpler representation of a segment (only two endpoints instead of all points contained inside an initial segment). A simpler representation of the difference of two shapes is called a *simplex*. It literally means 'the simplest possible'. The simplest possible *thing* on a one-dimensional number-line is a number (a single point in 1D-space is called a *0-simplex*). A segment is the simplest possible shape which is sufficient to contain multiple points of a number line (a segment of two points in 1D space is called a *1-simplex*). Even if one segment covers the other segment in its entirety (a segment fully contains another segment) – you can still detect an intersection of them in space. And it does not matter which one you're subtracting from, the resulting set will still contain the Origin at zero.

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

WORK IN PROGRESS, A live demo of GJK in a 1D-space and a video of GJK in action coming up soon )

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
After plotting all of 12 resulting points in our 2D-space and connecting the *outermost* points with lines we get the following difference shape:

![Rectangle ABCD minus triangle EFG on 2D xy-plane](https://cloud.githubusercontent.com/assets/1294454/22000793/450e7e80-dc52-11e6-972c-4843338c3fad.jpg "Rectangle ABCD minus triangle EFG on 2D xy-plane")

GJK says that if we're able to enclose the Origin within the resulting shape then two initial shapes must have collided.
We immediately see that this shape actually contains the Origin. Therefore we can visually confirm that our initial rectangle `ABCD` indeed intersects our initial triangle `EFG`. 

Also note, that we subtracted all points from all other points (calculated all combinations of pairs) which yields some points inside our resulting shape. But as with 1D, we will optimise the algorithm to skip all internal points entirely.

### Building The Simplex

Now, remember, in 1D to determine whether the resulting segment contains the Origin we check if a primitive inequality `leftEndpoint < Origin < rightEndpoint` holds. We can think of it as if the segment *surrounds* the Origin from all sides of 1D space (from both left and right, as there are only 2 relative sides in one dimension on our sketch). In 1D for an object to be able to surround any point (the Origin is the zero point on a number line), that object must itself consist of at least two of its own points (in other words, it must be a segment defined by its two points on a number line). Therefore the 1D-version is trying to build a simplex of two endpoints (a segment on a number line). And then it checks whether the Origin is contained within the resulting segment.

![Simplices in various spaces](https://cloud.githubusercontent.com/assets/1294454/22043033/dd0fea8a-dd1e-11e6-9ec7-dd2c6103291f.jpg "Simplices in various spaces")

In 2D the process is similar. We can immediately tell if the Origin is inside the resulting polygon shape just by a brief visual inspection. This is done by the natural wiring of human brains. But a machine cannot distinguish *inside* from *outside* unless you teach it how to do that. It's a lot easier for a machine to see if a point is contained inside a simple shape like a triangle or a circle,  rather than a more complex shape like a general polygon with N vertices or some arbitrary irregular shape. Moreover, the algorithm has to do that in a fast but accurate way in order to be suitable for general-purpose collision detection.

And this is where the true power of simplicity of GJK comes into play. Think this way: you need at least two points in 1D to surround the Origin. So a 1D segment of two points is a 1-simplex. But in 2D two points are not enough to surround anything on a plane. Two points define a single straight line on a plane, but a straight line cannot enclose anything, because it's a line and it is straight. Therefore in 2D you need at least three points connected by segments, that is a triangle, to be able to enclose at least some area of the plane. The simplest possible shape that can *enclose* something in 2D is a triangle also known as *2-simplex*. 

Now our task of arithmetically enclosing the Origin within our resulting shape simplifies a little bit. We have to find such three points from our resulting set of points, that make a triangle that encloses the Origin. In other words, we need to build a 2-simplex that would satisfy our criteria. GJK can build a triangle and test if a certain point lies within that triangle, so we just made this task solvable by a machine.

After we subtracted our initial shapes one from another we got a resulting set of all of the points of a new shape that represents the difference of initial shapes. The most straightforward way to build such an Origin-enclosing triangle from a given set of points is to start taking triples of points (combinations of three points) to see if they form a triangle with the Origin inside it. If a triple of points makes such a triangle, then we can conclude that the difference of two shapes contains the Origin, so initial shapes must have collided or intersected. If not, we try some other triple, and that is done in a loop until we run out of points. If none of the triples did enclose the Origin, then no such triangle was found and there was no collision at all.

So, if we randomly select any three of our points and connect them with line segments, we will probably end up with a triangle similar to one of the following:

![2-Simplex on a coordinate plane](https://cloud.githubusercontent.com/assets/1294454/22035222/38bca25c-dd00-11e6-9236-c93f8ba6d050.jpg "2-Simplex on a coordinate plane")
![2-Simplex on a coordinate plane](https://cloud.githubusercontent.com/assets/1294454/22035223/38d78054-dd00-11e6-99e3-e2e0c40dfe2c.jpg "2-Simplex on a coordinate plane")
![2-Simplex on a coordinate plane](https://cloud.githubusercontent.com/assets/1294454/22035224/38e5afda-dd00-11e6-903b-69b98087e5da.jpg "2-Simplex on a coordinate plane")
![2-Simplex on a coordinate plane](https://cloud.githubusercontent.com/assets/1294454/22035443/12889bb2-dd01-11e6-9d75-234dbf351722.jpg "2-Simplex on a coordinate plane")

All of these triangles satisfy our criteria (all of them contain the Origin), so if we randomly select one of these and find the Origin inside, then we can immediately tell that there was a collision. But if we selected a triple of points that does not contain the Origin we might end up with something like this:

![Bad 2-Simplex on a coordinate plane](https://cloud.githubusercontent.com/assets/1294454/22040185/12e3d646-dd13-11e6-9f41-61f671e254ae.jpg "Bad 2-Simplex on a coordinate plane")
![Bad 2-Simplex on a coordinate plane](https://cloud.githubusercontent.com/assets/1294454/22040186/12e6f074-dd13-11e6-9dfc-6831d4f284d1.jpg "Bad 2-Simplex on a coordinate plane")

As long as the choice of points for our 2-simplex is random we might have to try all possible triples in worst case. Random walk is not the best strategy for finding a triangle that satisfies our criteria, there's a better algorithm for that ;) The goal of the algorithm is to find a combination of the best three points that enclose the Origin out of all possible triples of points, if such a combination exists at all. If such a triple does not exist then there is some distance between our initial shapes. The search for a simplex is the core of GJK.

In order to build the simplex efficiently the algorithm introduces a special routine that calculates the difference of two points of initial shapes. It is called a *support function* and it is the workhorse of the search.

#### The Support Function

Remember that in a 1D-space to obtain the resulting 1-simplex you subtract points from one another. The algorithm can skip 'internal' points and only compute the difference of outermost opposite points. The support function calculates the difference of opposite points in a more general way. As a bonus it also allows *flat vs curved* collisions! With it you can detect intersections of ellipses, circles, curves and splines in 2D and rounded shapes and more complex objects in 3D (which is cool).

Let's think of opposite points in two dimensions. Take a look at these examples of opposite points of a 2D-shape:

![Opposite points of a shape in 2D](https://cloud.githubusercontent.com/assets/1294454/22091779/cf553a08-de09-11e6-8161-c6abbbc9bd1b.jpg "Opposite points of a shape in 2D")
![Opposite points of a shape in 2D](https://cloud.githubusercontent.com/assets/1294454/22092331/f897542e-de0d-11e6-9127-c5c4f178b56a.jpg "Opposite points of a shape in 2D")

That is trivial. Now imagine you take not one but two arbitrary shapes and pick a random point of the first shape then pick a second point on the opposite side of the other shape. You might end up with something similar to this:

![Opposite points of two intersecting shapes in 2D](https://cloud.githubusercontent.com/assets/1294454/22092276/8fad1020-de0d-11e6-8287-3f43d05530ea.jpg "Opposite points of two intersecting shapes in 2D")
![Opposite points of two non-intersecting shapes in 2D](https://cloud.githubusercontent.com/assets/1294454/22092386/564bbb28-de0e-11e6-835d-e31e65788e3a.jpg "Opposite points of two non-intersecting shapes in 2D")

A difference of two points yields another point of resulting shape. That point is literally the distance vector. So, if you take a point of a shape and then choose a point of the other shape and then subtract the two points from one another, you get exact distance and direction between your shapes (at specific points). 

Note, that when you subtract *opposite* points of two shapes your resulting point-vector will always land somewhere on the contour (an outermost edge) of your resulting shape. Below is an illustration of how a difference of two opposite points (on the left) finally lands on the contour of resulting shape (on the right).

![Difference of opposite points projected into 2D Minkowski Space](https://cloud.githubusercontent.com/assets/1294454/22177875/724dd816-e038-11e6-8ed0-f28746cffc35.jpg "Difference of opposite points projected into 2D Minkowski Space")

The left side of the picture shows our simulated world space. On the right is our 2D *Minkowski space*. In GJK you can think of Minkowski space as if it was an imaginary world of shape differences. By subtracting two points in real world we therefore obtain a new point in another surreal world. It is also often called a *mapping* or a *projection* of a real-world intersection into Minkowski space (into the world of differences). The resulting shape in Minkowski space looks like a weird inside-out union of all points of the two initial shapes as if one shape is *"swept"* along the contour of the other. But this is how all intersections really look like in 2D. 

It's easy to show arithmetically that the resulting point is obtained by taking the difference of the two opposing points:

```
A(x1, y1) - B(x2, y2) = C(x1 - x2, y1 - y2)

 A(3,  1) -  B(1, -1) =  C(3 - 1, 1 - (-1)) = C(2, 2)
```

Resulting point `(2, 2)` ends up exactly on the contour of our difference shape. If initial points are opposite their difference will always be one of the outermost (*"external"*) points of the resulting shape. Also notice, that resulting distance vector `(2, 2)` is in one-to-one correspondence to the distance between the two initial opposite points. They are literally the same.

The support function of GJK maps a difference of two real-world points into Minkowski space. It seeks for two opposite points which are furthest apart along a given direction and returns their difference. In seek of opposite points along a direction the support function does the following:

![The GJK support function in seek of opposite points along a given direction](https://cloud.githubusercontent.com/assets/1294454/22177903/6e2dc3b2-e039-11e6-8492-7eb58f658560.jpg "The GJK support function in seek of opposite points along a given direction")

1. Start at the Origin.
2. Choose any direction you like (denoted as `D` on the image above). A direction is itself a vector, pointing somewhere. It can be random, you choose whatever you want for a start, later you'll see why initial direction doesn't really matter. The direction vector always starts at the Origin and is sometimes written as `OD = D(x,y) - O(0,0) = D(x,y)` (that is a direction from `O` towards `D`).
3. Take the first of two shapes. It does not matter which one of the two is first.
4. From the Origin seek for the furthest point of first shape in direction `D`.
5. To find the furthest point in direction `D` calculate the dot product of all the point-vectors of the first shape with direction `D`. This is the same as taking magnitudes (or distances) from the Origin to each point of the first shape in direction `D`. It is also often called *projecting a point-vector onto a direction-vector*. The distance to point `A` along direction `D` is the projection of vector `A` onto vector `D`. The most distant point `A` along `D` will have the greatest dot product of `A` and `D`. This way our first most distant point from the Origin along direction `D` is found. Note that a dot product can result in a negative projected vector.
6. Next an opposite point of the *other* shape must be found. To do that the support function switches the direction `D` to the opposite, literally flipping it `D = -D`. The process from step 5 is repeated, but this time with the other shape in a reverse direction `-D`. It looks for a point of the second shape that is most distant from the Origin along direction `-D`. To find the most distant point it calculates all dot products of all the point-vectors of the second shape and direction-vector `-D`. Then it takes the point which has the greatest dot product with `-D`. This way the second point `B` is found that is most distant from the Origin and is also opposite to the first point.
7. Once two opposite points `A` and `B` have been found, subtract one of them from the other and done. It doesn't matter which one you are subtracting from. The resulting vector is a mapping of their difference into 2D Minkowski space.

In other words, the support function takes an arbitrary line and two opposite points furthest from the Origin along that line, one point from each of the initial shapes. This is similar to subtracting segments in 1D. The support function finds the endpoints of two shapes along some direction-line and returns their difference that is another point in Minkowski space. On a one-dimensional number line the resulting point is a 1D-number. One a two-dimensional coordinate plane the resulting point is a 2D-vector (it has two coordinates).

Now its easy to show that it does not matter which initial direction you choose to start with. The support function does not care about given initial direction at all. Say, if on step 2 we choose a different arbitrary `D`, we might end up with something like this:

![The GJK support function in seek of opposite points along another direction](https://cloud.githubusercontent.com/assets/1294454/22178263/d12adcbc-e042-11e6-8a74-7c667c703f0d.jpg "The GJK support function in seek of opposite points along another direction")

It doesn't matter which direction `D` you choose to seek for opposite points `A` and `B`. By taking their difference you get a point `C` somewhere on a contour in Minkowski space. And it's easy to verify arithmetically that the intersection of opposite points `A` and `B` along any direction `D` still yields one of many points on the contour of our resulting shape:

```
A(2, -2) - B(-1, 2) = C(2 - (-1), -2 - 2) = C(3, -4)
```

So, in general the support function can work in any given direction and in any given space. You give it two shapes and a direction, and then it finds two opposite points in your space and returns their intersection in Minkowski space. In mathematics the intersection is usually denoted with symbol `∩`. 

```C
C = support (shape1, shape2, D); // return a point of (shape1 ∩ shape2) along arbitrary direction D
```

The intersection of two points always yields a third point. In 1D a point is a number. Thus, an intersection of two numbers or two points in 1D gives a third number or another 1D-point. In 2D an intersection of two vectors or two points gives a third vector or another 2D-point. 

##### A Word On Math

To calculate the difference of two numbers we subtract one of them from the other like this: `A - B = C`. In 1D which number of the two is A and which one is B does not matter, the algorithm works either way. This is because in 1D you have only one possible direction that is the actual number line itself. But in general when dealing with 2D or 3D coordinate vectors (which is usually the case in many applications) the order of subtraction actually does matter. A more accurate way of representing the difference of two vectors in Minkowski space is to take one vector and sum it with a negated version of the other vector, so that `A + (-B) = C`. Because of this fact the Minkowski support function is often defined as a sum of the first point-vector with the negated version of the second point-vector. After negating the second vector you simply add it to the first one to get their arithmetic total result. This is why the support function is called *Minkowski addition* or *Minkwoski sum* and you will probably never hear of *Minkowski subtraction* nor *Minkowski difference*.

Here's how a general implementation of the Minkowski sum support function for any space of arbitrary amount of dimensions might look like in C programming language:

```C
//-----------------------------------------------------------------------------
// Negate or 'flip' a point-vector in reverse direction

vec negate (vec p) {
    for (int i = 0; i < p.size; i++)
        p.components[i] = -p.components[i];
    return p;
}

//-----------------------------------------------------------------------------
// Return an arithmetic sum of two point-vectors
// A 1D-vector has only one component (one coordinate on a number line)
// In general a vector has one or more components

vec sum (vec a, vec b) {
    return a + b;
    for (int i = 0; i < a.size; i++)         
        a.components[i] += b.components[i];  // this is not very clean, but...
    return a;                                // at least it's understandable
}

//-----------------------------------------------------------------------------
// Dot product is the sum of all corresponding components of both vectors multiplied 

float dotProduct (vec a, vec b) {
    float product = 0;    
    for (int i = 0; i < a.size; i++)
        product += a.components[i] * b.components[i];
    return product;
}

//-----------------------------------------------------------------------------
// Get furthest vertex along a certain direction d
// This is the same as finding max dot product with d
// In 1D direction is always 1 (the number 1 on the number line)

size_t indexOfFurthestPoint (const vec * vertices, size_t count, vec d) {
    
    float maxProduct = dotProduct (d, vertices[0]);
    size_t index = 0;
    for (size_t i = 0; i < count; i++) {
        float product = dotProduct (d, vertices[i]); // may be negative
        if (product > maxProduct) {
            maxProduct = product;
            index = i;
        }
    }
    return index;
}

//-----------------------------------------------------------------------------
// Minkowski sum support function for GJK

vec support (const vec * vertices1, size_t count1, // first shape
             const vec * vertices2, size_t count2, // second shape
             vec d) {                              // direction

    // get furthest point of first body along an arbitrary direction
    size_t i = indexOfFurthestPoint (vertices1, count1, d);
    
    // get furthest point of second body along the opposite direction
    size_t j = indexOfFurthestPoint (vertices2, count2, negate (d));

    // return the Minkowski sum of two points to see if bodies 'overlap'
    // note that the second point-vector is negated, a + (-b) = c
    return sum (vertices1[i], negate (vertices2[j])); 
}
```

It does not really care how many dimensions are there in space. So given a direction and two shapes the support function always returns another point regardless of how many dimensions you have. It works the same for 1D, 2D, 3D, etc... The point returned by the support  function is always on the contour of intersection. This is because the initial points involved in the intersection are opposite.

Remember, the whole point of having a support function was to help us quickly build the simplex for GJK. The support function is used in the search for a 2-simplex that encloses the Origin in 2D. Now that we have such a support function, we can move on and build the rest of the algorithm on top of it.

#### The Evolution

...

WORK IN PROGRESS, A live demo of GJK in a 2D-space and a video of GJK in action coming up soon )

...

##### A Touch Of Degenerate Case

WORK IN PROGRESS, to be continued soon... )

### Roundness And Curvature

WORK IN PROGRESS, to be continued soon... )

### Adding 3D

A careful reader might have already noticed a pattern in the algorithm...

![Simplices in various spaces](https://cloud.githubusercontent.com/assets/1294454/22039058/c30ea898-dd0e-11e6-8e15-62a5b612036d.jpg "Simplices in various spaces")

For a 1D number line we need a 1-simplex of two points to enclose the Origin. If we can find such two points then our shapes do intersect indeed. If we cannot find such two points then our initial shapes must have some distance (non-zero difference) between them. For 2D coordinate plane a 2-simplex of three points (a triangle) is necessary to enclose the Origin. In 3D space we need a 3-simplex of four points (a tetrahedron). These are all examples of simplest possible shape primitives in given dimensions.

WORK IN PROGRESS, A live demo of GJK in a 3D-space and a video of GJK in action coming up soon )

### Distance or Depth Of Collision

WORK IN PROGRESS, to be continued soon... )

### Contact Points

WORK IN PROGRESS, to be continued soon... )

## References (must see)
Most of the info (along with reference implementation) was taken from dyn4j. It has a very thorough explanation of GJK and it is definitely a must visit for those who need to understand the intricacies of the algorithm.

1. http://www.dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/ GJK description (+ a lot of other useful articles)
2. http://mollyrocket.com/849 Awesome old-school GJK / Minkowski space video by Casey Muratori
3. https://github.com/wnbittle/dyn4j Quality source code for reference

## P.S. 3D-version coming soon...
![3D-version of GJK in plain C coming soon...](http://s21.postimg.org/da9txc3uv/Screen_Shot_2016_01_13_at_09_13_12.jpg "3D-version of GJK in plain C coming soon...")
