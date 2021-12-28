# Ray Marching Infinite Tunnel

***An Infinite Tunnel implemented with Ray Marching & SDFs in GLSL shader***

![](https://raw.githubusercontent.com/jason-ono/Story/master/inftunnel_assets/angle.png)

The project is live on [Shadertoy](https://www.shadertoy.com/view/NlVSWW)!

Common implementations of infinite tunnels on Shadertoy use clever techniques that achieve efficient runtime performance (such as [this one](https://iquilezles.org/www/articles/tunnel/tunnel.htm) by Inigo Quilez). This project, however, is an attempt to design an infinite tunnel using traditional ray marching techniques and signed distance functions (even if that results in poorer runtime performance...).

## Sources

- This project is based on Jamie Wong/[@jlfwong](https://github.com/jlfwong)'s [Ray Marching: Part 6](https://www.shadertoy.com/view/4tcGDr) project, as well as other code snippets introduced in his article ["Ray Marching and Signed Distance Functions"](http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/). The large portion of the code below comes from his project, especially with regards to the implementation of the Phong Reflection Model and Constructive Solid Geometry.
- All the signed distance functions used in this project come from [Inigo Quilez's articles on 3D SDFs](https://iquilezles.org/www/articles/distfunctions/distfunctions.htm).

## Basic Structure

- There are three main components in this project: the outer cylinder, moving objectz placed on the inner surface of the cylinder to make it look like the viewer is moving inside a tunnel, and an ellipsoid placed inside the tunnel (in the center) to display the reflection of the tunnel's inner surface.

- The camera is placed inside the cylinder, which is technically defined as an elongated torus (see the definition of `opElongateOuter`). Inside the cylinder, a group of 25 tori moves infinitely in one direction using [Sawtooth wave function](https://en.wikipedia.org/wiki/Sawtooth_wave).

  ![](https://raw.githubusercontent.com/jason-ono/Story/master/inftunnel_assets/mechanism.jpeg)

- This project uses Jamie Wong's implementation of the Phong Lighting Model, with five light sources: left, right, top, bottom, and in front, all with respect to the camera angle. I wanted the inner surface of the tunnel to look like it is glowing so that you can see the glow reflect on the ellipsoid. Thus, the ellipsoid does not have its own ambient color and instead fully reflects the color of the inner surface of the tunnel.

![](https://raw.githubusercontent.com/jason-ono/Story/master/inftunnel_assets/ellipsoid.png)

- All three items are placed using constructive solid geometry. 

## Customization

- The object inside the tunnel can be replaced easily. Simply replace the `sdEllipsoid` call inside `sceneSDF` function to one of `sdSphere`, `sdBox`, `sdCone` .

| `sdSphere`                                                   | `sdBox`                                                      | `sdCone`                                                     |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![](https://raw.githubusercontent.com/jason-ono/Story/master/inftunnel_assets/sphere.png) | ![](https://raw.githubusercontent.com/jason-ono/Story/master/inftunnel_assets/box.png) | ![](https://raw.githubusercontent.com/jason-ono/Story/master/inftunnel_assets/cone.png) |

- The color of the objects can be also changed in `mainImage` function.

![](https://raw.githubusercontent.com/jason-ono/Story/master/inftunnel_assets/green.png)

- You can also change the camera location by editing the coordinate of `eye` variable in `mainImage` function.
