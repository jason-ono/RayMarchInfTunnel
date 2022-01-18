/*
    Ray Marching Infinite Tunnel
    by Jason Ono

    - This project is based on Jamie Wong's Shadertoy project "Ray Marching: Part 6"
        (https://www.shadertoy.com/view/4tcGDr) introduced in his article about 
        foundations of ray marching and signed distance functions. The large portion of
        the code below comes from his project, especially with regards to the
        implementation of the Phong Reflection Model and Constructive Solid Geometry.
        - http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
    - All signed distance functions are from Inigo Quilez's website
        - https://iquilezles.org/www/articles/distfunctions/distfunctions.htm

 */

/* —————————————————————————————————————————————————————————————————————————————————————— */
/*
    SDFs by Inigo Quilez
 */

const int MAX_MARCHING_STEPS = 255;
const float MIN_DIST = 0.0;
const float MAX_DIST = 100.0;
const float EPSILON = 0.0001;

float sdSphere(vec3 samplePoint) {
    return length(samplePoint) - 0.250;
}

float sdEllipsoid(vec3 p) {
    vec3 r = vec3(0.3, 0.2, 0.4);
    float k0 = length(p / r);
    float k1 = length(p / (r * r));
    return k0 * (k0 - 1.0) / k1;
}

float sdBox(vec3 p) {
    vec3 b = vec3(0.1, 0.2, 0.3);
    vec3 q = abs(p) - b;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}

float sdTorus(vec3 p) {
    vec2 t = vec2(1.5, 0.05);
    vec2 q = vec2(length(p.xz) - t.x, p.y);
    return length(q) - t.y;
}

float sdTorusWave(vec3 p) {
    vec2 t = vec2(1.45, 0.1);
    vec2 q = vec2(length(p.xz) - t.x, p.y);
    return length(q) - t.y;
}

float opElongateOuter(in vec3 p) {
    vec3 h = vec3(0.0, 30.0, 0.0);
    vec3 q = p - clamp(p, -h, h);
    return sdTorus(q);
}

float opElongateWave(in vec3 p) {
    vec3 h = vec3(0.0, 0.005, 0.0);
    vec3 q = p - clamp(p, -h, h);
    return sdTorusWave(q);
}

float sdCone(in vec3 p) {
  // c is the sin/cos of the angle, h is height
  // Alternatively pass q instead of (c,h),
  // which is the point at the base in 2D
    vec2 c = vec2(0.5, 0.866);
    float h = 0.5;

    vec2 q = h * vec2(c.x / c.y, -1.0);

    vec2 w = vec2(length(p.xz), p.y);
    vec2 a = w - q * clamp(dot(w, q) / dot(q, q), 0.0, 1.0);
    vec2 b = w - q * vec2(clamp(w.x / q.x, 0.0, 1.0), 1.0);
    float k = sign(q.y);
    float d = min(dot(a, a), dot(b, b));
    float s = max(k * (w.x * q.y - w.y * q.x), k * (w.y - q.y));
    return sqrt(d) * sign(s);
}

mat4 rotateX(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat4(vec4(1, 0, 0, 0), vec4(0, c, -s, 0), vec4(0, s, c, 0), vec4(0, 0, 0, 1));
}

mat4 rotateY(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat4(vec4(c, 0, s, 0), vec4(0, 1, 0, 0), vec4(-s, 0, c, 0), vec4(0, 0, 0, 1));
}

vec2 opU(vec2 d1, vec2 d2) {
    return (d1.x < d2.x) ? d1 : d2;
}

/* —————————————————————————————————————————————————————————————————————————————————————— */

/**
 * Signed distance function describing the scene.
 * .x
 * - Absolute value of the return value indicates the distance to the surface.
 * - Sign indicates whether the point is inside or outside the surface,
 * - negative indicating inside.
 * .y
 * - value reserved for coloring
 */
vec2 sceneSDF(vec3 samplePoint) {

    // to rotate the object in center
    vec3 cubePoint = ((rotateY(iTime * 0.3)) * (rotateX(iTime * 0.3)) * vec4(samplePoint, 1.0)).xyz;

    // to vertically rotate the torus
    vec3 adjustedPoint = ((rotateY(0.0)) * (rotateX(1.5708)) * vec4(samplePoint, 1.0)).xyz; // ~ pi/2

    // startin point for constructive solid geometry
    vec2 res = vec2(1e10, 0.0);

    // center object with repeated rotation applied
        /*
        change the SDF here to see alternate shapes
        available options:
        - sdEllipsoid
        - sdSphere
        - sdCone
        - sdBox
        */
    res = opU(res, vec2(sdEllipsoid(cubePoint), 1.0));

    // outer wall, static
    res = opU(res, vec2(opElongateOuter(adjustedPoint), 0.8));

    // waves of tori
    float counter = 0.0;
    for(int i = 0; i < 25; i++) {
        counter = counter + 2.0;
        /*
        the group of tori move from one end of the cylinder to the other infinitely using
        Sawtooth wave-like function
        https://en.wikipedia.org/wiki/Sawtooth_wave
         */
        res = opU(res, vec2(opElongateWave(adjustedPoint -
                                            vec3(0.0,
                                            10.0 * ((0.08 * iTime) - floor(0.08 * iTime)) + counter - 50.0,
                                            0.0)), 0.1));
    }

    return res;
}

/**
 * By Jamie Wong
 * https://www.shadertoy.com/view/4tcGDr
 *
 * Return the shortest distance from the eyepoint to the scene surface along
 * the marching direction. If no part of the surface is found between start and end,
 * return end.
 *    
 * eye: the eye point, acting as the origin of the ray
 * marchingDirection: the normalized direction to march in
 * start: the starting distance away from the eye
 * end: the max distance away from the ey to march before giving up
 */
vec2 shortestDistanceToSurface(vec3 eye, vec3 marchingDirection, float start, float end) {
    float depth = start;
    float color;
    for(int i = 0; i < MAX_MARCHING_STEPS; i++) {
        vec2 mapped = sceneSDF(eye + depth * marchingDirection);
        float dist = mapped.x;
        color = mapped.y;
        if(dist < EPSILON) {
            return vec2(depth, color);
        }
        depth += dist;
        if(depth >= end) {
            return vec2(end, color);
        }
    }
    return vec2(end, color);
}

/**
 * By Jamie Wong
 * https://www.shadertoy.com/view/4tcGDr
 *
 * Return the normalized direction to march in from the eye point for a single pixel.
 * 
 * fieldOfView: vertical field of view in degrees
 * size: resolution of the output image
 * fragCoord: the x,y coordinate of the pixel in the output image
 */
vec3 rayDirection(float fieldOfView, vec2 size, vec2 fragCoord) {
    vec2 xy = fragCoord - size / 2.0;
    float z = size.y / tan(radians(fieldOfView) / 2.0);
    return normalize(vec3(xy, -z));
}

/**
 * Using the gradient of the SDF, estimate the normal on the surface at point p.
 */
vec3 estimateNormal(vec3 p) {
    return normalize(vec3(sceneSDF(vec3(p.x + EPSILON, p.y, p.z)).x - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)).x, sceneSDF(vec3(p.x, p.y + EPSILON, p.z)).x - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)).x, sceneSDF(vec3(p.x, p.y, p.z + EPSILON)).x - sceneSDF(vec3(p.x, p.y, p.z - EPSILON)).x));
}

/**
 * By Jamie Wong
 * https://www.shadertoy.com/view/4tcGDr
 * Lighting contribution of a single point light source via Phong illumination.
 * 
 * The vec3 returned is the RGB color of the light's contribution.
 *
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 * lightPos: the position of the light
 * lightIntensity: color/intensity of the light
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
vec3 phongContribForLight(vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye, vec3 lightPos, vec3 lightIntensity) {
    vec3 N = estimateNormal(p);
    vec3 L = normalize(lightPos - p);
    vec3 V = normalize(eye - p);
    vec3 R = normalize(reflect(-L, N));

    float dotLN = dot(L, N);
    float dotRV = dot(R, V);

    if(dotLN < 0.0) {
        // Light not visible from this point on the surface
        return vec3(0.0, 0.0, 0.0);
    }

    if(dotRV < 0.0) {
        // Light reflection in opposite direction as viewer, apply only diffuse
        // component
        return lightIntensity * (k_d * dotLN);
    }
    return lightIntensity * (k_d * dotLN + k_s * pow(dotRV, alpha));
}

/**
 * Original Function by Jamie Wong, edited by Jason Ono
 * https://www.shadertoy.com/view/4tcGDr
 * 
 * Lighting via Phong illumination.
 * 
 * The vec3 returned is the RGB color of that point after lighting is applied.
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
vec3 phongIllumination(vec3 k_a, vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye) {
    const vec3 ambientLight = 0.5 * vec3(1.0, 1.0, 1.0);
    vec3 color = ambientLight * k_a;

    // top
    vec3 light1Pos = vec3(0.0, 5.0, 0.0);
    vec3 light1Intensity = vec3(0.4, 0.4, 0.4);
    color += phongContribForLight(k_d, k_s, alpha, p, eye, light1Pos, light1Intensity);

    // right
    vec3 light2Pos = vec3(5.0, 0.0, 0.0);
    vec3 light2Intensity = vec3(0.4, 0.4, 0.4);
    color += phongContribForLight(k_d, k_s, alpha, p, eye, light2Pos, light2Intensity);

    // left
    vec3 light3Pos = vec3(-5.0, 0.0, 0.0);
    vec3 light3Intensity = vec3(0.4, 0.4, 0.4);
    color += phongContribForLight(k_d, k_s, alpha, p, eye, light3Pos, light3Intensity);

    // bottom
    vec3 light4Pos = vec3(0.0, -5.0, 0.0);
    vec3 light4Intensity = vec3(0.4, 0.4, 0.4);
    color += phongContribForLight(k_d, k_s, alpha, p, eye, light4Pos, light4Intensity);

    // front
    vec3 light5Pos = vec3(0.0, 0.0, 5.0);
    vec3 light5Intensity = vec3(0.4, 0.4, 0.4);
    color += phongContribForLight(k_d, k_s, alpha, p, eye, light5Pos, light5Intensity);

    return color;
}

void mainImage(out vec4 fragColor, in vec2 fragCoord) {

    vec3 dir = rayDirection(45.0, iResolution.xy, fragCoord);
    vec3 eye = vec3(0.0, 0.0, 5.0); // center camera position
    // vec3 eye = vec3(0.9*cos(iTime*0.5), 0.9*sin(iTime*0.1), 5.0); // dynamic camera position
    // vec3 eye = vec3(0.5,0.5, 5.0); // alternate camera position
    vec2 distVec = shortestDistanceToSurface(eye, dir, MIN_DIST, MAX_DIST);
    float dist = distVec.x;
    float col = distVec.y;

    if(dist > MAX_DIST - EPSILON) {
        // Didn't hit anything
        fragColor = vec4(0.0, 0.0, 0.0, 0.0);
        return;
    }

    vec3 K_d;
    vec3 color;

    // for reflection of the object in center
    vec3 refDir;
    vec3 reflEye;
    vec2 reflDistVec;
    float reflDist;
    float reflCol;

    // center ball
    if(col == 1.0) {

        reflEye = eye - dist; // new camera position (on the object's surface)
        // shadow direction
        refDir = -estimateNormal(reflEye);
        // new ray march instance
        reflDistVec = shortestDistanceToSurface(reflEye, refDir, MIN_DIST, MAX_DIST);
        reflDist = reflDistVec.x;
        reflCol = reflDistVec.y;
        vec3 ref_d;

        // wall
        if(reflCol == 0.8) {
            ref_d = vec3(0.1, 0.1, 0.1); // gray
            // ref_d = vec3(0.0, 1.0, 0.0); // green
        }
        // moving waves
        if(reflCol == 0.1) {
            // ref_d = vec3(0.0, 0.0, 1.0); // blue
            ref_d = vec3(0.0, 1.0, 1.0); // light blue
        }

        vec3 p = eye + dist * dir;
        vec3 K_a = vec3(0.2, 0.2, 0.2);
        vec3 K_s = vec3(1.0, 1.0, 1.0);
        float shininess = 100.0;

        vec3 reflp = reflEye + reflDist * -estimateNormal(reflEye);

        vec3 refColor = phongIllumination(K_a, ref_d, K_s, shininess, p, eye);

        // the original object doesn't have an original color assigned, emphasize the reflection
        color = refColor;
    } else {
        // outer wall
        if(col == 0.8) {
            K_d = vec3(0.1, 0.1, 0.1); // gray
            // K_d = vec3(0.0, 1.0, 0.0); // green
        }
        // moving waves
        if(col == 0.1) {
            // K_d = vec3(0.0, 0.0, 1.0); // blue
            K_d = vec3(0.0, 0.6, 0.6); // light blue
        }

        // The closest point on the surface to the eyepoint along the view ray
        vec3 p = eye + dist * dir;

        vec3 K_a = vec3(0.2, 0.2, 0.2);
        vec3 K_s = vec3(1.0, 1.0, 1.0);

        // for the glossier look
        float shininess = 50.0;

        color = phongIllumination(K_a, K_d, K_s, shininess, p, eye);
    }
    fragColor = vec4(color, 1.0);
}