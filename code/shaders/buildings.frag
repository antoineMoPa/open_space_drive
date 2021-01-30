#version 130

uniform float time;
uniform vec3 camera_position;

// Input from vertex shader
in vec2 texcoord;
in vec4 screen_position;
in vec4 position;
in vec3 normal;

/*
  Returns the intersection as a multiple of planev1 and planev2

  P0 a point on the plane
  P01 another point on the plane
  P02 another point on the plane

  The last point delimit a rectangle:

       .P01_________
       |            |
       |        lb  |
       |       /    |
    P0 .______/_____.P02
             /
            /la

  la and lb delimit a line.

  Returns a vector xyzw where x is and indicator. x < 0 if there is no intersection.
*/
vec4 planeLineIntersection(vec3 P0, vec3 P01, vec3 P02, vec3 la, vec3 lb) {
  // Many thanks to wikipedia
  // Reference:
  // https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
  vec3 lab = lb - la;
  float det = - dot(lab, cross(P01,P02));

  if (abs(det) < 0.01) {
    return vec4(-1.0,0.0,0.0,0.0);
  }

  float t = 1.0/det * dot(cross(P01, P02),  la - P0);
  float u = 1.0/det * dot(cross(P02, -lab), la - P0);
  float v = 1.0/det * dot(cross(-lab, P01), la - P0);

  vec3 x = P0 + P01 * u + P02 * v;

  float indicator = 1.0;

  if (t < 0.0 || t > 1.5) {
    indicator = -2.0;
  }

  if (u < -0.01 || v < -0.01 || u > 1.01 || v > 1.01) {
    indicator = -3.0;
  }

  vec4 ret;
  ret.xyzw = vec4(indicator, x);
  return ret;
}

vec4 wallColorDistanceBlend(vec3 p, vec3 c, vec4 col){
  float distfactor = clamp(length(p-c)/2000.0,0.0,1.0);
  col.rgb = distfactor * vec3(1.0,0.6,0.5) + (1.0 - distfactor) * col.rgb;
  return col;
}

vec4 drawFloor(vec3 wc, vec3 wr, vec3 up, vec3 n, vec3 c, vec3 p, float s) {
  vec4 col = vec4(0.0);

  vec3 P0 = wc - wr * 0.25 * s - up * 0.25 * s - n * s * 0.5;
  vec3 P02 = -n * s * 0.75;
  vec3 P01 = wr * 0.5 * s;

  vec4 intersection = planeLineIntersection(P0, P02, P01, c, p).xyzw;
  float indicator = intersection.x;
  vec3 point = intersection.yzw;

  if (indicator > 0.0) {
    col.a = 1.0;
    col.g = 0.04;
    col.b = 0.09;
    col = wallColorDistanceBlend(p, c, col);
  }

  return col;
}

vec4 drawRightWall(vec3 wc, vec3 wr, vec3 up, vec3 n, vec3 c, vec3 p, float s) {
  vec4 col = vec4(0.0);

  vec3 P0 = wc - up * 0.25 * s + wr * 0.25 * s - n * s * 0.5;
  vec3 P02 = -n * s * 0.75;
  vec3 P01 = up * 0.5 * s;

  vec4 intersection = planeLineIntersection(P0, P02, P01, c, p).xyzw;
  float indicator = intersection.x;
  vec3 point = intersection.yzw;

  if (indicator > 0.0) {
    col.a = 1.0;
    col.g = 0.1;
    col.b = 0.2;
    col = wallColorDistanceBlend(p, c, col);
  }

  return col;
}

vec4 drawLeftWall(vec3 wc, vec3 wr, vec3 up, vec3 n, vec3 c, vec3 p, float s) {
  vec4 col = vec4(0.0);

  vec3 P0 = wc - up * 0.25 * s - wr * 0.25 * s - n * s * 0.5;
  vec3 P02 = -n * s * 0.75;
  vec3 P01 = up * 0.5 * s;

  vec4 intersection = planeLineIntersection(P0, P02, P01, c, p).xyzw;
  float indicator = intersection.x;
  vec3 point = intersection.yzw;

  if (indicator > 0.0) {
    col.a = 1.0;
    col.g = 0.1;
    col.b = 0.2;
    col = wallColorDistanceBlend(p, c, col);
  }

  return col;
}

vec4 drawCeiling(vec3 wc, vec3 wr, vec3 up, vec3 n, vec3 c, vec3 p, float s) {
  vec4 col = vec4(0.0);

  vec3 P0 = wc - wr * 0.25 * s + up * 0.25 * s - n * s * 0.5;
  vec3 P02 = -n * s * 0.75;
  vec3 P01 = wr * 0.5 * s;

  vec4 intersection = planeLineIntersection(P0, P02, P01, c, p).xyzw;
  float indicator = intersection.x;
  vec3 point = intersection.yzw;

  if (indicator > 0.0) {
    col.a = 1.0;
    col.rgb += vec3(0.3,0.1,0.1);
    col = wallColorDistanceBlend(p, c, col);
  }

  return col;
}

vec4 drawBackWall(vec3 wc, vec3 wr, vec3 up, vec3 n, vec3 c, vec3 p, float s) {
  vec4 col = vec4(0.0);

  vec3 P0 = wc - wr * 0.25 * s - up * 0.25 * s - n * s * 1.25;
  vec3 P02 = up * 0.5 * s;
  vec3 P01 = wr * 0.5 * s;

  vec4 intersection = planeLineIntersection(P0, P02, P01, c, p).xyzw;
  float indicator = intersection.x;
  vec3 point = intersection.yzw;

  if (indicator > 0.0) {
    col.a = 1.0;
    col.rb += 0.3;
    col = wallColorDistanceBlend(p, c, col);
  }

  return col;
}

void main() {
  vec4 col = vec4(0.0);

                                                            // Define some shortcuts
  const float s  = 8.0;                                     // Scale
  vec3 p   = position.xyz;                                  // Position in 3D world
  vec3 n   = normal;                                        // Normal vector
  vec3 up  = vec3(0.0,0.0,1.0);                             // Up vector
  vec3 c   = camera_position;
  vec3 ws  = (vec3(1.0)-abs(n));                            // Window surface
  vec3 wc  = floor(p / s - 0.5 + 0.01 * n) * s + s;         // Window center position in 3D world
  vec3 wr  = ws * (vec3(1.0) - up);                         // Window right vector

  float dist = length(p-c);                                 // Distance between wall and camera

  if (dist < 1000.0) {
    col += drawFloor(wc, wr, up, n, c, p, s);
    col += drawRightWall(wc, wr, up, n, c, p, s);
    col += drawLeftWall(wc, wr, up, n, c, p, s);
    col += drawCeiling(wc, wr, up, n, c, p, s);
    col += drawBackWall(wc, wr, up, n, c, p, s);
  } else {
    col.rgb += dist/2000.0 * vec3(1.0,0.6,0.5);
  }

  float windows = 1.0;

  const float TAU = 6.2832;
  const float f = TAU / s;
  const float offset = 0;
  windows *= mix(clamp(cos(position.x * f + offset)/0.1, 0.0,1.0), 1.0, abs(n.x));
  windows *= mix(clamp(cos(position.y * f + offset)/0.1, 0.0,1.0), 1.0, abs(n.y));
  windows *= clamp(cos(position.z * f + offset)/0.1, 0.0,1.0);
  windows *= 1.0-abs(n.z);

  col *= windows;


  gl_FragColor = col;
}
