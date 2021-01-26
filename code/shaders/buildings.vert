#version 130

// Uniform inputs
uniform mat4 p3d_ModelViewProjectionMatrix;

// Vertex inputs
in vec4 p3d_Vertex;
in vec2 p3d_MultiTexCoord0;
in vec3 p3d_Normal;

// Output to fragment shader
out vec2 texcoord;
out vec4 screen_position;
out vec4 position;
out vec3 normal;


void main() {
  // Floor position to make sure windows don't go over edges
  float scale = 8.0;
  position = p3d_Vertex;
  position.xyz = floor(p3d_Vertex.xyz/scale) * scale + scale/2;
  position.w = 1.0;
  normal = p3d_Normal;
  gl_Position = screen_position = p3d_ModelViewProjectionMatrix * position;
  texcoord = p3d_MultiTexCoord0;
}
