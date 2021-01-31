#version 130

// Uniform inputs
uniform mat4 p3d_ModelViewProjectionMatrix;
uniform mat3 p3d_NormalMatrix;
uniform mat4 p3d_ModelViewMatrix;

// Vertex inputs
in vec4 p3d_Vertex;
in vec2 p3d_MultiTexCoord0;
in vec3 p3d_Normal;

// Output to fragment shader
out vec2 texcoord;
out vec4 screen_position;
out vec4 position;
out vec3 normal;
out vec4 worldPosition;

void main() {
  normal = p3d_Normal;
  position = p3d_Vertex;
  worldPosition = p3d_ModelViewMatrix * p3d_Vertex;
  gl_Position = screen_position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
  texcoord = p3d_MultiTexCoord0;
}
