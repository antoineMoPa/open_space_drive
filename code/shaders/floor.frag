#version 130

uniform float time;

// Input from vertex shader
in vec2 texcoord;
in vec4 screen_position;
in vec4 position;
in vec3 normal;

void main() {
  vec3 col = vec3(0.0);

  col = vec3(0.4, 0.0, 0.4);

  float windows = 1.0;
  windows *= clamp(cos(position.z * 4.0)/0.1, 0.0,1.0);
  windows *= clamp(cos(position.y * 4.0)/0.1, 0.0,1.0);
  windows *= 1.0 - normal.z;
  col -= windows * 0.3;

  float floor = 0.0;
  floor += normal.z;

  col -= 0.3 * floor;

  col.b += 0.1;

  gl_FragColor = vec4(col,1.0);
}
