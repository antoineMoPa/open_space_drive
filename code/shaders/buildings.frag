#version 130

uniform float time;

// Input from vertex shader
in vec2 texcoord;
in vec4 screen_position;
in vec4 position;
in vec3 normal;

void main() {
  vec3 col = vec3(0.0);

  col = vec3(0.3, 0.0, 0.3);

  float windows = 1.0;
  windows *= mix(clamp(cos(position.x * 4.0)/0.1, 0.0,1.0), 1.0, abs(normal.x));
  windows *= mix(clamp(cos(position.y * 4.0)/0.1, 0.0,1.0), 1.0, abs(normal.y));
  windows *= clamp(cos(position.z * 4.0)/0.1, 0.0,1.0);
  windows *= 1.0-abs(normal.z);
  windows -= screen_position.z/1000.0;

  vec3 window_color = vec3(0.3, 0.3, 0.0);
  window_color.r += 0.3 * cos(position.x / 100.0);
  window_color.g += 0.3 * cos(position.y / 100.0);
  window_color.b += 0.3 * cos(position.z / 100.0);
  col += windows * window_color;

  float floor = 0.0;
  floor += normal.z;

  col -= 0.3 * floor;

  col.b += 0.1;

  gl_FragColor = vec4(col,1.0);
}
