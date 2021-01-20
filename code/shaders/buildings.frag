#version 130

uniform float time;
uniform vec3 camera_position;

// Input from vertex shader
in vec2 texcoord;
in vec4 screen_position;
in vec4 position;
in vec3 normal;

void main() {
  vec3 col = vec3(0.0);

  col = vec3(0.1, 0.0, 0.1);

  float windows = 1.0;
  const float TAU = 6.2832;
  const float offset = 3.1416;
  windows *= mix(clamp(cos(position.x * TAU + offset)/0.1, 0.0,1.0), 1.0, abs(normal.x));
  windows *= mix(clamp(cos(position.y * TAU + offset)/0.1, 0.0,1.0), 1.0, abs(normal.y));
  windows *= clamp(cos(position.z * TAU + offset)/0.1, 0.0,1.0);
  windows *= 1.0-abs(normal.z);

  // Give a feeling of depth to the windows
  vec3 window_position = mod(position.xyz, vec3(1.0)) - 0.5;
  float depth = 0.3 * cos(position.x/10.0);
  vec3 pos_behind_window = window_position + normalize(position.xyz - camera_position) * depth;
  vec3 window_color = vec3(0.3, 0.3, 0.0);

  window_color -= normal.x * windows * 0.4 * clamp(cos(pos_behind_window.y * TAU * 1.1)/0.1,0.0,1.0);
  window_color -= normal.y * windows * 0.4 * clamp(cos(pos_behind_window.x * TAU * 1.1)/0.1,0.0,1.0);
  window_color -= windows * 0.2 * clamp(cos(pos_behind_window.z * TAU * 2.0)/0.1,0.0,1.0);

  windows -= screen_position.z/1000.0;

  window_color.r += 0.3 * abs(cos(position.x / 100.0));
  window_color.g += 0.2 * abs(cos(position.y / 100.0));
  window_color.b += 0.1 * abs(cos(position.z / 100.0));
  col += windows * window_color;

  float floor = 0.0;
  floor += normal.z;

  col -= 0.3 * floor;

  col.b += 0.02;

  gl_FragColor = vec4(col,1.0);
}
