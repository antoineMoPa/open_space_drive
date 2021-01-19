#version 130

uniform float time;

// Input from vertex shader
in vec2 texcoord;
in vec4 screen_position;
in vec4 position;

void main() {
  vec4 color = vec4(0.0);

  vec4 base_color = vec4(0.15,0.0,0.1,0.2);

  color += base_color;

  float distance = abs(screen_position.z);

  color *= 0.3 + 1.0 / pow(distance/300.0, 3.0);

  const float fade_close_distance = 30.0;
  float too_close = 1.0-clamp(fade_close_distance/distance, 0.0, 1.0);
  color *= too_close;
  color = clamp(color, 0.0, 1.0);
  color = min(color, vec4(0.3));

  color *= 0.4;
  color += 0.5 * base_color;

  color = clamp(color, 0.0, 1.0);

  gl_FragColor = color.rgba;
}
