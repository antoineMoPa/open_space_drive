#version 130

uniform float time;
uniform float acceleration;

// Input from vertex shader
in vec2 texcoord;
in vec4 screen_position;
in vec4 position;

void main() {
  vec4 color = vec4(0.0);

  float fire = acceleration * 0.3;
  fire += acceleration * cos(position.x * 4.0 + time * 100.0);
  fire += acceleration * cos(position.y * 3.0 + time * 100.0);
  fire += acceleration * cos(position.z * 5.0 + time * 100.0);
  fire *= 2.0;
  const vec4 fire_color = vec4(0.4,0.0, 0.8,0.8);
  color += fire * fire_color;

  gl_FragColor = color.rgba;
}
