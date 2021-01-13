#version 130

uniform float time;

// Input from vertex shader
in vec2 texcoord;
in vec4 screen_position;
in vec4 position;

void main() {
  vec4 color = vec4(0.0);

  float edges = 2.0 * texcoord.x - 1.0;
  float size = 0.03;
  edges = abs(edges);
  edges -= 1.0 - size;
  edges /= size;
  edges = 1.0 - abs(edges - 0.0);
  edges = clamp(edges, 0.0, 1.0);
  float neon = 1. * pow(edges, 5.0);
  float halo = 0.8 * pow(edges, 3.0) - 1.0*neon;
  color += neon * vec4(2.0,0.0,0.0,1.0);
  color += halo * vec4(0.9,0.5,0.7,1.0);

  color = clamp(color, 0.0 ,1.0);
  color.a *= 0.9;

  float distance = clamp(screen_position.z/2000.0, 0.0, 1.0);
  color *= 1.0 - distance;

  color = clamp(color, 0.0, 1.0);

  color += 0.1 * clamp(distance/2.0 * vec4(1.0, 0.9, 0.8, 0.3), 0.0, 1.0);

  gl_FragColor = color.rgba;
}
