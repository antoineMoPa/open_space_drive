#version 130

// Input from vertex shader
in vec2 texcoord;
in vec4 screen_position;

void main() {
  vec4 color = vec4(0.0);

  float edges = 2.0 * texcoord.x - 1.0;
  float size = 0.03;
  edges = abs(edges);
  edges -= 1.0 - size;
  edges /= size;
  edges = 1.0 - abs(edges - 0.0);
  edges = clamp(edges, 0.0, 1.0);
  //edges = max(edges, 0.0);
  color += 1.2 * pow(edges, 2.0) * edges * vec4(2.0,1.0,1.0,1.0);
  color += 1.0 * pow(edges, 2.0) * vec4(1.0,0.0,0.0,1.0);

  color = clamp(color, 0.0 ,1.0);
  color.a *= 0.9;

  color *= 1.0 - screen_position.z/300.0;

  gl_FragColor = color.rgba;
}
