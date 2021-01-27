#version 130

uniform samplerCube p3d_Texture0;
uniform vec3 cameraPosition;

// Input from vertex shader
in vec2 texcoord;
in vec4 position;
in vec4 screen_position;
in vec3 normal;

uniform struct {
  vec4 ambient;
  vec4 diffuse;
  vec3 specular;
  float shininess;
} p3d_Material;

void main() {
  vec4 col = vec4(0.0);

  col += p3d_Material.diffuse * 0.3;
  // I don't think the math is right, but the effect is good.
  // feel free to fix.
  vec3 view = normalize(cameraPosition - position.xyz);
  vec3 reflection = reflect(view, normal.xyz);
  float reflectionIntensity = 0.3 + 0.3 * dot(reflection, view);
  col += reflectionIntensity * textureCube(p3d_Texture0, reflection);
  col.a = 1.0;

  gl_FragColor = col;
}
