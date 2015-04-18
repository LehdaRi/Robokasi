#version 330 core

varying vec4 pos;
varying vec4 norm;
varying vec3 col;

out vec3 color;

void main() {
    color = col * (0.35+clamp(dot(normalize(vec3(-1.0f, -1.0f, -1.0f)), normalize(norm.xyz)), 0.0, 1.0)*0.65);
}
