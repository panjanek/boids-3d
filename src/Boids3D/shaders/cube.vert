#version 430

layout(location = 0) in vec3 position;

uniform mat4 view;
uniform mat4 projection;
uniform float fieldSize;

void main()
{
    vec3 worldPos = position * fieldSize;
    gl_Position = projection * view * vec4(worldPos, 1.0);
}