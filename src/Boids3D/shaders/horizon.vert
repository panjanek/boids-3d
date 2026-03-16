#version 430

layout(location = 0) in vec3 position;

uniform mat4 view;
uniform mat4 projection;
uniform float fieldSize;

out vec3 worldPos;

void main()
{
    worldPos = vec3(position.x, 0.0, position.z) * fieldSize * 10.0;
    gl_Position = projection * view * vec4(worldPos,1.0);
}