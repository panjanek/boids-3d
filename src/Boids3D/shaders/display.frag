#version 430

layout(location = 0) in vec3 vColor;
layout(location = 1) in float vFadingAlpha;
layout(location = 2) in float vLighting;

out vec4 outputColor;

void main()
{
    vec3 skyColor = vec3(0.15f, 0.25f, 0.45f);

    float light = 0.5 + 0.5 * vLighting;
    vec3 objectColor = vColor * light;
    float visibility = vFadingAlpha;
    //vec4 col = vec4(objectColor * vFadingAlpha, 1.0);
    vec3 finalColor = mix(skyColor, objectColor, visibility);
    outputColor = vec4(finalColor, 1.0);
}