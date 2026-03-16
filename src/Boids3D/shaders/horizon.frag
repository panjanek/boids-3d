#version 430

in vec3 worldPos;

uniform vec3 cameraPos;
uniform float fieldSize;

out vec4 outputColor;

void main()
{
    vec3 groundColor = vec3(0.18,0.25,0.18);
    vec3 skyColor = vec3(0.15f, 0.25f, 0.45f);

    vec2 diff = worldPos.xz - cameraPos.xz;
    float dist = length(diff);

    float horizonFade = smoothstep(fieldSize * 1.2,
                                   fieldSize * 5.0,
                                   dist);
    
    // ----- antialiased grid -----
    float gridScale = 30.0;   // world units per cell
    float lineWidth = 0.04;  // line thickness in grid units
    
    vec2 coord = worldPos.xz / gridScale;
    vec2 grid = fract(coord);
    
    // distance to nearest line along each axis
    float dx = min(grid.x, 1.0 - grid.x);
    float dz = min(grid.y, 1.0 - grid.y);
    
    // pixel footprint for AA
    float ax = fwidth(coord.x);
    float az = fwidth(coord.y);
    
    // anti-aliased lines
    float lineX = 1.0 - smoothstep(lineWidth - ax, lineWidth + ax, dx);
    float lineZ = 1.0 - smoothstep(lineWidth - az, lineWidth + az, dz);
    
    // combine
    float line = max(lineX, lineZ);

    groundColor = mix(groundColor, vec3(0.05,0.1,0.05), line); 
    vec3 color = mix(groundColor, skyColor, horizonFade);
    outputColor = vec4(color,1.0);
}