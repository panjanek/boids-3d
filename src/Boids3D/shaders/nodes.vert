#version 430

struct Node
{
   vec4 position;
   vec4 velocity;
   int species;
   int flags;
   int  cellIndex;
   float xzAngle;
   float yAngle;
   int _pad0;
   int _pad1;
   int _pad2;
   vec4 direction;
};

layout(std430, binding = 2) buffer NodesBuffer {
    Node points[];
};

uniform mat4 view;
uniform mat4 projection;
uniform float paricleSize;
uniform vec2 viewportSize;
uniform float unhighlightedAlpha;

layout(location = 0) out vec3 vColor;
layout(location = 1) out float vDepth;
layout(location = 2) out float vFadingAlpha;
layout(location = 3) out vec3 vCenterView;
layout(location = 4) out vec2 vQuad;
layout(location = 5) in vec2 quadPos;
layout(location = 6) out vec3 vOffsetView;
layout(location = 7) out float vSphereRadiusMult;

void main()
{
    float sphereRadius = 2 * paricleSize + (viewportSize.x/1920);
    uint id = gl_InstanceID;
    Node p = points[id];
    vSphereRadiusMult = 1.0;

    //coloring
    const vec3 colors[] = vec3[](
        vec3(1.0, 1.0, 0.0), // yellow
        vec3(1.0, 0.0, 1.0), // magenta
        vec3(0.0, 1.0, 1.0), // cyan
        vec3(1.0, 0.0, 0.0), // red
        vec3(0.0, 1.0, 0.0), // green
        vec3(0.0, 0.0, 1.0), // blue
        vec3(1.0, 1.0, 1.0), // white
        vec3(0.5, 0.5, 0.5)  // gray
    );
    int colorIdx = p.species;
    vColor = colorIdx >= 0 ? colors[colorIdx % 8] : vec3(1.0, 0.0, 0.0);
    vFadingAlpha = 1.0;

    sphereRadius *= vSphereRadiusMult;

    //hide
    //if (p.flags == 2) sphereRadius = 0;

    //real spheres
    vec4 viewPos = view * vec4(p.position.xyz, 1.0);
    vCenterView = viewPos.xyz;
    vQuad = quadPos;

    // In VIEW SPACE the camera basis is fixed
    float inflate = 1.5;
    vec3 offset = vec3(quadPos * sphereRadius * inflate, 0.0);

    vOffsetView = offset;

    vec4 pos = viewPos + vec4(offset, 0.0);
    gl_Position = projection * pos;
}