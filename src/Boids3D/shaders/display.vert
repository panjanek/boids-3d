#version 430

struct Particle
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

layout(std430, binding = 2) buffer OutputBuffer {
    Particle points[];
};

uniform mat4 view;
uniform mat4 projection;
uniform float particleSize;
uniform float fogDensity;

layout(location = 0) out vec3 vColor;
layout(location = 1) out float vFadingAlpha;
layout(location = 2) out float vLighting;

vec3 tetraVerts[4] = vec3[](
    vec3(0.0, 0.0, 4.0),     // tip
    vec3(-1.0, -1.0, -1.0),
    vec3( 1.0, -1.0, -1.0),
    vec3( 0.0,  1.0, -1.0)
);

int indices[12] = int[](
    0,1,2,
    0,2,3,
    0,3,1,
    1,3,2
);

vec3 hsv2rgb(vec3 c)
{
    vec3 p = abs(fract(c.xxx + vec3(0.0, 2.0/3.0, 1.0/3.0)) * 6.0 - 3.0);
    return c.z * mix(vec3(1.0), clamp(p - 1.0, 0.0, 1.0), c.y);
}

vec3 directionToColor(vec3 dir)
{
    float azimuth = atan(dir.x, dir.z);
    float elevation = asin(clamp(dir.y, -1.0, 1.0));
    float h1 = azimuth * (1.0 / (2.0 * 3.14159265)) + 0.5;
    float h2 = elevation * (1.0 / 3.14159265) + 0.5;
    float hue = fract(h1 + 0.25 * dir.y);
    vec3 hsv = vec3(hue, 1.0, 1.0);
    return hsv2rgb(hsv);
}

float amplify(float x, int pow)
{
    float a = 1;
    for(int i=0; i<pow; i++)
        a = a * (1-x);

    return 1-a;
}

void main()
{
    vec3 lightDir = vec3(1,1,-1);
    uint particleID = gl_InstanceID;
    Particle p = points[particleID];
    
    float sizeAmp = 1.0;
    if (p.flags == 1)
        sizeAmp = 1.5;

    vec3 pos = p.position.xyz;
    vec3 dir = normalize(p.direction.xyz);

    vec3 helper = abs(dir.y) > 0.99 ? vec3(1,0,0) : vec3(0,1,0);
    vec3 right = normalize(cross(helper, dir));
    vec3 up = cross(dir, right);

    mat3 rot = mat3(right, up, dir);

    int tri = gl_VertexID / 3;
    int triVertex = gl_VertexID % 3;

    int i0 = indices[tri*3 + 0];
    int i1 = indices[tri*3 + 1];
    int i2 = indices[tri*3 + 2];

    vec3 v0 = rot * tetraVerts[i0] * particleSize * sizeAmp;
    vec3 v1 = rot * tetraVerts[i1] * particleSize * sizeAmp;
    vec3 v2 = rot * tetraVerts[i2] * particleSize * sizeAmp;

    vec3 normal = normalize(cross(v1 - v0, v2 - v0));

    float lighting = max(dot(normal, normalize(lightDir)), 0.0);

    vec3 local;
    if(triVertex == 0) local = v0;
    if(triVertex == 1) local = v1;
    if(triVertex == 2) local = v2;

    vec3 worldPos = pos + local;

    vec4 viewPos = view * vec4(worldPos,1.0);
    gl_Position = projection * viewPos;

    float distance = max(-viewPos.z, 0.001);

    const vec3 colors[] = vec3[](
        vec3(0.2, 1.0, 0.2), // green
        vec3(0.2, 0.0, 1.0), // blue
        vec3(1.0, 0.2, 0.2), // red
        vec3(1.0, 0.2, 1.0), // magenta
        vec3(1.0, 1.0, 0.2), // yellow     
        vec3(0.2, 1.0, 1.0), // cyan
        vec3(1.0, 1.0, 1.0), // white
        vec3(0.5, 0.5, 0.5)  // gray
    );

    vColor = colors[tri % 8];
    
    vec3 colorSeed = p.direction.xyz;
    colorSeed.x += triVertex*0.2;
    colorSeed.y += (particleID % 10) * 0.03;
    vColor = directionToColor(normalize(colorSeed));
    
    int amp = 2;
    if (p.flags == 1)
    {
        vColor = vec3(1-vColor.r, 1-vColor.g, 1-vColor.b);
        amp = 4;
    }
    
    vColor.r = amplify(vColor.r, amp);
    vColor.g = amplify(vColor.g, amp);
    vColor.b = amplify(vColor.b, amp);

    
    vFadingAlpha = exp(-fogDensity * distance);
    
    //float viewTerm = clamp(dot(normalize(-viewPos.xyz), normal),0,1);
    //lighting *= 0.7 + 0.3 * viewTerm; //optional
    vLighting = lighting;
}