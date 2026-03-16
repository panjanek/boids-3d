using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Boids3D.Models
{
    [StructLayout(LayoutKind.Explicit, Size = 96)]
    public unsafe struct ShaderConfig
    {
        public ShaderConfig()
        {

        }

        [FieldOffset(0)] public int particleCount = 0;

        [FieldOffset(4)] public float dt = 0.025f;

        [FieldOffset(8)] public float separationRadius = 5f;  

        [FieldOffset(12)] public float t = 0;

        [FieldOffset(16)] public float randomization = 0.75f;

        [FieldOffset(20)] public float fieldSize = 800;

        [FieldOffset(24)] public float cellSize = 0;

        [FieldOffset(28)] public float maxDist = 60;

        [FieldOffset(32)] public int speciesCount = 0;

        [FieldOffset(36)] public float alignRadius = 30f;   

        [FieldOffset(40)] public int trackedIdx;

        [FieldOffset(44)] public float wallForce = 10;

        [FieldOffset(48)] public float cohesionRadius = 30f;     

        [FieldOffset(52)] public int cellCount = 0;

        [FieldOffset(56)] public int totalCellCount = 0;

        [FieldOffset(60)] public float separationForce = 1.5f;   

        [FieldOffset(64)] public float alignForce = 1.0f;           

        [FieldOffset(68)] public float cohesionForce = 0.8f;   

        [FieldOffset(72)] public float maxSpeed = 10f; 

        [FieldOffset(76)] public float separationRadius2;
        [FieldOffset(80)] public float alignRadius2;
        [FieldOffset(84)] public float cohesionRadius2;
        [FieldOffset(88)] public float fov = -0.707f;
        [FieldOffset(92)] public float fovDeg = 135;
    }
}
