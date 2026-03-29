using System;
using System.Collections.Generic;
using System.DirectoryServices.ActiveDirectory;
using System.Linq;
using System.Security.RightsManagement;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;
using System.Threading.Tasks.Sources;
using Boids3D.Chemistries;
using OpenTK.Audio.OpenAL;
using OpenTK.Mathematics;

namespace Boids3D.Models
{
    public class Simulation
    {
        public const int MaxSpeciesCount = 6;

        public ShaderConfig config;

        public float particleSize = 0.3f;

        public float fogDensity = 0.0002f;

        public float forwardMove = 0.0f;

        public float lineWidth = 300f;

        [JsonIgnore]
        public Particle[] particles;
        
        public Edge[] edges = new Edge[0];

        public int seed = 11;

        public float followDistance = 75;

        public IChemistry chemistry;
        
        public Random rnd = new Random(123);
        

        public Simulation()
        {
            config = new ShaderConfig();
        }

        public void StartSimulation(int particlesCount, int speciesCount, float size)
        {
            var previousSpeciesCount = config.speciesCount;
            config.speciesCount = speciesCount;
            config.fieldSize = size;
            config.particleCount = particlesCount;
            InitializeParticles(particlesCount);
        }
        

        public void InitializeParticles(int count)
        {
            if (particles == null || particles.Length != count)
                particles = new Particle[count];
            edges = new Edge[0];

            chemistry.Initialize(this);
        }
    }
}
