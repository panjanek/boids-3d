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
        public const int KeypointsCount = 3;

        public ShaderConfig config;

        public float particleSize = 0.3f;

        public float fogDensity = 0.0002f;

        public float forwardMove = 0.0f;
        
        public float rotationSpeed = 0.001f;

        public float lineWidth = 300f;
        
        public float reactionDistance = 1f;

        public float reactionProbability = 0.5f;

        public float unhighlightAlpha = 0.3f;

        public int reactionsFrequency = 100;

        [JsonIgnore]
        public Particle[] particles;
        
        public Edge[] edges = new Edge[0];
        
        public Vector4[] forces;

        public int seed = 11;

        public float followDistance = 75;

        public IChemistry chemistry;
        
        public Random rnd = new Random(123);
        

        public Simulation()
        {
            config = new ShaderConfig();
        }

        public void StartSimulation(int particlesCount, float size)
        {
            var previousSpeciesCount = config.speciesCount;
            config.fieldSize = size;
            config.particleCount = particlesCount;
            InitializeParticles(particlesCount);
        }
        
        public int GetForceOffset(bool sameMolecule, int specMe, int specOther)
        {
            int baseOffset = sameMolecule ? config.speciesCount * config.speciesCount * KeypointsCount : 0;
            int offset = baseOffset + (specMe * config.speciesCount + specOther) * KeypointsCount;
            return offset;
        }
        
        private void SetSimpleForce(bool sameMolecule, int specMe, int specOther, float val0, float val1)
        {
            int offset = GetForceOffset(sameMolecule, specMe, specOther);
            var d = config.maxDist / KeypointsCount;
            forces[offset + 0] = new Vector4(0 * d, val0, 0, 0);
            forces[offset + 1] = new Vector4(1 * d, 0, 0, 0);
            forces[offset + 2] = new Vector4(2 * d, val1, 0, 0);
        }
        
        public void InitializeDefaultForces()
        {
            forces = new Vector4[2 * config.speciesCount * config.speciesCount * KeypointsCount];
            for (int i = 0; i < config.speciesCount; i++)
            {
                for (int j = 0; j < config.speciesCount; j++)
                {
                    SetSimpleForce(false, i, j, -1, 0);
                    SetSimpleForce(true, i, j, -1, 0);
                }
            }
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
