using System;
using System.Collections.Generic;
using System.DirectoryServices.ActiveDirectory;
using System.Linq;
using System.Security.RightsManagement;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;
using System.Threading.Tasks.Sources;
using OpenTK.Audio.OpenAL;
using OpenTK.Mathematics;

namespace Boids3D.Models
{
    public class Simulation
    {
        public const int MaxSpeciesCount = 6;

        public ShaderConfig config;

        public float particleSize = 0.7f;

        public float fogDensity = 0.0005f;

        public float forwardMove = 0.0f;

        [JsonIgnore]
        public Particle[] particles;

        public int seed = 11;

        public float followDistance = 75; 
        

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

            var rnd = new Random(1);
            for(int i=0; i< count; i++)
            {
                particles[i].position = new Vector4(config.fieldSize * rnd.NextSingle(), config.fieldSize * rnd.NextSingle(), config.fieldSize * rnd.NextSingle(), 0);
                particles[i].species = rnd.Next(config.speciesCount);

                var dir = new Vector4(rnd.NextSingle() * 2 - 1, rnd.NextSingle() * 2 - 1, rnd.NextSingle() * 2 - 1, 0);
                dir.Normalize();
                particles[i].direction = dir;
                particles[i].velocity = dir * (10f + rnd.NextSingle() * 20);

                
                /*
                var center = new Vector4(config.fieldSize / 2, config.fieldSize / 2, config.fieldSize / 2, 0);
                var radius = config.fieldSize / 2;
                while ((particles[i].position - center).Length > radius)
                    particles[i].position = new Vector4(config.fieldSize * rnd.NextSingle(), config.fieldSize * rnd.NextSingle(), config.fieldSize * rnd.NextSingle(), 0);


                var inward = center - particles[i].position;
                inward.Normalize();
                particles[i].direction = inward;
                
                particles[i].velocity = dir * (1f + rnd.NextSingle() * 2);
                */

            }
        }
    }
}
