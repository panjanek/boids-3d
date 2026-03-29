using Boids3D.Models;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public abstract class ChemistryBase
{
    public virtual void Initialize(Simulation sim)
    {
        for(int i=0; i< sim.config.particleCount; i++)
        {
            sim.particles[i].position = new Vector4(sim.config.fieldSize * sim.rnd.NextSingle(), sim.config.fieldSize * sim.rnd.NextSingle(), sim.config.fieldSize * sim.rnd.NextSingle(), 0);
            sim.particles[i].species = sim.rnd.Next(sim.config.speciesCount);

            var dir = new Vector4(sim.rnd.NextSingle() * 2 - 1, sim.rnd.NextSingle() * 2 - 1, sim.rnd.NextSingle() * 2 - 1, 0);
            dir.Normalize();
            sim.particles[i].direction = dir;
            sim.particles[i].velocity = dir * (10f + sim.rnd.NextSingle() * 20);
        }
        /*
        sim.edges = new Edge[sim.config.particleCount/2];
        for (int e = 0; e < sim.edges.Length; e++)
        {
            sim.edges[e].a = (uint)sim.rnd.Next(sim.particles.Length);
            sim.edges[e].b = (uint)sim.rnd.Next(sim.particles.Length);
            sim.edges[e].restLength = 3;
        }*/
    }
}