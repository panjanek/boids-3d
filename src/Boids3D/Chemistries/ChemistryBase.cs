using Boids3D.Models;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public abstract class ChemistryBase
{
    public void Initialize(Simulation sim)
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
    }
}