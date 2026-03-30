using Boids3D.Models;
using OpenTK.Graphics.ES30;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public class SimplePolimerization : ChemistryBase, IChemistry
{
    public void Initialize(Simulation sim)
    {
        this.sim = sim;
        InternalInitialize([1],[1], [0]);
    }

    protected override void InternalReact()
    {
        ConnectToNear((idx, rnd) =>
            {
                //if (sim.rnd.NextDouble() < 0.97)
                //    return false;
                
                
                return neighboursCount[idx] < 2 && !done[idx];
            },
            (idx, otherIdx, rnd) =>
            {
                if (done[otherIdx] || neighboursCount[otherIdx] >= 2)
                    return -1;

                var p = sim.particles[idx];
                var other = sim.particles[otherIdx];
                var distance = (p.position - other.position).Length;
                return (distance < 30 * sim.reactionDistance) ? 3 : -1;
            });
        
    }
}