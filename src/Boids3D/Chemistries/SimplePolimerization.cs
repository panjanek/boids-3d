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
                return neighboursCount[idx] < 2 && !done[idx];
            },
            (int idx, int otherIdx, Random rnd, out float length) =>
            {
                length = 3;
                if (done[otherIdx] || neighboursCount[otherIdx] >= 2)
                    return false;

                var p = sim.particles[idx];
                var other = sim.particles[otherIdx];
                var distance = (p.position - other.position).Length;
                return distance < 30 * sim.reactionDistance;
            });
        
    }
}