using Boids3D.Models;
using OpenTK.Graphics.ES30;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public class PolimerizationChemistry : ChemistryBase, IChemistry
{
    public void Initialize(Simulation sim)
    {
        this.sim = sim;
        InternalInitialize([3,2,1],[1,2,3]);
    }

    protected override void InternalReact()
    {
        ConnectToNear(idx =>
            {
                return neighboursCount[idx] < 2 && !done[idx];
            },
            (idx, otherIdx) =>
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