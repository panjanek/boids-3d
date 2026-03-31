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
        ConnectToNear(30, (idx, rnd) =>
            {
                return neighboursCount[idx] < 2 && !done[idx];
            },
            (int idx, int otherIdx, float distanceSquared, Random rnd, out float length) =>
            {
                length = 3;
                if (done[otherIdx] || neighboursCount[otherIdx] >= 2)
                    return false;

                return true;
            });
    }
}