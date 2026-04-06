using Boids3D.Models;
using OpenTK.Graphics.ES30;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public class TestingForcesChemistry : ChemistryBase, IChemistry
{
    public void Initialize(Simulation sim)
    {
        this.sim = sim;
        InternalInitialize([1],[1], [0]);
        sim.InitializeDefaultForces();
        sim.SetSimpleForce(true, 0, 0, -1, 1);
    }

    protected override void InternalReact()
    {
        ConnectToNear(30, (idx, rnd) =>
            {
                return neighboursCount[idx] < 2;
                //return false;
            },
            (int idx, int otherIdx, float distanceSquared, Random rnd, out float length) =>
            {
                length = 3;
                
                if (molecules[idx] == molecules[otherIdx])
                    return false;
                
                if (neighboursCount[otherIdx] >= 2)
                    return false;

                return true;
            });
    }
}