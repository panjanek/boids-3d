using Boids3D.Models;
using OpenTK.Graphics.ES30;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public class HydroCarbonPolimerization : ChemistryBase, IChemistry
{
    public void Initialize(Simulation sim)
    {
        this.sim = sim;
        InternalInitialize([1, 3], [2,1], [0, 1]);
    }

    protected override void InternalReact()
    {
        ComputeMolecules();
        ConnectToNear(15, (idx, rnd) =>
            {
                if (done[idx])
                    return false;
                
                var p = sim.particles[idx];
                if (p.type == 1 && neighboursCount[idx] >= 1) //hydrogen: only one link
                    return false;
                
                if (p.type == 0 && neighboursCount[idx] >= 4)  //carbon: max 4
                    return false;

                return true;
            },
            (int idx, int otherIdx, float distanceSquared, Random rnd, out float length) =>
            {
                length = 3;
                if (done[otherIdx])
                    return false;
                
                if (molecules[idx] == molecules[otherIdx])
                    return false;

                var p = sim.particles[idx];
                var other = sim.particles[otherIdx];
                length = p.type == other.type ? 3 : 1.5f;

                if (p.type == 1 && other.type == 1)
                    return false;

                if (other.type == 0 && neighboursCount[otherIdx] >= 4)
                    return false;

                if (other.type == 1 && neighboursCount[otherIdx] >= 1)
                    return false;

                if (p.type == 0 && other.type == 0 && (CountImmediateConnections(idx, 0) >= 2 ||
                                                       CountImmediateConnections(otherIdx, 0) >= 2))
                    return false;
                
                return true;
            });

        /*
        var dupl = sim.edges.GroupBy(e =>
        new {
            e.a, e.b
        }).Where(g=>g.Count() > 1).ToList();
        if (dupl.Count() > 0)
        {
            var a = dupl.Count();
            Console.WriteLine(dupl);
        }*/
    }
}