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
        ConnectToNear(15, (idx, rnd) =>
            {
                var p = sim.particles[idx];
                
                if (p.type == 0 && neighboursCount[idx] >= 4)  //carbon: max 4
                    return false;
                
                if (IsTerminalHydrogen(idx))
                    return true;
                
                if (p.type == 1 && neighboursCount[idx] >= 1) //hydrogen: only one link
                    return false;

                return true;
            },
            (int idx, int otherIdx, float distanceSquared, Random rnd, out float length) =>
            {
                length = 3;
                
                if (molecules[idx] == molecules[otherIdx])
                    return false;

                var p = sim.particles[idx];
                var other = sim.particles[otherIdx];
                length = p.type == other.type ? 3 : 1.5f;

                if (IsTerminalHydrogen(idx) && IsTerminalHydrogen(otherIdx))
                {
                    length = 1.5f;
                    return true;
                }
                
                if (p.type == 0 && neighboursCount[idx] >= 4)
                    return false;
                
                if (p.type == 1 && neighboursCount[idx] >= 1)
                    return false;

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

    private bool IsTerminalHydrogen(int idx)
    {
        var p = sim.particles[idx];
        if (p.type == 1 && neighboursCount[idx] == 1)
        {
            int moleculeId = molecules[idx];
            if (CountInMolecule(moleculeId, 0) > 3)
            {
                var carbonIdx = (int)neighbours[neighboursStart[idx]];
                if (sim.particles[carbonIdx].type == 0)
                {
                    if (neighboursCount[carbonIdx] == 4 &&
                        CountImmediateConnections(carbonIdx, 1) == 3 &&
                        CountImmediateConnections(carbonIdx, 0) == 1)
                    {
                        bool hasHydrogenWith2Bonds = false;
                        IterateImmediateConnections(carbonIdx, c =>
                        {
                            if (sim.particles[c].type == 1 && neighboursCount[c] != 1)
                                hasHydrogenWith2Bonds = true;

                        });
                        
                        return !hasHydrogenWith2Bonds;
                    }
                }
            }
        }

        return false;
    }
}