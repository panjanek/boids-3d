using Boids3D.Models;
using OpenTK.Graphics.ES30;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public class RingsChemistry : ChemistryBase, IChemistry
{
    public void Initialize(Simulation sim)
    {
        this.sim = sim;
        InternalInitialize([1, 1], [2,1], [0, 1]);
        sim.InitializeDefaultForces();
        
        sim.SetSimpleForce(true, 1, 1, 2f, 1f);
    }

    protected override void InternalReact()
    {
        int minRingSize = 8;
        float ccLen = 5f;
        float chLen = 2.5f;
        
        ConnectToNear(30, (idx, rnd) =>
            {
                return sim.particles[idx].type == 0;
            },
            (int idx, int otherIdx, float distanceSquared, Random rnd, out float length) =>
            {
                length = 3;

                var moleculeId = molecules[idx];
                var otherMoleculeId = molecules[otherIdx];
                
                /*
                if (CountInMolecule(moleculeId, 0) > ringSize)
                    return false;
                
                if (CountInMolecule(moleculeId, 0) + CountInMolecule(otherMoleculeId, 0) > ringSize)
                    return false;
                */

                if (sim.particles[otherIdx].type == 0)
                {
                    if (CountImmediateConnections(idx, 0) + CountImmediateConnections(otherIdx, 0) < 2)
                    {
                        length = ccLen;
                        return true;
                    }

                    if (CountImmediateConnections(idx, 0) == 1 && 
                        CountImmediateConnections(otherIdx, 0) == 1 &&
                        moleculeId != otherMoleculeId) //&&
                        //CountInMolecule(moleculeId, 0) + CountInMolecule(otherMoleculeId, 0) >= minRingSize)
                    {
                        length = ccLen;
                        return true;
                    }
                }
                else
                {
                    if (neighboursCount[otherIdx] == 0 && CountImmediateConnections(idx, 1) == 0)
                    {
                        length = chLen;
                        return true;
                    }
                }

                return false;
            });

        IterateMolecules((moleculeId, rnd, added, removed) =>
        {
            if (CountInMolecule(moleculeId, 0) >= minRingSize)
            {
                List<int> terminal = new List<int>();
                IterateMolecule(moleculeId, pIDx =>
                {
                    if (sim.particles[pIDx].type == 0 && CountImmediateConnections(pIDx, 0) == 1)
                        terminal.Add(pIDx);
                });

                if (terminal.Count == 2)
                {
                    added.Add(new Edge(){ a = (uint)terminal[0], b = (uint)terminal[1], restLength = ccLen });
                }
            }
            
        });
    }
}