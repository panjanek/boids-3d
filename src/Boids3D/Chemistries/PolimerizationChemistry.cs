using Boids3D.Models;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public class PolimerizationChemistry : ChemistryBase, IChemistry
{
    private bool[] done;

    public override void Initialize(Simulation sim)
    {
        base.Initialize(sim);
        done = new bool[sim.particles.Length];
    }


    public void React(Simulation sim, int[] cellOffsets, int[] cellCounts, int[] particleIndices, uint[] neighboursStart, uint[] neighboursCount, uint[] neighbours)
    {
        Array.Clear(done, 0, done.Length);
        var newEdgesTotal = new List<Edge>();
        for (int idx = 0; idx < sim.particles.Length; idx++)
        {
            var newEdges = CheckOne(sim, idx, cellOffsets, cellCounts, particleIndices, neighboursStart, neighboursCount, neighbours);
            newEdgesTotal.AddRange(newEdges);
        }
        
        var list = sim.edges.ToList();
        list.AddRange(newEdgesTotal);
        sim.edges = list.ToArray();
    }

    private List<Edge> CheckOne(Simulation sim, int idx, int[] cellOffsets, int[] cellCounts, int[] particleIndices, uint[] neighboursStart, uint[] neighboursCount, uint[] neighbours)
    {
        var newEdges = new List<Edge>();
        uint neighCount = neighboursCount[idx];
        if (neighCount >= 2 || done[idx])
            return newEdges;
        
        var p = sim.particles[idx];
        
        int cellCount2 = sim.config.cellCount * sim.config.cellCount;
        int gridX = p.cellIndex % sim.config.cellCount;
        int gridY = (p.cellIndex / sim.config.cellCount) % sim.config.cellCount;
        int gridZ = p.cellIndex / (cellCount2);
        var main = new Vector3i(gridX, gridY, gridZ);
        for (int dz = -1; dz <= 1; dz++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
        {
            Vector3i cell = main + new Vector3i(dx, dy, dz);
            if (cell.X < 0 || cell.X >= sim.config.cellCount || 
                cell.Y < 0 || cell.Y >= sim.config.cellCount ||
                cell.Z < 0 || cell.Z >= sim.config.cellCount)
                continue;
        
            int cellIdx = cell.X +
                          cell.Y * sim.config.cellCount +
                          cell.Z * cellCount2;
            int offset = cellOffsets[cellIdx];
            int count = cellCounts[cellIdx];

            for (int indiceIdx = offset; indiceIdx < offset + count; indiceIdx++)
            {
                int otherIdx = particleIndices[indiceIdx];
                if (idx != otherIdx && !done[otherIdx])
                {
                    var other = sim.particles[otherIdx];
                    
                    
                    var distance = (p.position - other.position).Length;
                    if (distance < 5)
                    {
                        newEdges.Add(new Edge() { a = (uint)idx, b = (uint)otherIdx, restLength = 3 });
                        done[idx] = true;
                        done[otherIdx] = true;
                    }
                    
                    
                }
            }
        }
        
        return newEdges;
    }
}