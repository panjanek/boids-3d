using Boids3D.Models;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public abstract class ChemistryBase
{
    protected Simulation sim;

    protected int[] cellOffsets;

    protected int[] cellCounts;

    protected int[] particleIndices;

    protected uint[] neighboursStart;

    protected uint[] neighboursCount;

    protected uint[] neighbours;
    
    protected bool[] done;
    protected void InternalInitialize()
    {
        this.sim = sim;
        done = new bool[sim.particles.Length];
        for(int i=0; i< sim.config.particleCount; i++)
        {
            sim.particles[i].position = new Vector4(sim.config.fieldSize * sim.rnd.NextSingle(), sim.config.fieldSize * sim.rnd.NextSingle(), sim.config.fieldSize * sim.rnd.NextSingle(), 0);
            sim.particles[i].species = sim.rnd.Next(sim.config.speciesCount);

            var dir = new Vector4(sim.rnd.NextSingle() * 2 - 1, sim.rnd.NextSingle() * 2 - 1, sim.rnd.NextSingle() * 2 - 1, 0);
            dir.Normalize();
            sim.particles[i].direction = dir;
            sim.particles[i].velocity = dir * (10f + sim.rnd.NextSingle() * 20);
        }
        /*
        sim.edges = new Edge[sim.config.particleCount/2];
        for (int e = 0; e < sim.edges.Length; e++)
        {
            sim.edges[e].a = (uint)sim.rnd.Next(sim.particles.Length);
            sim.edges[e].b = (uint)sim.rnd.Next(sim.particles.Length);
            sim.edges[e].restLength = 3;
        }*/
    }

    public virtual void React(int[] cellOffsets, int[] cellCounts, int[] particleIndices, uint[] neighboursStart, uint[] neighboursCount, uint[] neighbours)
    {
        this.cellOffsets = cellOffsets;
        this. cellCounts = cellCounts;
        this.particleIndices = particleIndices;
        this.neighboursStart = neighboursStart;
        this.neighboursCount = neighboursCount;
        this.neighbours = neighbours;
        Array.Clear(done, 0, done.Length);
        InternalReact();
    }

    protected abstract void InternalReact();
    
    protected void ConnectToNear(Func<int, bool> shouldCheck, Func<int, int, float> shouldConnect)
    {
        var newEdges = new List<Edge>();
        for (int idx = 0; idx < sim.particles.Length; idx++)
        {
            if (shouldCheck(idx))
            {
                if (ConnectToNearOne(idx, shouldConnect, out var otherIdx, out var length))
                {
                    newEdges.Add(new Edge() { a = (uint)idx, b = (uint)otherIdx, restLength = length });
                    done[idx] = true;
                    done[otherIdx] = true;
                }
            }
        }
        
        var list = sim.edges.ToList();
        list.AddRange(newEdges);
        sim.edges = list.ToArray();
    }


    private bool ConnectToNearOne(int idx, Func<int, int, float> shouldConnect, out int connectTo, out float length)
    {
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
                if (idx != otherIdx)
                {
                    var connection = shouldConnect(idx, otherIdx);
                    if (connection > 0)
                    {
                        connectTo = otherIdx;
                        length = connection;
                        return true;
                    }
                }
            }
        }

        connectTo = -1;
        length = 0;
        return false;
    }
}