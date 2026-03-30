using Boids3D.Models;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public abstract class ChemistryBase
{
    protected Simulation sim;

    private int[] cellOffsets;

    private int[] cellCounts;

    private int[] particleIndices;

    protected uint[] neighboursStart;

    protected uint[] neighboursCount;

    protected uint[] neighbours;
    
    protected bool[] done;

    private Edge[] addedEdges;

    private int addedEdgesCount;
    protected void InternalInitialize(double[] proportion, float[] sizes, int[] colors)
    {
        done = new bool[sim.particles.Length];
        addedEdges = new Edge[sim.particles.Length];
        
        var propTotal = proportion.Sum();
        proportion = proportion.Select(x => x / propTotal).ToArray();
        var propSums = new double[proportion.Length];
        propSums[0] = proportion[0];
        for (int p = 1; p < proportion.Length; p++)
            propSums[p] = propSums[p - 1] + proportion[p];
        
        for(int i=0; i< sim.config.particleCount; i++)
        {
            sim.particles[i].position = new Vector4(sim.config.fieldSize * sim.rnd.NextSingle(), sim.config.fieldSize * sim.rnd.NextSingle(), sim.config.fieldSize * sim.rnd.NextSingle(), 0);

            var typeRand = sim.rnd.NextDouble();
            for(int p=0; p<propSums.Length;p++)
                if (typeRand < propSums[p])
                {
                    sim.particles[i].type = p;
                    break;
                }


            sim.particles[i].size = sizes[sim.particles[i].type];
            sim.particles[i].color = colors[sim.particles[i].type];

            var dir = new Vector4(sim.rnd.NextSingle() * 2 - 1, sim.rnd.NextSingle() * 2 - 1, sim.rnd.NextSingle() * 2 - 1, 0);
            dir.Normalize();
            sim.particles[i].direction = dir;
            sim.particles[i].velocity = dir * (10f + sim.rnd.NextSingle() * 20);
        }
    }

    public virtual void React(int[] cellOffsets, int[] cellCounts, int[] particleIndices, uint[] neighboursStart, uint[] neighboursCount, uint[] neighbours)
    {
        this.cellOffsets = cellOffsets;
        this.cellCounts = cellCounts;
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
        addedEdgesCount = 0;
        for (int idx = 0; idx < sim.particles.Length; idx++)
        {
            if (shouldCheck(idx))
            {
                if (ConnectToNearOne(idx, shouldConnect, out var otherIdx, out var length))
                {
                    addedEdges[addedEdgesCount].a = (uint)idx;
                    addedEdges[addedEdgesCount].b = (uint)otherIdx;
                    addedEdges[addedEdgesCount].restLength = length;
                    addedEdgesCount++;
                    done[idx] = true;
                    done[otherIdx] = true;
                }
            }
        }
        

        if (addedEdgesCount > 0)
        {
            var newEdges = new Edge[sim.edges.Length + addedEdgesCount];
            Array.Copy(sim.edges, newEdges, sim.edges.Length);
            Array.Copy(addedEdges, 0, newEdges, sim.edges.Length, addedEdgesCount);
            sim.edges = newEdges;
        }
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