using Boids3D.Models;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public abstract class ChemistryBase
{
    protected Simulation sim;

    private int[] cellOffsets;

    private int[] cellCounts;

    private int[] particleIndices;

    private int currentCellCount = -1;

    private List<int>[] partitions;

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

        sim.config.speciesCount = proportion.Length;
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
    
    protected void ConnectToNear(Func<int, Random, bool> shouldCheck, ShouldConnectDelegate shouldConnect, bool parallel = true)
    {
        if (parallel)
        {
            CreatePartitions();
            var totalAddedEdges = new List<Edge>();
            var locking = new object();
            foreach (var partition in partitions)
            {
                
                Parallel.ForEach(partition, cellIndex =>
                {
                    Random rnd = new Random();
                    List<Edge> threadAddedEdges = new List<Edge>();
                    ConnectToNearOneCell(cellIndex, shouldCheck, shouldConnect, rnd, threadAddedEdges);
                    lock (locking)
                    {
                        totalAddedEdges.AddRange(threadAddedEdges);
                    }
                });
            }
            
            if (totalAddedEdges.Count > 0)
                AddEdges(totalAddedEdges.ToArray(), totalAddedEdges.Count);
        }
        else
        {
            addedEdgesCount = 0;
            for (int cellIndex = 0; cellIndex < sim.config.totalCellCount; cellIndex++)
                ConnectToNearOneCell(cellIndex, shouldCheck, shouldConnect, sim.rnd);

            if (addedEdgesCount > 0)
                AddEdges(addedEdges, addedEdgesCount);
        }
    }

    private void CreatePartitions()
    {
        if (currentCellCount != sim.config.cellCount)
        {
            currentCellCount = sim.config.cellCount;
            partitions = new List<int>[27];
            for (int i = 0; i < 27; i++)
                partitions[i] = new List<int>();
            int cellCount2 = sim.config.cellCount * sim.config.cellCount;
            for (int cellIndex = 0; cellIndex < sim.config.totalCellCount; cellIndex++)
            {
                int gridX = cellIndex % sim.config.cellCount;
                int gridY = (cellIndex / sim.config.cellCount) % sim.config.cellCount;
                int gridZ = cellIndex / (cellCount2);
                int partitionIdx = (gridX % 3) * 9 + (gridY % 3) * 3 + (gridZ % 3);
                partitions[partitionIdx].Add(cellIndex);
            }
        }
    }

    private void AddEdges(Edge[] addedEdgesArg, int addedEdgesCountArg)
    {
        var newEdges = new Edge[sim.edges.Length + addedEdgesCountArg];
        Array.Copy(sim.edges, newEdges, sim.edges.Length);
        Array.Copy(addedEdgesArg, 0, newEdges, sim.edges.Length, addedEdgesCountArg);
        sim.edges = newEdges;
    }

    private void ConnectToNearOneCell(int cellIndex, Func<int, Random, bool> shouldCheck, ShouldConnectDelegate shouldConnect, Random rnd, List<Edge> producedEdges = null)
    {
        int mainOffset = cellOffsets[cellIndex];
        int mainCount = cellCounts[cellIndex];
        
        int cellCount2 = sim.config.cellCount * sim.config.cellCount;
        int gridX = cellIndex % sim.config.cellCount;
        int gridY = (cellIndex / sim.config.cellCount) % sim.config.cellCount;
        int gridZ = cellIndex / (cellCount2);
        var main = new Vector3i(gridX, gridY, gridZ);

        for (int mainIndiceIdx = mainOffset; mainIndiceIdx < mainOffset + mainCount; mainIndiceIdx++)
        {
            int idx = particleIndices[mainIndiceIdx];
            if (!shouldCheck(idx, rnd))
                continue;

            for (int dz = -1; dz <= 1; dz++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++)
            {
                Vector3i otherCell = main + new Vector3i(dx, dy, dz);
                if (otherCell.X < 0 || otherCell.X >= sim.config.cellCount ||
                    otherCell.Y < 0 || otherCell.Y >= sim.config.cellCount ||
                    otherCell.Z < 0 || otherCell.Z >= sim.config.cellCount)
                    continue;

                int otherCellIdx = otherCell.X +
                                   otherCell.Y * sim.config.cellCount +
                                   otherCell.Z * cellCount2;
                int otherOffset = cellOffsets[otherCellIdx];
                int otherCount = cellCounts[otherCellIdx];
                for (int otherIndiceIdx = otherOffset; otherIndiceIdx < otherOffset + otherCount; otherIndiceIdx++)
                {
                    int otherIdx = particleIndices[otherIndiceIdx];
                    if (idx != otherIdx && !AreImmediatelyConnected(idx, otherIdx))
                    {
                        if (shouldConnect(idx, otherIdx, rnd, out var length))
                        {
                            if (producedEdges == null)
                            {
                                addedEdges[addedEdgesCount].a = (uint)idx;
                                addedEdges[addedEdgesCount].b = (uint)otherIdx;
                                addedEdges[addedEdgesCount].restLength = length;
                                addedEdgesCount++;
                            }
                            else
                            {
                                producedEdges.Add(new Edge(){ a=(uint)idx, b=(uint)otherIdx, restLength = length});
                                
                            }

                            done[idx] = true;
                            done[otherIdx] = true;
                            goto ContinueIdx;
                        }
                    }
                }
            }
            
            ContinueIdx:
            continue;
        }
    }
    
    protected bool AreImmediatelyConnected(int idx, int idx2)
    {
        uint neighCount = neighboursCount[idx];
        if (neighCount == 0)
            return false;
        
        uint neighStart = neighboursStart[idx];
        for (uint i = 0; i < neighCount; i++)
        {
            uint neighIdx = neighStart + i;
            uint otherIdx = neighbours[neighIdx];
            if (otherIdx == idx2)
                return true;
        }

        return false;
    }
    
    protected int CountImmediateConnections(int idx, int type)
    {
        uint neighCount = neighboursCount[idx];
        if (neighCount == 0)
            return 0;
        
        int count = 0;
        uint neighStart = neighboursStart[idx];
        for (uint i = 0; i < neighCount; i++)
        {
            uint neighIdx = neighStart + i;
            uint otherIdx = neighbours[neighIdx];
            if (sim.particles[otherIdx].type == type)
                count++;
        }

        return count;
    }
}

public delegate bool ShouldConnectDelegate(int idx, int otherIdx, Random rnd, out float length);