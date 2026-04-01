using Boids3D.Models;
using Boids3D.Utils;
using OpenTK.Mathematics;

namespace Boids3D.Chemistries;

public abstract class ChemistryBase
{
    protected const int ThreadCount = 20;
    
    protected Simulation sim;
    
    protected uint[] neighboursStart;

    protected uint[] neighboursCount;

    protected uint[] neighbours;
    
    protected bool[] done;

    protected int[] molecules;

    protected int[] moleculesStart;

    protected int[] moleculesCount;

    protected int[] moleculeParticleIndices;

    protected int moleculesCnt;
    
    private Edge[] addedEdges;

    private int addedEdgesCount;
    
    private int[] cellOffsets;

    private int[] cellCounts;

    private int[] particleIndices;

    private int currentCellCount = -1;

    private List<int>[] partitions;
    
    private NearParticlesThreadContext[] nearThreads = new NearParticlesThreadContext[ThreadCount];
    protected void InternalInitialize(double[] proportion, float[] sizes, int[] colors)
    {
        done = new bool[sim.particles.Length];
        addedEdges = new Edge[sim.particles.Length];
        for (int t = 0; t < nearThreads.Length; t++)
            nearThreads[t] = new NearParticlesThreadContext();

        molecules = new int[sim.particles.Length];
        moleculesStart = new int[sim.particles.Length];
        moleculesCount = new int[sim.particles.Length];
        moleculeParticleIndices = new int[sim.particles.Length];
        
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
    
    protected void ConnectToNear(float maxDistance, Func<int, Random, bool> shouldCheck, ShouldConnectDelegate shouldConnect, bool parallel = true)
    {
        maxDistance = maxDistance * sim.reactionDistance;
        if (parallel)
        {
            CreatePartitions();
            var totalAddedEdges = new List<Edge>();
            var locking = new object();
            foreach (var partition in partitions)
            {
                foreach(var thread in nearThreads)
                    thread.AddedEdges.Clear();
                
                ParallelHelper.ParallelProcess(nearThreads, partition, (thread, cellIndex) =>
                {
                    ConnectToNearOneCell(maxDistance, cellIndex, shouldCheck, shouldConnect, thread.Rnd, thread.AddedEdges);
                });
                
                totalAddedEdges.AddRange(nearThreads.SelectMany(t=>t.AddedEdges));
            }
            
            if (totalAddedEdges.Count > 0)
                AddEdges(totalAddedEdges.ToArray(), totalAddedEdges.Count);
        }
        else
        {
            addedEdgesCount = 0;
            for (int cellIndex = 0; cellIndex < sim.config.totalCellCount; cellIndex++)
                ConnectToNearOneCell(maxDistance, cellIndex, shouldCheck, shouldConnect, sim.rnd);

            if (addedEdgesCount > 0)
                AddEdges(addedEdges, addedEdgesCount);
        }
    }

    protected void ComputeMolecules()
    {
        var stack = new int[sim.particles.Length];
        
        int moleculeId = 0;
        int offset = 0;
        Array.Fill(molecules, -1);
        Array.Clear(moleculesCount);
        Array.Clear(moleculesStart);
        Array.Clear(moleculeParticleIndices);
        for (int idx = 0; idx < sim.particles.Length; idx++)
        {
            if (molecules[idx] == -1)
            {
                int stackTop = 0;
                moleculesStart[moleculeId] = offset;
                moleculeParticleIndices[offset] = idx;
                offset++;
                stack[stackTop] = idx;
                molecules[idx] = moleculeId;
                moleculesCount[moleculeId] = 1;
                while (stackTop >= 0)
                {
                    int p = stack[stackTop];
                    stackTop--;

                    uint neighStart = neighboursStart[p];
                    uint neighCount = neighboursCount[p];
                    for (uint i = 0; i < neighCount; i++)
                    {
                        uint neighIdx = neighStart + i;
                        uint otherIdx = neighbours[neighIdx];
                        if (molecules[otherIdx] == -1)
                        {
                            stackTop++;
                            stack[stackTop] = (int)otherIdx;
                            molecules[otherIdx] = moleculeId;
                            moleculesCount[moleculeId]++;
                            moleculeParticleIndices[offset] = (int)otherIdx;
                            offset++;
                        }
                    }
                    
                }

                moleculeId++;
            }
        }

        moleculesCnt = moleculeId;

        
        var groupped = molecules.Select((mId, particleIdx) => new {mId, particleIdx}).GroupBy(x => x.mId).ToList().OrderByDescending(x => x.Count()).ToList();
        var biggestMoleculeId = groupped.First().Key;
        var biggestMoleculeSize1 = groupped.First().Count();
        var biggesMoleculeParticles1 = groupped.First().Select(x=>x.particleIdx).ToArray();
        var biggestMoleculeSize2 = moleculesCount[biggestMoleculeId];
        var biggesMoleculeParticles2 = moleculeParticleIndices.Skip(moleculesStart[biggestMoleculeId]).Take(biggestMoleculeSize2).ToArray();
        if (biggestMoleculeSize1 != biggestMoleculeSize2 ||
            biggesMoleculeParticles1.Intersect(biggesMoleculeParticles2).Count() != biggesMoleculeParticles1.Length)
            throw new Exception("Something is wrong");

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

    private void ConnectToNearOneCell(float maxDistance, 
                                      int cellIndex, Func<int, Random, bool> shouldCheck, 
                                      ShouldConnectDelegate shouldConnect, Random rnd, 
                                      List<Edge> producedEdges = null)
    {
        if (rnd.NextSingle() > sim.reactionProbability)
            return;
        
        float maxDistanceSquared = maxDistance * maxDistance;
        int mainOffset = cellOffsets[cellIndex];
        int mainCount = cellCounts[cellIndex];
        
        int cellCount2 = sim.config.cellCount * sim.config.cellCount;
        int gridX = cellIndex % sim.config.cellCount;
        int gridY = (cellIndex / sim.config.cellCount) % sim.config.cellCount;
        int gridZ = cellIndex / (cellCount2);
        var main = new Vector3i(gridX, gridY, gridZ);

        NearParticleComparer comparer = new NearParticleComparer();
        var maxNearParticlesCount = CountParticlesInAdjacentCells(main);
        NearParticle[] near = new NearParticle[maxNearParticlesCount];
       

        for (int mainIndiceIdx = mainOffset; mainIndiceIdx < mainOffset + mainCount; mainIndiceIdx++)
        {
            int idx = particleIndices[mainIndiceIdx];
            if (!shouldCheck(idx, rnd))
                continue;

            // prepare array of near particles
            int nearCount = 0;
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
                    float distanceSquared = (sim.particles[idx].position - sim.particles[otherIdx].position).LengthSquared;
                    if (idx != otherIdx && distanceSquared < maxDistanceSquared && !AreImmediatelyConnected(idx, otherIdx))
                    {
                        near[nearCount].particleIndex = otherIdx;
                        near[nearCount].distanceSquared = distanceSquared;
                        nearCount++;
                    }
                }
            }
            
            //iterate list of ordered near particles
            Array.Sort(near, 0, nearCount, comparer);
            for (int k = 0; k < nearCount; k++)
            {
                int otherIdx = near[k].particleIndex;
                if (shouldConnect(idx, otherIdx, near[k].distanceSquared, rnd, out var length))
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
                    break;
                }
            }
            
            
            
        }
    }

    private int CountParticlesInAdjacentCells(Vector3i main)
    {
        int count = 0;
        int cellCount2 = sim.config.cellCount * sim.config.cellCount;
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
            count += cellCounts[otherCellIdx];
        }
        
        return count;
    }
}

public delegate bool ShouldConnectDelegate(int idx, int otherIdx, float distanceSquared, Random rnd, out float length);

public struct NearParticle
{
    public int particleIndex;

    public float distanceSquared;
}

public class NearParticleComparer : IComparer<NearParticle>
{
    public int Compare(NearParticle x, NearParticle y)
    {
        return x.distanceSquared < y.distanceSquared ? -1 :
            x.distanceSquared > y.distanceSquared ? 1 : 0;
    }
}

public class NearParticlesThreadContext : IThreadContext
{
    public int StartIndex { get; set; }
    public int EndIndex { get; set; }

    public Random Rnd { get; set; } = new Random();

    public List<Edge> AddedEdges { get; set; } = new List<Edge>();
}