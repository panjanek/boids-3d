using Boids3D.Models;

namespace Boids3D.Chemistries;

public interface IChemistry
{
    void Initialize(Simulation sim);

    void React(int[] cellOffsets, int[] cellCounts, int[] particleIndices, 
               uint[] neighboursStart, uint[] neighboursCount, uint[] neighbours, int[] edgeIndices,
               int[] molecules, int[] moleculesStart, int[] moleculesCount, int[] moleculeParticleIndices, int moleculesCnt);
}