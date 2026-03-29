using Boids3D.Models;

namespace Boids3D.Chemistries;

public interface IChemistry
{
    void Initialize(Simulation sim);

    void React();
}