namespace Boids3D.Utils;

public static class ParallelHelper
{
    public static void ParallelProcess<T>(int numberOfThreads, List<T> data, Action<T> process)
    {
        if (data == null) throw new ArgumentNullException(nameof(data));
        if (process == null) throw new ArgumentNullException(nameof(process));
        if (numberOfThreads <= 0) throw new ArgumentOutOfRangeException(nameof(numberOfThreads));

        int length = data.Count;
        if (length == 0) return;

        // Clamp thread count to data size
        numberOfThreads = Math.Min(numberOfThreads, length);

        int chunkSize = (length + numberOfThreads - 1) / numberOfThreads;

        var options = new ParallelOptions
        {
            MaxDegreeOfParallelism = numberOfThreads
        };

        Parallel.For(0, numberOfThreads, options, threadIndex =>
        {
            int start = threadIndex * chunkSize;
            int end = Math.Min(start + chunkSize, length);

            for (int i = start; i < end; i++)
            {
                process(data[i]);
            }
        });
    }
}