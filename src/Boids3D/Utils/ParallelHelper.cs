namespace Boids3D.Utils;

public static class ParallelHelper
{
    public static void ParallelProcess<TContext, TData>(TContext[] threads, List<TData> data, Action<TContext, TData> process)
     where TContext : IThreadContext
    {
        if (data == null) throw new ArgumentNullException(nameof(data));
        if (process == null) throw new ArgumentNullException(nameof(process));
        int numberOfThreads = threads.Length;

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

            threads[threadIndex].StartIndex = start;
            threads[threadIndex].EndIndex = end;
            for (int i = start; i < end; i++)
            {
                process(threads[threadIndex], data[i]);
            }
        });
    }
}

public interface IThreadContext
{
    public int StartIndex { get; set; }
    
    public int EndIndex { get; set; }
}
