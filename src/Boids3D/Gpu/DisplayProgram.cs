using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms.VisualStyles;
using System.Windows.Media.Media3D;
using System.Windows.Media.TextFormatting;
using Boids3D.Gpu;
using Boids3D.Models;
using Boids3D.Utils;
using OpenTK.GLControl;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using AppContext = Boids3D.Models.AppContext;

namespace Boids3D.Gpu
{
    public class DisplayProgram
    {
        private int nodesProgram;

        private int projNodesLocation;

        private int viewportSizeNodesLocation;

        private int viewportSizeEdgesLocation;

        private int particleSizeLocation;

        private int viewNodesLocation;

        private int viewEdgesLocation;

        private int lineWidthLocation;

        private int fogDensityNodesLocation;

        private int unhighlightedNodesAlphaLocation;

        private int fogDensityEdgeLocation;

        private int unhighlightedEdgeAlphaLocation;

        private int quadVao;

        private int quadVbo;

        private int quadEbo;

        private int edgesVAO;

        private int edgesProgram;

        private int projEdgesLocation;

        private int dummyVao;
        

        public DisplayProgram()
        {
            nodesProgram = ShaderUtil.CompileAndLinkRenderShader("nodes.vert", "nodes.frag");

            projNodesLocation = GL.GetUniformLocation(nodesProgram, "projection");
            if (projNodesLocation == -1) throw new Exception("Uniform 'projection' not found. Shader optimized it out?");
            particleSizeLocation = GL.GetUniformLocation(nodesProgram, "paricleSize");
            if (particleSizeLocation == -1) throw new Exception("Uniform 'paricleSize' not found. Shader optimized it out?");
            viewportSizeNodesLocation = GL.GetUniformLocation(nodesProgram, "viewportSize");
            if (viewportSizeNodesLocation == -1) throw new Exception("Uniform 'viewportSize' not found. Shader optimized it out?");
            viewNodesLocation = GL.GetUniformLocation(nodesProgram, "view");
            if (viewNodesLocation == -1) throw new Exception("Uniform 'view' not found. Shader optimized it out?");
            fogDensityNodesLocation = GL.GetUniformLocation(nodesProgram, "fogDensity");
            if (fogDensityNodesLocation == -1) throw new Exception("Uniform 'fogDensity' not found. Shader optimized it out?");

            edgesProgram = ShaderUtil.CompileAndLinkRenderShader("edges.vert", "edges.frag");
            projEdgesLocation = GL.GetUniformLocation(edgesProgram, "projection");
            if (projEdgesLocation == -1) throw new Exception("Uniform 'projection' not found. Shader optimized it out?");
            viewportSizeEdgesLocation = GL.GetUniformLocation(edgesProgram, "viewportSize");
            if (viewportSizeEdgesLocation == -1) throw new Exception("Uniform 'viewportSize' not found. Shader optimized it out?");
            viewEdgesLocation = GL.GetUniformLocation(edgesProgram, "view");
            if (viewEdgesLocation == -1) throw new Exception("Uniform 'view' not found. Shader optimized it out?");
            lineWidthLocation = GL.GetUniformLocation(edgesProgram, "lineWidth");
            if (lineWidthLocation == -1) throw new Exception("Uniform 'lineWidth' not found. Shader optimized it out?");
            fogDensityEdgeLocation = GL.GetUniformLocation(edgesProgram, "fogDensity");
            if (fogDensityEdgeLocation == -1) throw new Exception("Uniform 'fogDensity' not found. Shader optimized it out?");
            unhighlightedEdgeAlphaLocation = GL.GetUniformLocation(edgesProgram, "unhighlightedAlpha");
            if (unhighlightedEdgeAlphaLocation == -1) throw new Exception("Uniform 'unhighlightedAlpha' not found. Shader optimized it out?");

            dummyVao = GL.GenVertexArray();

            float[] quad =
                {
                    -1, -1,
                     1, -1,
                     1,  1,
                    -1,  1
                };

            uint[] indices = { 0, 1, 2, 2, 3, 0 };

            quadVao = GL.GenVertexArray();
            quadVbo = GL.GenBuffer();
            quadEbo = GL.GenBuffer();
            edgesVAO = GL.GenVertexArray();

            GL.BindVertexArray(quadVao);

            GL.BindBuffer(BufferTarget.ArrayBuffer, quadVbo);
            GL.BufferData(BufferTarget.ArrayBuffer, quad.Length * sizeof(float), quad, BufferUsageHint.StaticDraw);

            GL.VertexAttribPointer(5, 2, VertexAttribPointerType.Float, false, 2 * sizeof(float), 0);
            GL.EnableVertexAttribArray(5);

            GL.BindBuffer(BufferTarget.ElementArrayBuffer, quadEbo);
            GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(uint), indices, BufferUsageHint.StaticDraw);

            GL.BindVertexArray(0);

            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Less);
            GL.DepthMask(true);

            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(
                BlendingFactor.SrcAlpha,
                BlendingFactor.OneMinusSrcAlpha
            );
        }

        public void Run(int nodesBuffer,
                        int edgesBuffer,
                        Matrix4 projectionMatrix,
                        Vector2 viewportSize,
                        Matrix4 view,
                        Simulation sim)
        {
            GL.Clear(
                ClearBufferMask.ColorBufferBit |
                ClearBufferMask.DepthBufferBit
            );
            
            //nodes as points
            GL.UseProgram(nodesProgram);
            GL.BindBufferBase(BufferRangeTarget.ShaderStorageBuffer, 2, nodesBuffer);
            GL.BindVertexArray(quadVao);
            GL.UniformMatrix4(projNodesLocation, false, ref projectionMatrix);
            GL.Uniform1(particleSizeLocation, sim.particleSize);
            GL.Uniform2(viewportSizeNodesLocation, viewportSize);
            GL.UniformMatrix4(viewNodesLocation, false, ref view);
            GL.Uniform1(fogDensityNodesLocation, sim.fogDensity);

            GL.DrawElementsInstanced(
                PrimitiveType.Triangles,
                6,
                DrawElementsType.UnsignedInt,
                IntPtr.Zero,
                sim.config.particleCount
            );

            // edges as quads (x6)
            /*
            GL.UseProgram(edgesProgram);
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            GL.Enable(EnableCap.Multisample);
            GL.Enable(EnableCap.FramebufferSrgb);
            GL.DepthFunc(DepthFunction.Lequal);

            GL.Disable(EnableCap.LineSmooth);
            GL.Disable(EnableCap.PolygonSmooth);
            GL.Disable(EnableCap.AlphaTest);
            GL.Enable(EnableCap.SampleAlphaToCoverage);

            GL.BindBufferBase(BufferRangeTarget.ShaderStorageBuffer, 2, nodesBuffer);
            GL.BindBufferBase(BufferRangeTarget.ShaderStorageBuffer, 3, edgesBuffer);
            var projection = view * projectionMatrix;
            GL.UniformMatrix4(projEdgesLocation, false, ref projection);
            GL.UniformMatrix4(viewEdgesLocation, false, ref view);
            GL.Uniform2(viewportSizeEdgesLocation, ref viewportSize);
            GL.Uniform1(lineWidthLocation, sim.lineWidth);
            GL.Uniform1(fogDensityEdgeLocation, sim.fogDensity);
            GL.DrawArrays(PrimitiveType.Triangles, 0, sim.edges.Length * 6);
            */

        }
    }
}
