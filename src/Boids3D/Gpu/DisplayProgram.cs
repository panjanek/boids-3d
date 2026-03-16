using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK.GLControl;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using Boids3D.Utils;

namespace Boids3D.Gpu
{
    public class DisplayProgram
    {
        private int program;

        private int projLocation;

        private int particleSizeLocation;

        private int fogDensityLocation;

        private int viewLocation;

        private int cubeProgram;
        
        private int projCubeLocation;
        
        private int viewCubeLocation;

        private int fieldSizeCubeLocation;
        
        private int horizonProgram;
        
        private int projHorizonLocation;
        
        private int viewHorizonLocation;

        private int fieldSizeHorizonLocation;

        private int cameraPosHorizonLocation;

        private int dummyVao;

        private int cubeVao;
        
        private int cubeVbo;
        
        private int groundVao;
        
        private int groundVbo;
        
        private float[] cubeLines =
        {
            // bottom square
            0,0,0,  1,0,0,
            1,0,0,  1,1,0,
            1,1,0,  0,1,0,
            0,1,0,  0,0,0,

            // top square
            0,0,1,  1,0,1,
            1,0,1,  1,1,1,
            1,1,1,  0,1,1,
            0,1,1,  0,0,1,

            // vertical edges
            0,0,0,  0,0,1,
            1,0,0,  1,0,1,
            1,1,0,  1,1,1,
            0,1,0,  0,1,1
        };
        
        private float[] groundVertices =
        {
            -1, 0, -1,
            1, 0, -1,
            1, 0,  1,

            -1, 0, -1,
            1, 0,  1,
            -1, 0,  1
        };

        public DisplayProgram()
        {
            program = ShaderUtil.CompileAndLinkRenderShader("display.vert", "display.frag");
            projLocation = GL.GetUniformLocation(program, "projection");
            if (projLocation == -1) throw new Exception("Uniform 'projection' not found. Shader optimized it out?");
            particleSizeLocation = GL.GetUniformLocation(program, "particleSize");
            if (particleSizeLocation == -1) throw new Exception("Uniform 'particleSize' not found. Shader optimized it out?");
            fogDensityLocation = GL.GetUniformLocation(program, "fogDensity");
            if (fogDensityLocation == -1) throw new Exception("Uniform 'fogDensity' not found. Shader optimized it out?");
            viewLocation = GL.GetUniformLocation(program, "view");
            if (viewLocation == -1) throw new Exception("Uniform 'view' not found. Shader optimized it out?");
            
            cubeProgram = ShaderUtil.CompileAndLinkRenderShader("cube.vert", "cube.frag");
            viewCubeLocation = GL.GetUniformLocation(cubeProgram, "view");
            if (viewCubeLocation == -1) throw new Exception("Uniform 'view' not found. Shader optimized it out?");
            projCubeLocation = GL.GetUniformLocation(cubeProgram, "projection");
            if (projCubeLocation == -1) throw new Exception("Uniform 'projection' not found. Shader optimized it out?");
            fieldSizeCubeLocation = GL.GetUniformLocation(cubeProgram, "fieldSize");
            if (fieldSizeCubeLocation == -1) throw new Exception("Uniform 'fieldSize' not found. Shader optimized it out?");
            
            horizonProgram = ShaderUtil.CompileAndLinkRenderShader("horizon.vert", "horizon.frag");
            viewHorizonLocation = GL.GetUniformLocation(horizonProgram, "view");
            if (viewHorizonLocation == -1) throw new Exception("Uniform 'view' not found. Shader optimized it out?");
            projHorizonLocation = GL.GetUniformLocation(horizonProgram, "projection");
            if (projHorizonLocation == -1) throw new Exception("Uniform 'projection' not found. Shader optimized it out?");
            fieldSizeHorizonLocation = GL.GetUniformLocation(horizonProgram, "fieldSize");
            if (fieldSizeHorizonLocation == -1) throw new Exception("Uniform 'fieldSize' not found. Shader optimized it out?");
            cameraPosHorizonLocation = GL.GetUniformLocation(horizonProgram, "cameraPos");
            if (fieldSizeHorizonLocation == -1) throw new Exception("Uniform 'cameraPos' not found. Shader optimized it out?");
            
            //boids
            dummyVao = GL.GenVertexArray();
            GL.BindVertexArray(0);
            GL.Enable(EnableCap.Multisample);
            
            //cube
            cubeVao = GL.GenVertexArray();
            cubeVbo = GL.GenBuffer();
            GL.BindVertexArray(cubeVao);

            GL.BindBuffer(BufferTarget.ArrayBuffer, cubeVbo);
            GL.BufferData(
                BufferTarget.ArrayBuffer,
                cubeLines.Length * sizeof(float),
                cubeLines,
                BufferUsageHint.StaticDraw);

            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(
                0,
                3,
                VertexAttribPointerType.Float,
                false,
                3 * sizeof(float),
                0);

            GL.BindVertexArray(0);
            
            // horizon
            groundVao = GL.GenVertexArray();
            GL.BindVertexArray(groundVao);
            
            groundVbo = GL.GenBuffer();
            GL.BindBuffer(BufferTarget.ArrayBuffer, groundVbo);

            GL.BufferData(
                BufferTarget.ArrayBuffer,
                groundVertices.Length * sizeof(float),
                groundVertices,
                BufferUsageHint.StaticDraw);

            GL.EnableVertexAttribArray(0);

            GL.VertexAttribPointer(
                0,
                3,
                VertexAttribPointerType.Float,
                false,
                3 * sizeof(float),
                0);

            GL.BindVertexArray(0);
        }

        public void Run(Matrix4 projectionMatrix,
                       Matrix4 viewMatrix,
                       int particlesCount, 
                       float particleSize, 
                       float fogDensity,
                       float fieldSize,
                       bool cubeVisible,
                       Vector3 cameraPos,
                       bool horizonVisible)
        {
            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Lequal);
            GL.DepthMask(true);
            GL.Clear(
                ClearBufferMask.ColorBufferBit |
                ClearBufferMask.DepthBufferBit
            );
            
            if (horizonVisible)
                GL.ClearColor(0.15f, 0.25f, 0.45f, 1);
            else
                GL.ClearColor(0,0,0,1);

            GL.Disable(EnableCap.Blend);
            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.FrontFace(FrontFaceDirection.Ccw);

            GL.UseProgram(program);
            GL.BindVertexArray(dummyVao);

            GL.UniformMatrix4(projLocation, false, ref projectionMatrix);
            GL.UniformMatrix4(viewLocation, false, ref viewMatrix);
            GL.Uniform1(particleSizeLocation, particleSize);
            GL.Uniform1(fogDensityLocation, fogDensity);

            GL.DrawArraysInstanced(
                PrimitiveType.Triangles,
                0,
                12, // vertices per tetrahedron
                particlesCount // number of particles
            );

            if (cubeVisible)
                DrawCube(projectionMatrix, viewMatrix, fieldSize);

            if (horizonVisible)
                DrawHorizon(projectionMatrix, viewMatrix, fieldSize, cameraPos);
        }

        private void DrawHorizon(Matrix4 projectionMatrix, Matrix4 viewMatrix, float fieldSize, Vector3 cameraPos)
        {
            GL.UseProgram(horizonProgram);

            GL.UniformMatrix4(viewHorizonLocation, false, ref viewMatrix);
            GL.UniformMatrix4(projHorizonLocation, false, ref projectionMatrix);
            GL.Uniform1(fieldSizeHorizonLocation, fieldSize);
            GL.Uniform3(cameraPosHorizonLocation, cameraPos);

            GL.BindVertexArray(groundVao);

            GL.Disable(EnableCap.CullFace);
            GL.Disable(EnableCap.Blend);
            GL.DepthMask(true);

            GL.DrawArrays(
                PrimitiveType.Triangles,
                0,
                6);

            GL.BindVertexArray(0);
        }

        private void DrawCube(Matrix4 projectionMatrix, Matrix4 viewMatrix, float fieldSize)
        {
            GL.UseProgram(cubeProgram);

            GL.UniformMatrix4(viewCubeLocation, false, ref viewMatrix);
            GL.UniformMatrix4(projCubeLocation, false, ref projectionMatrix);
            GL.Uniform1(fieldSizeCubeLocation, fieldSize);

            GL.BindVertexArray(cubeVao);

            GL.Disable(EnableCap.CullFace);
            GL.Disable(EnableCap.Blend);
            GL.DepthMask(true);

            GL.DrawArrays(
                PrimitiveType.Lines,
                0,
                24);

            GL.BindVertexArray(0);
        }
    }
}
