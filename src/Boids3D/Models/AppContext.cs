using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Boids3D.Gpu;
using Boids3D.Gui;

namespace Boids3D.Models
{
    public class AppContext
    {
        public Simulation simulation;

        public MainWindow mainWindow;

        public OpenGlRenderer renderer;

        public ConfigWindow configWindow;
    }
}
