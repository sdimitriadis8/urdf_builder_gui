# 🦾 URDF Builder GUI
**A visual, interactive tool for creating URDF robot models and exporting their .xml files with ease.**

---

## 🚀 Overview
URDF Builder GUI helps roboticists, students, and educators **build and preview URDF models without manually writing XML**.  
It provides a clean, modern interface to add links, joints, geometries, inertias, and collisions — with **real-time 3D visualization** and **live XML output**.

Built with **Python + Three.js**, this lightweight tool aims to make robot modeling more **intuitive and beginner-friendly**.

---

## ✨ Features
- 🧩 Create and edit **links and joints** through a user-friendly GUI  
- 🎨 Add various basic **geometries** (box, cylinder, sphere) with visual parameters  
- 🧮 Optional **collision** geometry with visual overlay
- 🧮 Optional **manual inertia** entry or automatic identical assignment  
- 👁️ Real-time **3D preview** to visualize geometry and transformations of your robot model 
- ⚙️ **Instant XML generation** and export to `.urdf`
- 🔄 Support for **ROS1 & ROS2**-compatible structure

---

## 📥 Installation
### Prerequisites
- Python 3.7 or higher
- pip (Python package manager)

### 🪟 Windows Installation

#### Method 1: Using Python directly

1. **Install Python**
    
    Download from 
   ```cmd
   https://python.org/downloads/
   ```
   During installation, CHECK "Add Python to PATH"
   
2. **Verify Python installation**

    Open Command Prompt and run:
   ```cmd
   python --version
   pip --version
   ```
3. **Install dependencies**
    ```cmd
    pip install PyQt5 PyOpenGL
    ```
4. **Download and run**

    Download urdf_builder_gui.py from this repo and run it from cmd with:
    ```cmd
    python urdf_builder_gui.py
    ```

#### Method 2: Using virtual environment (Recommended)

    # Create virtual environment
    python -m venv urdf_env
    urdf_env\Scripts\activate

    # Install packages
    pip install PyQt5 PyOpenGL

    # Run the application
    python urdf_builder_gui.py

---

### 🐧 Linux Installation

For Ubuntu/Debian:

    # Update package list
    sudo apt update

    # Install Python and pip (if not already installed)
    sudo apt install python3 python3-pip

    # Install system dependencies for PyQt5
    sudo apt install libxcb-xinerama0

    # Install Python packages
    pip3 install PyQt5 PyOpenGL

    # Download and run
    python3 urdf_builder_gui.py

For Fedora/RHEL:

    # Install Python and dependencies
    sudo dnf install python3 python3-pip

    # Install Python packages
    pip3 install PyQt5 PyOpenGL

    # Run the application
    python3 urdf_builder_gui.py

Using virtual environment (Recommended):

    # Create virtual environment
    python3 -m venv urdf_env
    source urdf_env/bin/activate

    # Install packages
    pip install PyQt5 PyOpenGL

    # Run the application
    python urdf_builder_gui.py

---

### 🍎 macOS Installation
#### Method 1: Using Homebrew

    # Install Homebrew (if not installed)
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

    # Install Python
    brew install python

    # Install packages
    pip3 install PyQt5 PyOpenGL

    # Run the application
    python3 urdf_builder_gui.py

#### Method 2: Using virtual environment

    # Create virtual environment
    python3 -m venv urdf_env
    source urdf_env/bin/activate

    # Install packages
    pip install PyQt5 PyOpenGL

    # Run the application
    python urdf_builder_gui.py

---

## Quick Start
### Download the Application
```bash
# Clone the repository
git clone https://github.com/sdimitriadis8/urdf_builder_gui.git
cd urdf_builder_gui

# Or download the Python file directly from GitHub
```

### Run the Application
#### Windows:
```cmd
python urdf_builder_gui.py
```
#### Linux/macOS:
```bash
python3 urdf_builder_gui.py
```

---

## Usage
### Basic Workflow

1. **Create Links**

    - Enter a link name in the "Add Link" section

    - Select geometry type (box, cylinder, sphere) and dimensions 

    - Set mass, origin and properties (inertia, collision)

    - Click "Add Link" to add to your xml

2. **Create Joints**

    - Enter a joint name in the "Add Joint" section

    - Select joint type (revolute, prismatic, fixed, continuous)

    - Choose parent and child links from dropdown

    - Configure origin, axis, and limits

    - Click "Add Joint" to add to your xml

3. **3D Visualization**

    - Rotate: Left-click and drag

    - Pan: Right-click and drag

    - Zoom: Mouse wheel

4. **Export URDF**

    - Use "Export URDF" button to save your model

    - Or copy URDF code from the preview panel


### Terminal Controls

- The application runs in a terminal window

- Close the terminal to exit the application

- Error messages will appear in the terminal if issues occur

---

## Troubleshooting
### Common Issues

**"Command not found" errors:**

- Windows: Ensure Python is in PATH during installation

- Linux/macOS: Use python3 instead of python

**Module not found errors:**
```bash

# Reinstall packages
pip install --upgrade PyQt5 PyOpenGL

# Or try with pip3
pip3 install PyQt5 PyOpenGL
```

**PyQt5 installation issues on Linux:**
```bash

# Ubuntu/Debian
sudo apt install python3-pyqt5

# Fedora
sudo dnf install python3-qt5
```

**3D view not rendering:**

- Update graphics drivers

- Some systems may require additional OpenGL packages

### Getting Help

If you encounter issues:

1. Check that Python and all dependencies are installed correctly*

2. Run from terminal to see detailed error messages

3. Create a GitHub issue with:

    - Your operating system and version
    - Python version (python --version or python3 --version)

    - Complete error message from terminal
4. Contact me through email: info@28x.gr
---

## File Structure
```text

urdf_builder_gui/
├── urdf_builder_gui.py    # Main application
├── README.md              # This file
└── LICENSE.txt            # License file
```

---

## Dependencies

- **PyQt5:** GUI framework

- **PyOpenGL:** 3D rendering

- **Standard Library:** xml.etree, math, sys

---

## 🧱 Tech Stack
| Component | Description |
|------------|-------------|
| **Frontend** | HTML + TailwindCSS + Three.js |
| **Backend** | Python (Flask / FastAPI planned) |
| **File Format** | URDF (Universal Robot Description Format) |
| **Visualization** | Three.js 3D Viewer |

---

## 🧰 Planned Features
- ✅ Full collision parameter editing  
- ✅ Toggle visual vs collision geometry  
- 🔜 Joint hierarchy viewer  
- 🔜 Save & load full project configuration  
- 🔜 Automatic inertia 
estimation (mass + geometry)  

---

## 💡 Example Use Case
- Rapidly prototype a robot’s structure before simulation. 
- Perfect for beginners to experiment changes fast before Rviz 
- Teach students URDF concepts interactively  
- Visually debug link/joint transformations  
- Generate URDFs compatible with **Gazebo**, **Rviz**, or **ROS2 Launch**

---

## Contributing

Contributions are welcome! Feel free to submit pull requests or open issues for bugs and feature requests.
