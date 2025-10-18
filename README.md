# 🦾 URDF Builder GUI
**A visual, interactive tool for creating and editing URDF robot models with ease.**

---

## 🚀 Overview
URDF Builder GUI helps roboticists, students, and educators **build and preview URDF models without manually writing XML**.  
It provides a clean, modern interface to add links, joints, geometries, inertias, and collisions — with **real-time 3D visualization** and **live XML output**.

Built with **Python + Three.js**, this lightweight tool aims to make robot modeling more **intuitive and beginner-friendly**.

---

## ✨ Features
- 🧩 Create and edit **links, joints, collisions, inertias** interactively  
- 🎨 Add **geometries** (box, cylinder, sphere, mesh) with visual parameters  
- 🧮 Optional **manual inertia** entry or automatic identical assignment  
- 👁️ Real-time **3D viewer** to visualize geometry and transformations  
- ⚙️ **Instant XML generation** and export to `.urdf`  
- 🔄 Support for **ROS1 & ROS2**-compatible structure  

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
- 🔜 Automatic inertia estimation (mass + geometry)  

---

## 💡 Example Use Case
- Rapidly prototype a robot’s structure before simulation  
- Teach students URDF concepts interactively  
- Visually debug link/joint transformations  
- Generate URDFs compatible with **Gazebo**, **Rviz**, or **ROS2 Launch**

---

## 🧑‍💻 Getting Started
```bash
git clone https://github.com/YOUR_USERNAME/urdf-builder-gui.git
cd urdf-builder-gui
python -m http.server 8080
