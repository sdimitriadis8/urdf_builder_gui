import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QComboBox, QPushButton,
    QVBoxLayout, QHBoxLayout, QFormLayout, QTextEdit, QFileDialog, QMessageBox,
    QGroupBox, QCheckBox, QListWidget, QListWidgetItem, QGridLayout, QSizePolicy
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QSurfaceFormat
from PyQt5.QtWidgets import QOpenGLWidget
from xml.etree import ElementTree as ET
from xml.dom import minidom

# OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from math import degrees, pi

# ----------------------- Data classes -----------------------
class Link:
    def __init__(self, name,
                 geom_type='box', size=(0.1,0.1,0.1),
                 mass=1.0, inertia=None,
                 manual_inertia=False,
                 origin=(0.0,0.0,0.0), rpy=(0.0,0.0,0.0),
                 include_collision=False,
                 collision_geom=None, collision_size=None):
        self.name = name
        self.geom_type = geom_type
        self.size = size
        self.mass = mass
        self.inertia = inertia  # dict str values or None
        self.manual_inertia = manual_inertia
        self.origin = origin
        self.rpy = rpy
        self.include_collision = include_collision
        # collision-specific stored if manual set; otherwise None meaning use visual
        self.collision_geom = collision_geom
        self.collision_size = collision_size


class Joint:
    def __init__(self, name, jtype, parent, child,
                 origin_xyz=(0.0,0.0,0.0), origin_rpy=(0.0,0.0,0.0),
                 axis=(0.0,0.0,1.0), limit=None, effort=10.0, velocity=1.0):
        self.name = name
        self.jtype = jtype
        self.parent = parent
        self.child = child
        self.origin_xyz = origin_xyz
        self.origin_rpy = origin_rpy
        self.axis = axis
        self.limit = limit
        self.effort = effort
        self.velocity = velocity


class URDFModel:
    def __init__(self):
        self.links = {}   # name -> Link
        self.joints = {}  # name -> Joint

    def to_urdf_string(self):
        robot = ET.Element('robot', {'name': 'generated_robot'})
        for link in self.links.values():
            l = ET.SubElement(robot, 'link', {'name': link.name})
            # Only include inertial if manual_inertia flag is True and inertia provided
            if link.manual_inertia and link.inertia:
                inertial = ET.SubElement(l, 'inertial')
                ET.SubElement(inertial, 'mass', {'value': str(link.mass)})
                ET.SubElement(inertial, 'inertia', {k: str(v) for k, v in link.inertia.items()})
            # Visual with origin (origin always included from link.origin)
            visual = ET.SubElement(l, 'visual')
            ET.SubElement(visual, 'origin', {'xyz': f"{link.origin[0]} {link.origin[1]} {link.origin[2]}", 'rpy': f"{link.rpy[0]} {link.rpy[1]} {link.rpy[2]}"})
            geom = ET.SubElement(visual, 'geometry')
            if link.geom_type == 'box':
                ET.SubElement(geom, 'box', {'size': f"{link.size[0]} {link.size[1]} {link.size[2]}"})
            elif link.geom_type == 'cylinder':
                ET.SubElement(geom, 'cylinder', {'radius': str(link.size[0]), 'length': str(link.size[2])})
            elif link.geom_type == 'sphere':
                ET.SubElement(geom, 'sphere', {'radius': str(link.size[0])})
            # Collision: if enabled, use manual collision properties if present else visual
            if link.include_collision:
                collision = ET.SubElement(l, 'collision')
                ET.SubElement(collision, 'origin', {'xyz': f"{link.origin[0]} {link.origin[1]} {link.origin[2]}", 'rpy': f"{link.rpy[0]} {link.rpy[1]} {link.rpy[2]}"})
                coll_geom = ET.SubElement(collision, 'geometry')
                use_geom = link.collision_geom if link.collision_geom else link.geom_type
                use_size = link.collision_size if link.collision_size else link.size
                if use_geom == 'box':
                    ET.SubElement(coll_geom, 'box', {'size': f"{use_size[0]} {use_size[1]} {use_size[2]}"})
                elif use_geom == 'cylinder':
                    ET.SubElement(coll_geom, 'cylinder', {'radius': str(use_size[0]), 'length': str(use_size[2])})
                elif use_geom == 'sphere':
                    ET.SubElement(coll_geom, 'sphere', {'radius': str(use_size[0])})
        for joint in self.joints.values():
            j = ET.SubElement(robot, 'joint', {'name': joint.name, 'type': joint.jtype})
            ET.SubElement(j, 'parent', {'link': joint.parent})
            ET.SubElement(j, 'child', {'link': joint.child})
            ET.SubElement(j, 'origin', {'xyz': f"{joint.origin_xyz[0]} {joint.origin_xyz[1]} {joint.origin_xyz[2]}", 'rpy': f"{joint.origin_rpy[0]} {joint.origin_rpy[1]} {joint.origin_rpy[2]}"})
            if joint.jtype != 'fixed':
                ET.SubElement(j, 'axis', {'xyz': f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}"})
            if joint.limit:
                ET.SubElement(j, 'limit', {'lower': str(joint.limit[0]), 'upper': str(joint.limit[1]), 'effort': str(joint.effort), 'velocity': str(joint.velocity)})
        rough = ET.tostring(robot, 'utf-8')
        reparsed = minidom.parseString(rough)
        return reparsed.toprettyxml(indent="  ")

    def load_from_urdf_string(self, urdf_text):
        try:
            root = ET.fromstring(urdf_text)
        except ET.ParseError:
            return False
        self.links.clear(); self.joints.clear()
        for l in root.findall('link'):
            name = l.get('name')
            mass_el = l.find('./inertial/mass')
            mass = float(mass_el.get('value')) if mass_el is not None and mass_el.get('value') else 1.0
            inertia_el = l.find('./inertial/inertia')
            inertia = None
            manual_inertia = False
            if inertia_el is not None:
                inertia = {}
                manual_inertia = True
                for key in ('ixx','ixy','ixz','iyy','iyz','izz'):
                    val = inertia_el.get(key)
                    if val is not None:
                        inertia[key] = val
            vis_origin = l.find('./visual/origin')
            if vis_origin is not None:
                xyz = tuple(map(float, vis_origin.get('xyz').split())) if vis_origin.get('xyz') else (0.0,0.0,0.0)
                rpy = tuple(map(float, vis_origin.get('rpy').split())) if vis_origin.get('rpy') else (0.0,0.0,0.0)
            else:
                xyz=(0.0,0.0,0.0); rpy=(0.0,0.0,0.0)
            geom_el = l.find('./visual/geometry')
            geom_type='box'; size=(0.1,0.1,0.1)
            if geom_el is not None:
                b = geom_el.find('box'); c = geom_el.find('cylinder'); s = geom_el.find('sphere')
                if b is not None and b.get('size'):
                    try: size = tuple(map(float, b.get('size').split())); geom_type='box'
                    except: pass
                elif c is not None and c.get('radius') and c.get('length'):
                    try: r=float(c.get('radius')); L=float(c.get('length')); size=(r*2,r*2,L); geom_type='cylinder'
                    except: pass
                elif s is not None and s.get('radius'):
                    try: r=float(s.get('radius')); size=(r*2,r*2,r*2); geom_type='sphere'
                    except: pass
            # check collision
            coll_el = l.find('collision')
            include_collision = coll_el is not None
            collision_geom = None; collision_size = None
            if coll_el is not None:
                cg = coll_el.find('geometry')
                if cg is not None:
                    b = cg.find('box'); c = cg.find('cylinder'); s = cg.find('sphere')
                    if b is not None and b.get('size'):
                        try: collision_size = tuple(map(float, b.get('size').split())); collision_geom='box'
                        except: pass
                    elif c is not None and c.get('radius') and c.get('length'):
                        try: r=float(c.get('radius')); L=float(c.get('length')); collision_size=(r*2,r*2,L); collision_geom='cylinder'
                        except: pass
                    elif s is not None and s.get('radius'):
                        try: r=float(s.get('radius')); collision_size=(r*2,r*2,r*2); collision_geom='sphere'
                        except: pass
            self.links[name] = Link(name, geom_type, size, mass, inertia, manual_inertia, origin=xyz, rpy=rpy, include_collision=include_collision, collision_geom=collision_geom, collision_size=collision_size)

        for j in root.findall('joint'):
            name = j.get('name'); jtype = j.get('type','fixed')
            parent = j.find('parent').get('link') if j.find('parent') is not None else ""
            child = j.find('child').get('link') if j.find('child') is not None else ""
            origin_el = j.find('origin')
            if origin_el is not None:
                xyz = tuple(map(float, origin_el.get('xyz').split())) if origin_el.get('xyz') else (0.0,0.0,0.0)
                rpy = tuple(map(float, origin_el.get('rpy').split())) if origin_el.get('rpy') else (0.0,0.0,0.0)
            else:
                xyz=(0.0,0.0,0.0); rpy=(0.0,0.0,0.0)
            axis_el = j.find('axis'); axis=(0.0,0.0,1.0)
            if axis_el is not None and axis_el.get('xyz'):
                axis = tuple(map(float, axis_el.get('xyz').split()))
            limit_el = j.find('limit'); limit=None; effort=10.0; velocity=1.0
            if limit_el is not None:
                if limit_el.get('lower') and limit_el.get('upper'):
                    try: limit=(float(limit_el.get('lower')), float(limit_el.get('upper')))
                    except: limit=None
                if limit_el.get('effort'): effort=float(limit_el.get('effort'))
                if limit_el.get('velocity'): velocity=float(limit_el.get('velocity'))
            joint = Joint(name, jtype, parent, child, origin_xyz=xyz, origin_rpy=rpy, axis=axis, limit=limit, effort=effort, velocity=velocity)
            self.joints[name] = joint
        return True

# ----------------------- GL Viewer (QOpenGLWidget) -----------------------
class GLWidget(QOpenGLWidget):
    def __init__(self, model):
        fmt = QSurfaceFormat()
        fmt.setDepthBufferSize(24)
        QSurfaceFormat.setDefaultFormat(fmt)
        super().__init__()
        self.model = model
        self.rot_x = -30.0
        self.rot_y = 30.0
        self.zoom = -6.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.last_pos = None

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, [5,5,10,1])
        glClearColor(0.95,0.95,0.95,1)

    def resizeGL(self, w, h):
        glViewport(0,0,w,h)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(45.0, w/h if h else 1.0, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        # camera transform: pan -> zoom -> rotate (orbit-like)
        glTranslatef(self.pan_x, self.pan_y, self.zoom)
        glRotatef(self.rot_x, 1.0, 0.0, 0.0)
        glRotatef(self.rot_y, 0.0, 1.0, 0.0)

        # simple grid ground
        glDisable(GL_LIGHTING)
        glColor3f(0.85,0.85,0.85)
        glBegin(GL_LINES)
        for i in range(-6,7):
            glVertex3f(i, -6, 0); glVertex3f(i, 6, 0)
            glVertex3f(-6, i, 0); glVertex3f(6, i, 0)
        glEnd()
        glEnable(GL_LIGHTING)

        # draw each link at its origin; no added jitter
        for link in self.model.links.values():
            # Visual
            glPushMatrix()
            tx, ty, tz = link.origin
            glTranslatef(tx, ty, tz)
            # apply link orientation (rpy in radians -> convert to deg)
            rx, rp, ry = link.rpy
            glRotatef(ry * 180.0 / pi, 0,0,1)
            glRotatef(rp * 180.0 / pi, 0,1,0)
            glRotatef(rx * 180.0 / pi, 1,0,0)

            # draw local frame axes
            self._draw_frame(scale=0.25)

            # draw geometry (visual)
            glColor3f(0.35, 0.65, 0.9)
            sx,sy,sz = link.size
            if link.geom_type == 'box':
                glPushMatrix()
                glScalef(sx, sy, sz)
                self._draw_unit_cube()
                glPopMatrix()
            elif link.geom_type == 'cylinder':
                glPushMatrix()
                glTranslatef(0, 0, -sz/2.0)
                quad = gluNewQuadric()
                gluCylinder(quad, sx, sx, sz, 32, 4)
                # end caps
                glPushMatrix(); glTranslatef(0,0,0); gluDisk(quad, 0, sx, 32, 1); glPopMatrix()
                glPushMatrix(); glTranslatef(0,0,sz); gluDisk(quad, 0, sx, 32, 1); glPopMatrix()
                glPopMatrix()
            elif link.geom_type == 'sphere':
                glPushMatrix()
                quad = gluNewQuadric()
                gluSphere(quad, sx, 20, 20)
                glPopMatrix()
            glPopMatrix()

            # Collision (if enabled) drawn as translucent overlay
            if link.include_collision:
                # choose geometry and size: manual collision overrides visual
                cgeom = link.collision_geom if link.collision_geom else link.geom_type
                csize = link.collision_size if link.collision_size else link.size
                glPushMatrix()
                tx, ty, tz = link.origin
                glTranslatef(tx, ty, tz)
                rx, rp, ry = link.rpy
                glRotatef(ry * 180.0 / pi, 0,0,1)
                glRotatef(rp * 180.0 / pi, 0,1,0)
                glRotatef(rx * 180.0 / pi, 1,0,0)

                # translucent red-ish overlay
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
                glDisable(GL_LIGHTING)
                glColor4f(1.0, 0.3, 0.3, 0.28)

                sx,sy,sz = csize
                if cgeom == 'box':
                    glPushMatrix()
                    glScalef(sx, sy, sz)
                    self._draw_unit_cube()
                    glPopMatrix()
                elif cgeom == 'cylinder':
                    glPushMatrix()
                    glTranslatef(0, 0, -sz/2.0)
                    quad = gluNewQuadric()
                    gluCylinder(quad, sx, sx, sz, 24, 4)
                    glPopMatrix()
                elif cgeom == 'sphere':
                    glPushMatrix()
                    quad = gluNewQuadric()
                    gluSphere(quad, sx, 20, 20)
                    glPopMatrix()

                glEnable(GL_LIGHTING)
                glDisable(GL_BLEND)
                glPopMatrix()

    def _draw_frame(self, scale=0.2):
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(scale,0,0)
        glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,scale,0)
        glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,scale)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_unit_cube(self):
        glBegin(GL_QUADS)
        # front
        glNormal3f(0,0,1)
        glVertex3f(-0.5,-0.5,0.5); glVertex3f(0.5,-0.5,0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(-0.5,0.5,0.5)
        # back
        glNormal3f(0,0,-1)
        glVertex3f(-0.5,-0.5,-0.5); glVertex3f(-0.5,0.5,-0.5); glVertex3f(0.5,0.5,-0.5); glVertex3f(0.5,-0.5,-0.5)
        # left
        glNormal3f(-1,0,0)
        glVertex3f(-0.5,-0.5,-0.5); glVertex3f(-0.5,-0.5,0.5); glVertex3f(-0.5,0.5,0.5); glVertex3f(-0.5,0.5,-0.5)
        # right
        glNormal3f(1,0,0)
        glVertex3f(0.5,-0.5,-0.5); glVertex3f(0.5,0.5,-0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,-0.5,0.5)
        # top
        glNormal3f(0,1,0)
        glVertex3f(-0.5,0.5,-0.5); glVertex3f(-0.5,0.5,0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,0.5,-0.5)
        # bottom
        glNormal3f(0,-1,0)
        glVertex3f(-0.5,-0.5,-0.5); glVertex3f(0.5,-0.5,-0.5); glVertex3f(0.5,-0.5,0.5); glVertex3f(-0.5,-0.5,0.5)
        glEnd()

    def mousePressEvent(self, ev):
        self.last_pos = ev.pos()

    def mouseMoveEvent(self, ev):
        if not self.last_pos:
            self.last_pos = ev.pos(); return
        dx = ev.x() - self.last_pos.x(); dy = ev.y() - self.last_pos.y()
        buttons = ev.buttons()
        if buttons & Qt.LeftButton:
            self.rot_x += dy * 0.5
            self.rot_y += dx * 0.5
        elif buttons & Qt.RightButton:
            self.pan_x += dx * 0.01; self.pan_y -= dy * 0.01
        self.last_pos = ev.pos()
        self.update()

    def wheelEvent(self, ev):
        self.zoom += ev.angleDelta().y() / 120.0 * 0.3
        self.update()

# ----------------------- Main UI -----------------------
class URDFBuilderUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("URDF Builder (origin/inertia/collision optional)")
        self.model = URDFModel()
        self._build_ui()
        self.update_preview_and_view()

    def _build_ui(self):
        main = QVBoxLayout(self)
        main.setSpacing(10)

        # TOP ROW: Add Link + Elements | Add Joint
        top_row = QHBoxLayout()
        top_row.setSpacing(10)

        # LEFT: Add Link + Elements
        left_column = QVBoxLayout()
        left_column.setSpacing(8)

        # Add Link group
        left_column.addWidget(QLabel("<b>Add Link</b>"))
        link_g = QGroupBox()
        lf = QFormLayout()
        lf.setLabelAlignment(Qt.AlignRight)
        lf.setVerticalSpacing(8)  # Reduced vertical spacing
        self.link_name = QLineEdit()
        self.link_name.setMaximumWidth(180)
        self.geom_combo = QComboBox()
        self.geom_combo.addItems(['box','cylinder','sphere'])
        self.geom_combo.setMaximumWidth(120)
        # react to geometry changes to update size labels
        self.geom_combo.currentTextChanged.connect(self._update_size_fields)

        size_h = QHBoxLayout()
        size_h.setSpacing(6)
        # size widgets & labels (we'll adapt labels/visibility depending on geometry)
        self.size_x_label = QLabel("x:")
        self.size_x = QLineEdit("0.1")
        self.size_y_label = QLabel("y:")
        self.size_y = QLineEdit("0.1")
        self.size_z_label = QLabel("z:")
        self.size_z = QLineEdit("0.1")
        for b in (self.size_x,self.size_y,self.size_z): b.setMaximumWidth(60)
        size_h.addWidget(self.size_x_label); size_h.addWidget(self.size_x)
        size_h.addWidget(self.size_y_label); size_h.addWidget(self.size_y)
        size_h.addWidget(self.size_z_label); size_h.addWidget(self.size_z)

        # mass + manual inertia checkbox (mass always visible)
        mass_h = QHBoxLayout()
        mass_h.setSpacing(8)
        mass_label = QLabel("")
        mass_label.setFixedWidth(36)
        self.mass_input = QLineEdit("1.0")
        self.mass_input.setMaximumWidth(80)
        mass_h.addWidget(mass_label); mass_h.addWidget(self.mass_input)
        # Manual Inertia checkbox placed BEFORE inertia fields per request
        self.manual_inertia_cb = QCheckBox("Inertia")
        self.manual_inertia_cb.toggled.connect(self._toggle_inertia)
        mass_h.addWidget(self.manual_inertia_cb)

        # inertia matrix (hidden by default) -> placed AFTER the checkbox (below it)
        self.inertia_grid = QGridLayout()
        self.inertia_grid.setSpacing(4)
        self.inertia_fields = {}
        inertia_keys = [("ixx","ixy","ixz"),("ixy","iyy","iyz"),("ixz","iyz","izz")]
        defaults = {"ixx":"0.001","iyy":"0.001","izz":"0.001","ixy":"0","ixz":"0","iyz":"0"}
        # create label+field widgets but keep them hidden until checkbox toggled
        for r in range(3):
            for c in range(3):
                key = inertia_keys[r][c]
                if (r,c) in ((0,1),(1,0)): key = "ixy"
                if (r,c) in ((0,2),(2,0)): key = "ixz"
                if (r,c) in ((1,2),(2,1)): key = "iyz"
                if key in self.inertia_fields:
                    fld = self.inertia_fields[key]
                else:
                    fld = QLineEdit(defaults.get(key,"0"))
                    fld.setMaximumWidth(70)
                    fld.setVisible(False)
                    self.inertia_fields[key] = fld
                lbl = QLabel(key + ":")
                lbl.setVisible(False)
                self.inertia_grid.addWidget(lbl, r, c*2)
                self.inertia_grid.addWidget(fld, r, c*2+1)

        # Manual origin checkbox + origin fields (hidden by default)
        self.manual_origin_cb = QCheckBox("Origin")
        self.manual_origin_cb.toggled.connect(self._toggle_origin)
        # we will hide both the labels "Position (xyz)" and "Orientation (rpy)" until checkbox checked
        self.origin_label_xyz = QLabel("Position (xyz)")
        self.origin_label_xyz.setVisible(False)
        self.origin_label_rpy = QLabel("Orientation (rpy)")
        self.origin_label_rpy.setVisible(False)
        origin_h = QHBoxLayout()
        origin_h.setSpacing(6)
        self.origin_x = QLineEdit("0")
        self.origin_y = QLineEdit("0")
        self.origin_z = QLineEdit("0")
        for b in (self.origin_x,self.origin_y,self.origin_z): b.setMaximumWidth(70); b.setVisible(False)
        origin_h.addWidget(QLabel("x:")); origin_h.itemAt(origin_h.count()-1).widget().setVisible(False)
        origin_h.addWidget(self.origin_x)
        origin_h.addWidget(QLabel("y:")); origin_h.itemAt(origin_h.count()-1).widget().setVisible(False)
        origin_h.addWidget(self.origin_y)
        origin_h.addWidget(QLabel("z:")); origin_h.itemAt(origin_h.count()-1).widget().setVisible(False)
        origin_h.addWidget(self.origin_z)

        rpy_h = QHBoxLayout()
        rpy_h.setSpacing(6)
        self.origin_r = QLineEdit("0")
        self.origin_p = QLineEdit("0")
        self.origin_yaw = QLineEdit("0")
        for b in (self.origin_r,self.origin_p,self.origin_yaw): b.setMaximumWidth(70); b.setVisible(False)
        rpy_h.addWidget(QLabel("roll:")); rpy_h.itemAt(rpy_h.count()-1).widget().setVisible(False)
        rpy_h.addWidget(self.origin_r)
        rpy_h.addWidget(QLabel("pitch:")); rpy_h.itemAt(rpy_h.count()-1).widget().setVisible(False)
        rpy_h.addWidget(self.origin_p)
        rpy_h.addWidget(QLabel("yaw:")); rpy_h.itemAt(rpy_h.count()-1).widget().setVisible(False)
        rpy_h.addWidget(self.origin_yaw)

        # collision checkbox (option A: collision copies visual geometry by default)
        self.collision_cb = QCheckBox("Enable Collision")
        # collision panel that shows when checked
        self.collision_panel_widget = QWidget()
        self.collision_panel_layout = QFormLayout(self.collision_panel_widget)
        self.collision_panel_widget.setVisible(False)
        self.collision_mode = QComboBox()
        self.collision_mode.addItems(["Use identical","Manual set"])
        self.collision_mode.currentTextChanged.connect(self._on_collision_mode_changed)
        # manual collision geometry widgets (hidden unless Manual set)
        self.coll_geom_combo = QComboBox()
        self.coll_geom_combo.addItems(['box','cylinder','sphere'])
        self.coll_size_x = QLineEdit("0.1")
        self.coll_size_y = QLineEdit("0.1")
        self.coll_size_z = QLineEdit("0.1")
        for w in (self.coll_size_x, self.coll_size_y, self.coll_size_z): w.setMaximumWidth(70)
        coll_size_h = QHBoxLayout()
        self.coll_size_x_label = QLabel("x:")
        self.coll_size_y_label = QLabel("y:")
        self.coll_size_z_label = QLabel("z:")
        coll_size_h.addWidget(self.coll_size_x_label); coll_size_h.addWidget(self.coll_size_x)
        coll_size_h.addWidget(self.coll_size_y_label); coll_size_h.addWidget(self.coll_size_y)
        coll_size_h.addWidget(self.coll_size_z_label); coll_size_h.addWidget(self.coll_size_z)
        # hide manual collision group initially
        self.coll_geom_combo.setVisible(False); self.coll_size_x.setVisible(False); self.coll_size_y.setVisible(False); self.coll_size_z.setVisible(False)
        self.collision_panel_layout.addRow("Mode", self.collision_mode)
        self.collision_panel_layout.addRow("Collision geometry", self.coll_geom_combo)
        self.collision_panel_layout.addRow("Collision size", coll_size_h)
        # toggle panel visible when checkbox toggled
        self.collision_cb.toggled.connect(lambda v: (self.collision_panel_widget.setVisible(v), self.update_preview_and_view()))
        # also hide manual fields initially
        self.coll_geom_combo.currentTextChanged.connect(self._update_collision_size_fields)

        add_link_btn = QPushButton("Add Link")
        add_link_btn.clicked.connect(self._on_add_link)

        lf.addRow("Name", self.link_name)
        lf.addRow("Geometry", self.geom_combo)
        lf.addRow("Size", size_h)
        lf.addRow("Mass", mass_h)
        # inertia fields placed below the checkbox
        lf.addRow(self.manual_inertia_cb)
        lf.addRow(self.inertia_grid)
        # origin checkbox + labels+fields hidden until checked
        lf.addRow(self.manual_origin_cb)
        lf.addRow(self.origin_label_xyz, origin_h)
        lf.addRow(self.origin_label_rpy, rpy_h)
        # collision panel
        lf.addRow(self.collision_cb)
        lf.addRow(self.collision_panel_widget)
        lf.addRow(add_link_btn)
        link_g.setLayout(lf)
        left_column.addWidget(link_g)

        # elements list
        left_column.addWidget(QLabel("<b>Elements</b>"))
        self.elements_list = QListWidget()
        self.elements_list.setMaximumHeight(160)
        self.elements_list.itemDoubleClicked.connect(self._load_selected_element)
        left_column.addWidget(self.elements_list)
        el_btns = QHBoxLayout()
        self.delete_btn = QPushButton("Delete Selected")
        self.delete_btn.clicked.connect(self._delete_selected)
        self.edit_btn = QPushButton("Edit Selected")
        self.edit_btn.clicked.connect(self._load_selected_element)
        el_btns.addWidget(self.edit_btn); el_btns.addWidget(self.delete_btn)
        left_column.addLayout(el_btns)

        # RIGHT: Add Joint
        right_column = QVBoxLayout()
        right_column.setSpacing(8)
        
        # Add Joint section
        joint_label = QLabel("<b>Add Joint</b>")
        right_column.addWidget(joint_label)
        joint_g = QGroupBox()
        jf = QFormLayout()
        jf.setLabelAlignment(Qt.AlignRight)
        jf.setVerticalSpacing(8)  # Reduced vertical spacing
        jf.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)  # Better field growth policy
        
        self.joint_name = QLineEdit()
        self.joint_name.setMaximumWidth(180)
        self.joint_type = QComboBox()
        self.joint_type.addItems(['revolute','continuous','prismatic','fixed'])
        self.joint_type.setMaximumWidth(120)
        self.parent_combo = QComboBox()
        self.child_combo = QComboBox()
        self.parent_combo.setMaximumWidth(160)
        self.child_combo.setMaximumWidth(160)
        
        origin_h2 = QHBoxLayout()
        origin_h2.setSpacing(6)
        self.j_origin_x = QLineEdit("0")
        self.j_origin_y = QLineEdit("0")
        self.j_origin_z = QLineEdit("0")
        for b in (self.j_origin_x,self.j_origin_y,self.j_origin_z): b.setMaximumWidth(70)
        origin_h2.addWidget(QLabel("x:")); origin_h2.addWidget(self.j_origin_x)
        origin_h2.addWidget(QLabel("y:")); origin_h2.addWidget(self.j_origin_y)
        origin_h2.addWidget(QLabel("z:")); origin_h2.addWidget(self.j_origin_z)
        
        rpy_h2 = QHBoxLayout()
        rpy_h2.setSpacing(6)
        self.j_origin_r = QLineEdit("0")
        self.j_origin_p = QLineEdit("0")
        self.j_origin_yaw = QLineEdit("0")
        for b in (self.j_origin_r,self.j_origin_p,self.j_origin_yaw): b.setMaximumWidth(70)
        rpy_h2.addWidget(QLabel("roll:")); rpy_h2.addWidget(self.j_origin_r)
        rpy_h2.addWidget(QLabel("pitch:")); rpy_h2.addWidget(self.j_origin_p)
        rpy_h2.addWidget(QLabel("yaw:")); rpy_h2.addWidget(self.j_origin_yaw)
        
        axis_h = QHBoxLayout()
        axis_h.setSpacing(6)
        self.axis_x = QLineEdit("0")
        self.axis_y = QLineEdit("0")
        self.axis_z = QLineEdit("1")
        for b in (self.axis_x,self.axis_y,self.axis_z): b.setMaximumWidth(70)
        axis_h.addWidget(QLabel("x:")); axis_h.addWidget(self.axis_x)
        axis_h.addWidget(QLabel("y:")); axis_h.addWidget(self.axis_y)
        axis_h.addWidget(QLabel("z:")); axis_h.addWidget(self.axis_z)
        
        limits_h = QHBoxLayout()
        limits_h.setSpacing(6)
        self.limit_l = QLineEdit("-1.57")
        self.limit_u = QLineEdit("1.57")
        self.effort = QLineEdit("10")
        self.velocity = QLineEdit("1.0")
        for b in (self.limit_l,self.limit_u,self.effort,self.velocity): b.setMaximumWidth(70)
        limits_h.addWidget(QLabel("lower:")); limits_h.addWidget(self.limit_l)
        limits_h.addWidget(QLabel("upper:")); limits_h.addWidget(self.limit_u)
        limits_h.addWidget(QLabel("effort:")); limits_h.addWidget(self.effort)
        limits_h.addWidget(QLabel("velocity:")); limits_h.addWidget(self.velocity)

        add_joint_btn = QPushButton("Add Joint")
        add_joint_btn.clicked.connect(self._on_add_joint)
        self.joint_type.currentTextChanged.connect(self._joint_type_changed)

        jf.addRow("Name", self.joint_name)
        jf.addRow("Type", self.joint_type)
        jf.addRow("Parent link", self.parent_combo)
        jf.addRow("Child link", self.child_combo)
        jf.addRow("Position", origin_h2)
        jf.addRow("Orientation", rpy_h2)
        jf.addRow("Axis", axis_h)
        jf.addRow("Limits", limits_h)
        jf.addRow(add_joint_btn)
        joint_g.setLayout(jf)
        right_column.addWidget(joint_g)

        # Add stretch to push buttons to bottom
        right_column.addStretch(1)

        # Add buttons at bottom of right column
        btn_h = QHBoxLayout()
        self.apply_btn = QPushButton("Apply Edited URDF → Model")
        self.apply_btn.clicked.connect(self._apply_edited_urdf)
        self.export_btn = QPushButton("Export URDF")
        self.export_btn.clicked.connect(self._export_urdf)
        btn_h.addWidget(self.apply_btn)
        btn_h.addWidget(self.export_btn)
        right_column.addLayout(btn_h)

        # Add left and right columns to top row
        top_row.addLayout(left_column, 1)
        top_row.addLayout(right_column, 1)

        # BOTTOM ROW: URDF Preview | 3D Preview
        bottom_row = QHBoxLayout()
        bottom_row.setSpacing(10)

        # LEFT: URDF Preview
        urdf_preview_column = QVBoxLayout()
        urdf_preview_column.setSpacing(8)
        urdf_preview_column.addWidget(QLabel("<b>URDF Preview (editable)</b>"))
        self.urdf_text = QTextEdit()
        self.urdf_text.setMinimumHeight(160)
        urdf_preview_column.addWidget(self.urdf_text)

        # RIGHT: 3D Preview
        gl_preview_column = QVBoxLayout()
        gl_preview_column.setSpacing(8)
        gl_preview_column.addWidget(QLabel("<b>3D Preview (interactive)</b>"))
        self.gl = GLWidget(self.model)
        self.gl.setMinimumHeight(360)
        gl_preview_column.addWidget(self.gl)

        bottom_row.addLayout(urdf_preview_column, 1)
        bottom_row.addLayout(gl_preview_column, 1)

        # Add all to main layout
        main.addLayout(top_row, 1)
        main.addLayout(bottom_row, 1)

        self.setLayout(main)
        self.resize(1300, 820)

        # initial UI adjustments
        self._update_size_fields(self.geom_combo.currentText())
        self._update_collision_size_fields(self.coll_geom_combo.currentText())
        self.collision_panel_widget.setVisible(False)

    # ---------- UI helpers & actions ----------
    def _toggle_inertia(self, checked):
        # show/hide all inertia grid widgets (labels+fields)
        for i in range(self.inertia_grid.count()):
            item = self.inertia_grid.itemAt(i)
            if item and item.widget():
                item.widget().setVisible(checked)

    def _toggle_origin(self, checked):
        # toggle the label rows and the fields
        self.origin_label_xyz.setVisible(checked)
        self.origin_label_rpy.setVisible(checked)
        # show/hide the inputs inside the origin/rpy layouts
        for w in (self.origin_x, self.origin_y, self.origin_z, self.origin_r, self.origin_p, self.origin_yaw):
            w.setVisible(checked)
        # also hide the small inline label widgets
        # The QFormLayout contains separate label rows; we handled showing label widgets explicitly.
        self.update()

    def _refresh_elements_list(self):
        self.elements_list.clear()
        for name in self.model.links:
            it = QListWidgetItem(f"Link: {name}"); it.setData(Qt.UserRole, ('link', name)); self.elements_list.addItem(it)
        for name in self.model.joints:
            it = QListWidgetItem(f"Joint: {name}"); it.setData(Qt.UserRole, ('joint', name)); self.elements_list.addItem(it)

    def _refresh_link_combos(self):
        cur_p = self.parent_combo.currentText(); cur_c = self.child_combo.currentText()
        self.parent_combo.clear(); self.child_combo.clear()
        names = list(self.model.links.keys())
        if not names:
            self.parent_combo.addItem(""); self.child_combo.addItem("")
            return
        for n in names:
            self.parent_combo.addItem(n); self.child_combo.addItem(n)
        if cur_p in names: self.parent_combo.setCurrentText(cur_p)
        if cur_c in names: self.child_combo.setCurrentText(cur_c)

    def _update_size_fields(self, geom):
        """Adapt the size input labels & visibility based on geometry selection (visual)."""
        if geom == 'box':
            # x,y,z visible
            self.size_x_label.setText("x:"); self.size_y_label.setText("y:"); self.size_z_label.setText("z:")
            for w in (self.size_x_label, self.size_x, self.size_y_label, self.size_y, self.size_z_label, self.size_z):
                w.setVisible(True)
        elif geom == 'cylinder':
            # radius, length (we map size_x -> radius, size_z -> length; hide size_y)
            self.size_x_label.setText("radius:"); self.size_z_label.setText("length:")
            self.size_y_label.setVisible(False); self.size_y.setVisible(False)
            self.size_x_label.setVisible(True); self.size_x.setVisible(True)
            self.size_z_label.setVisible(True); self.size_z.setVisible(True)
        elif geom == 'sphere':
            # radius only (map to size_x)
            self.size_x_label.setText("radius:"); self.size_x_label.setVisible(True); self.size_x.setVisible(True)
            self.size_y_label.setVisible(False); self.size_y.setVisible(False)
            self.size_z_label.setVisible(False); self.size_z.setVisible(False)

    def _update_collision_size_fields(self, geom):
        """Adapt the collision manual size input labels & visibility."""
        if geom == 'box':
            self.coll_size_x_label.setText("x:"); self.coll_size_y_label.setText("y:"); self.coll_size_z_label.setText("z:")
            for w in (self.coll_size_x_label, self.coll_size_x, self.coll_size_y_label, self.coll_size_y, self.coll_size_z_label, self.coll_size_z):
                w.setVisible(True)
            self.coll_geom_combo.setVisible(True)
        elif geom == 'cylinder':
            self.coll_size_x_label.setText("radius:"); self.coll_size_z_label.setText("length:")
            self.coll_size_y_label.setVisible(False); self.coll_size_y.setVisible(False)
            self.coll_size_x_label.setVisible(True); self.coll_size_x.setVisible(True)
            self.coll_size_z_label.setVisible(True); self.coll_size_z.setVisible(True)
            self.coll_geom_combo.setVisible(True)
        elif geom == 'sphere':
            self.coll_size_x_label.setText("radius:"); self.coll_size_x_label.setVisible(True); self.coll_size_x.setVisible(True)
            self.coll_size_y_label.setVisible(False); self.coll_size_y.setVisible(False)
            self.coll_size_z_label.setVisible(False); self.coll_size_z.setVisible(False)
            self.coll_geom_combo.setVisible(True)

    def _on_collision_mode_changed(self, text):
        is_manual = (text == "Manual set")
        # show or hide manual collision widgets
        self.coll_geom_combo.setVisible(is_manual)
        self.coll_size_x.setVisible(is_manual)
        # update the labels & other fields according to selected collision geom
        self._update_collision_size_fields(self.coll_geom_combo.currentText())
        self.update_preview_and_view()

    def _on_add_link(self):
        name = self.link_name.text().strip()
        if not name:
            QMessageBox.warning(self,"Error","Link name required"); return
        # parse size depending on visual geometry choice
        geom = self.geom_combo.currentText()
        try:
            if geom == 'box':
                sx = float(self.size_x.text()); sy = float(self.size_y.text()); sz = float(self.size_z.text())
            elif geom == 'cylinder':
                sx = float(self.size_x.text());  # radius stored in slot x
                sy = sx  # store radius twice as in model.size (radius*2 logic used for GL/URDF)
                sz = float(self.size_z.text())  # length
            elif geom == 'sphere':
                sx = float(self.size_x.text()); sy = sx; sz = sx
        except ValueError:
            QMessageBox.warning(self,"Error","Size entries must be numeric"); return
        try:
            mass = float(self.mass_input.text())
        except Exception:
            mass = 1.0

        # inertia only if manual inertia checked
        inertia = None
        manual_inertia_flag = self.manual_inertia_cb.isChecked()
        if manual_inertia_flag:
            inertia = {}
            try:
                # collect unique inertia keys from fields
                for key, fld in self.inertia_fields.items():
                    inertia[key] = float(fld.text())
            except Exception:
                QMessageBox.warning(self,"Error","Inertia entries must be numeric"); return

        # origin
        origin = (0.0,0.0,0.0); rpy = (0.0,0.0,0.0)
        if self.manual_origin_cb.isChecked():
            try:
                origin = (float(self.origin_x.text()), float(self.origin_y.text()), float(self.origin_z.text()))
                rpy = (float(self.origin_r.text()), float(self.origin_p.text()), float(self.origin_yaw.text()))
            except Exception:
                QMessageBox.warning(self,"Error","Origin/RPY must be numeric"); return

        # collision
        include_collision = self.collision_cb.isChecked()
        collision_geom = None; collision_size = None
        if include_collision:
            if self.collision_mode.currentText() == "Use identical":
                collision_geom = None
                collision_size = None
            else:
                # manual collision: read geom + sizes
                try:
                    cgeom = self.coll_geom_combo.currentText()
                    if cgeom == 'box':
                        cx = float(self.coll_size_x.text()); cy = float(self.coll_size_y.text()); cz = float(self.coll_size_z.text())
                        collision_geom = 'box'; collision_size = (cx,cy,cz)
                    elif cgeom == 'cylinder':
                        cr = float(self.coll_size_x.text()); cl = float(self.coll_size_z.text())
                        collision_geom = 'cylinder'; collision_size = (cr, cr, cl)
                    elif cgeom == 'sphere':
                        cr = float(self.coll_size_x.text())
                        collision_geom = 'sphere'; collision_size = (cr, cr, cr)
                except Exception:
                    QMessageBox.warning(self,"Error","Collision size entries must be numeric"); return

        # create and store link (no jitter/offset)
        stored_size = (sx, sy, sz)
        self.model.links[name] = Link(name, geom, stored_size, mass, inertia, manual_inertia_flag, origin, rpy, include_collision, collision_geom, collision_size)
        self._refresh_elements_list(); self._refresh_link_combos(); self.update_preview_and_view()

    def _on_add_joint(self):
        name = self.joint_name.text().strip()
        if not name:
            QMessageBox.warning(self,"Error","Joint name required"); return
        jtype = self.joint_type.currentText()
        parent = self.parent_combo.currentText(); child = self.child_combo.currentText()
        if parent == "" or child == "":
            QMessageBox.warning(self,"Error","Parent and child must be selected"); return
        if parent not in self.model.links or child not in self.model.links:
            QMessageBox.warning(self,"Error","Parent and child must exist"); return
        try:
            ox=float(self.j_origin_x.text()); oy=float(self.j_origin_y.text()); oz=float(self.j_origin_z.text())
            rr=float(self.j_origin_r.text()); rp=float(self.j_origin_p.text()); ry=float(self.j_origin_yaw.text())
            ax=float(self.axis_x.text()); ay=float(self.axis_y.text()); az=float(self.axis_z.text())
            low=None; high=None
            if self.limit_l.text().strip()!="" and self.limit_u.text().strip()!="":
                low=float(self.limit_l.text()); high=float(self.limit_u.text())
            eff = float(self.effort.text()) if self.effort.text().strip()!="" else 10.0
            vel = float(self.velocity.text()) if self.velocity.text().strip()!="" else 1.0
        except ValueError:
            QMessageBox.warning(self,"Error","Numeric fields must be numeric"); return
        joint = Joint(name, jtype, parent, child, origin_xyz=(ox,oy,oz), origin_rpy=(rr,rp,ry), axis=(ax,ay,az), limit=(low,high) if low is not None else None, effort=eff, velocity=vel)
        self.model.joints[name] = joint
        self._refresh_elements_list(); self.update_preview_and_view()

    def _delete_selected(self):
        it = self.elements_list.currentItem()
        if not it:
            QMessageBox.warning(self,"Error","No element selected"); return
        kind, name = it.data(Qt.UserRole)
        if kind == 'link':
            if name in self.model.links: del self.model.links[name]
            # remove joints referencing that link
            to_delete = [jn for jn,j in self.model.joints.items() if j.parent==name or j.child==name]
            for jn in to_delete: del self.model.joints[jn]
        else:
            if name in self.model.joints: del self.model.joints[name]
        self._refresh_elements_list(); self._refresh_link_combos(); self.update_preview_and_view()

    def _load_selected_element(self, item=None):
        if item is None: item = self.elements_list.currentItem()
        if not item: return
        kind, name = item.data(Qt.UserRole)
        if kind == 'link':
            l = self.model.links[name]
            self.link_name.setText(l.name)
            self.geom_combo.setCurrentText(l.geom_type)
            # adapt size fields to geometry then set values
            self._update_size_fields(l.geom_type)
            self.size_x.setText(str(l.size[0])); self.size_y.setText(str(l.size[1])); self.size_z.setText(str(l.size[2]))
            self.mass_input.setText(str(l.mass))
            if l.manual_inertia and l.inertia:
                self.manual_inertia_cb.setChecked(True)
                for k,v in l.inertia.items():
                    if k in self.inertia_fields: self.inertia_fields[k].setText(str(v))
            else:
                self.manual_inertia_cb.setChecked(False)
            self.collision_cb.setChecked(bool(l.include_collision))
            if l.include_collision and l.collision_geom:
                self.collision_mode.setCurrentText("Manual set")
                self.coll_geom_combo.setCurrentText(l.collision_geom)
                self._update_collision_size_fields(l.collision_geom)
                if l.collision_size:
                    self.coll_size_x.setText(str(l.collision_size[0])); self.coll_size_y.setText(str(l.collision_size[1])); self.coll_size_z.setText(str(l.collision_size[2]))
            else:
                self.collision_mode.setCurrentText("Use identical")
            if l.origin != (0.0,0.0,0.0) or l.rpy != (0.0,0.0,0.0):
                self.manual_origin_cb.setChecked(True)
                self.origin_x.setText(str(l.origin[0])); self.origin_y.setText(str(l.origin[1])); self.origin_z.setText(str(l.origin[2]))
                self.origin_r.setText(str(l.rpy[0])); self.origin_p.setText(str(l.rpy[1])); self.origin_yaw.setText(str(l.rpy[2]))
            else:
                self.manual_origin_cb.setChecked(False)
        else:
            j = self.model.joints[name]
            self.joint_name.setText(j.name); self.joint_type.setCurrentText(j.jtype)
            self._refresh_link_combos()
            self.parent_combo.setCurrentText(j.parent); self.child_combo.setCurrentText(j.child)
            self.j_origin_x.setText(str(j.origin_xyz[0])); self.j_origin_y.setText(str(j.origin_xyz[1])); self.j_origin_z.setText(str(j.origin_xyz[2]))
            self.j_origin_r.setText(str(j.origin_rpy[0])); self.j_origin_p.setText(str(j.origin_rpy[1])); self.j_origin_yaw.setText(str(j.origin_rpy[2]))
            self.axis_x.setText(str(j.axis[0])); self.axis_y.setText(str(j.axis[1])); self.axis_z.setText(str(j.axis[2]))
            if j.limit:
                self.limit_l.setText(str(j.limit[0])); self.limit_u.setText(str(j.limit[1]))
            else:
                self.limit_l.setText(""); self.limit_u.setText("")
            self.effort.setText(str(j.effort)); self.velocity.setText(str(j.velocity))

    def _apply_edited_urdf(self):
        txt = self.urdf_text.toPlainText()
        ok = self.model.load_from_urdf_string(txt)
        if not ok:
            QMessageBox.warning(self,"Error","Invalid URDF syntax; cannot parse."); return
        self._refresh_elements_list(); self._refresh_link_combos(); self.update_preview_and_view()
        QMessageBox.information(self,"Applied","Model updated from edited URDF text")

    def _export_urdf(self):
        txt = self.urdf_text.toPlainText()
        path, _ = QFileDialog.getSaveFileName(self,"Save URDF","robot.urdf","URDF files (*.urdf);;All files (*)")
        if not path: return
        try:
            with open(path,'w') as f: f.write(txt)
            QMessageBox.information(self,"Saved",f"Saved to {path}")
        except Exception as e:
            QMessageBox.warning(self,"Error",f"Failed to save: {e}")

    def _joint_type_changed(self, t):
        is_fixed = (t == "fixed")
        for w in (self.axis_x, self.axis_y, self.axis_z, self.limit_l, self.limit_u, self.effort, self.velocity):
            w.setDisabled(is_fixed)
        # origin remains editable

    def update_preview_and_view(self):
        # update urdf text
        self.urdf_text.setPlainText(self.model.to_urdf_string())
        # update lists and combos
        self._refresh_elements_list()
        self._refresh_link_combos()
        # update 3D view: the GLWidget reads from self.model.links (so ensure it uses latest)
        # but GLWidget expects self.model to stay; we simply call update()
        self.gl.update()

# ------------------ Run ------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = URDFBuilderUI()
    w.show()
    sys.exit(app.exec())
