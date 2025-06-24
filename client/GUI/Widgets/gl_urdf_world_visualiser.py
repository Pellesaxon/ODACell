
"""
A 3D URDF/Xacro visualizer using PySide6 and PyOpenGL.

This module provides a standalone desktop application for loading, parsing, and
rendering robot models defined in URDF or Xacro formats. An example
interactive 3D view where users can rotate and zoom the camera, and a control
panel with sliders to manipulate the robot's joint angles is also included.

Key Components:
- URDFParser: A class to load URDF/Xacro files, build a kinematic tree,
  and extract visual geometry. It supports mesh files as well as primitive
  shapes (box, cylinder, sphere).
- URDFOpenGLWidget: A QOpenGLWidget that handles all the 3D rendering of the
  robot model and the environment. It uses VBOs for efficient mesh rendering.

@author: Lukas LT
@version: 1.0
@date: 2025-06-24
"""

# --- Imports ---
import sys
import os
import math
import numpy as np
import trimesh

from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                               QHBoxLayout, QSlider, QLabel, QSplitter)
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtCore import Qt, Slot
from PySide6.QtGui import QMouseEvent, QWheelEvent

from urdf_parser_py.urdf import URDF, Mesh, Box, Cylinder, Sphere
from xacrodoc import XacroDoc

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import glutSolidCube

# --- Helper Functions ---
def create_translation_matrix(v):
    """Creates a 4x4 homogeneous translation matrix.

    Args:
        v: A 3-element array representing the translation vector [x, y, z].

    Returns:
        A 4x4 numpy array representing the translation matrix.

    """
    M = np.eye(4, dtype=np.float32)
    M[:3, 3] = v
    return M


def create_rotation_matrix_rpy(rpy):
    """Creates a 4x4 homogeneous rotation matrix from roll, pitch, yaw angles.

    The rotation is applied in the order: Z (yaw), Y (pitch), X (roll).

    Args:
        rpy: A tuple containing the roll, pitch, and yaw angles in radians.

    Returns:
        A 4x4 numpy array representing the rotation matrix.

    """
    roll, pitch, yaw = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]], dtype=np.float32)
    Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]], dtype=np.float32)
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]], dtype=np.float32)
    R = Rz @ Ry @ Rx
    M = np.eye(4, dtype=np.float32)
    M[:3, :3] = R
    return M


def create_rotation_matrix_axis_angle(axis, angle):
    """Creates a 4x4 homogeneous rotation matrix from an axis and an angle.

    Args:
        axis: A 3-element array representing the axis of rotation.
        angle: The angle of rotation in radians.

    Returns:
        A 4x4 numpy array representing the rotation matrix.
    
    """
    axis = np.array(axis, dtype=np.float32)
    if np.linalg.norm(axis) > 0:
        axis /= np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1 - c
    R = np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c]
    ], dtype=np.float32)
    M = np.eye(4, dtype=np.float32)
    M[:3, :3] = R
    return M

# --- URDF Parser with primitives support ---
class URDFParser:
    """
    Parses URDF/Xacro files and builds an internal robot representation.

    This class serves as a wrapper around 'urdf_parser_py' and 'xacrodoc'
    to handle loading robot models from one or more files. It builds a
    kinematic tree and provides methods to access visual and joint information.

    Attributes:
        urdf_paths (list[str] | str): The path(s) to the URDF/Xacro files.
        project_common_dir (str): The common directory for project resources, e.g. meshes used in both client and server.
        robot (URDF | None): The parsed robot object from 'urdf_parser_py'.
        link_map (dict): A map from link names to link objects.
        joint_map (dict): A map from joint names to joint objects.
        kinematic_tree (dict): A representation of the robot's structure, mapping
                               parent link names to a list of their child joint names.
        mimic_joints (dict): A map of mimic joints to their master joint and parameters.
    
    """
    def __init__(self, urdf_paths: list[str] | str):
        """Initializes the URDFParser.

        Args:
            urdf_paths: A single path to a URDF/Xacro file, or a list of paths
                        for Xacro files with includes. If a list is provided,
                        the root file must be a Xacro file that includes others.
        """
        self.urdf_paths = urdf_paths
        file_dir = os.path.dirname(os.path.abspath(__file__))
        self.project_common_dir = os.path.join(file_dir, "..", "..", "..", "common") 
        self.robot = None
        self.link_map = {}
        self.joint_map = {}
        self.kinematic_tree = {}
        self.mimic_joints = {}

    def parse(self):
        """
        Parses the URDF/Xacro file(s) and builds the kinematic tree.

        This method handles both single URDF/Xacro files and multi-file Xacro
        projects by first processing compiling them to a single URDF with `xacrodoc` if necessary.
        On success, it populates the 'robot', 'link_map', 'joint_map', and
        other instance attributes.

        Returns:
            True if parsing was successful, False otherwise.
        """
        try:
            # Handle multi-file xacro by providing includes
            if type(self.urdf_paths) is list:
                includes = self.urdf_paths
                doc = XacroDoc.from_includes(includes=includes, resolve_packages=False)
                urdf_str = doc.to_urdf_string(pretty=True)
                self.robot = URDF.from_xml_string(urdf_str)
            else:
                if not type(self.urdf_paths) is str:
                    raise ValueError("URDF/Xacro paths must be a list of strings or a single string")
                # Handle single URDF
                self.robot = URDF.from_xml_file(self.urdf_paths)
            self._build_kinematic_tree()
            return True
        except Exception as e:
            print(f"Failed to parse URDF: {e}")
            return False

    def _build_kinematic_tree(self):
        """Populates the internal kinematic tree from the parsed robot model."""
        if not self.robot:
            return
        self.link_map = self.robot.link_map
        self.joint_map = self.robot.joint_map
        self.kinematic_tree = {link.name: [] for link in self.robot.links}
        for joint in self.robot.joints:
            if joint.parent in self.kinematic_tree:
                self.kinematic_tree[joint.parent].append(joint.name)
            if joint.mimic:
                self.mimic_joints[joint.name] = {
                    "joint": joint.mimic.joint,
                    "multiplier": joint.mimic.multiplier or 1.0,
                    "offset": joint.mimic.offset or 0.0
                }

    def get_visuals(self):
        """Extracts all visual geometry information from the parsed robot.

        Iterates through all links and their visual components, extracting
        information needed for rendering. It resolves mesh file paths relative
        to the project root.

        Returns:
            A list of tuples, where each tuple contains:
            (str: geometry_type ('mesh', 'box', etc.),
             urdf.Visual: the visual object,
             any: geometry-specific data (filepath, size, etc.))
        """
        visuals = []
        for link in self.robot.link_map.values():
            if not link.visuals:
                continue
            for vis in link.visuals:
                geom = vis.geometry
                if isinstance(geom, Mesh):
                    fn = geom.filename.replace('package://', '')
                    full_path = os.path.join(self.project_common_dir, fn)
                    visuals.append(('mesh', vis, full_path))
                elif isinstance(geom, Box):
                    visuals.append(('box', vis, tuple(geom.size)))
                elif isinstance(geom, Cylinder):
                    visuals.append(('cylinder', vis, (geom.length, geom.radius)))
                elif isinstance(geom, Sphere):
                    visuals.append(('sphere', vis, geom.radius))
        return visuals

# --- OpenGL Widget ---
class URDFOpenGLWidget(QOpenGLWidget):
    """
    A PySide6 QOpenGLWidget for rendering a URDF model.

    This widget handles the OpenGL context, camera controls, and drawing of the
    robot model. It loads geometry into GPU buffers (VBOs) for efficient
    rendering and updates the robot's pose based on joint angles.

    Attributes:
        parser (URDFParser): The parser containing the robot data.
        camera_pitch (float): The camera's pitch angle in degrees.
        camera_yaw (float): The camera's yaw angle in degrees.
        camera_distance (float): The distance of the camera from the origin.
        joint_angles (dict): A map of joint names to their current angle in radians.
        visual_data (dict): A cache for OpenGL data (VBOs, colors) for each visual.
    """
    def __init__(self, urdf_parser, parent=None):
        """Initializes the OpenGL widget.

        Args:
            urdf_parser: An instance of `URDFParser` with a loaded robot model.
            parent: The parent widget, if any.
        """
        super().__init__(parent)
        self.parser = urdf_parser
        self.robot = self.parser.robot
        self.camera_pitch = 45.0
        self.camera_yaw = -135.0
        self.camera_distance = 1.0
        self.last_mouse_pos = None
        self.visual_data = {}
        # Initialize joint angles to 0, respecting mimic joints
        self.joint_angles = {j.name: 0.0 for j in self.robot.joints if j.type in ['revolute', 'prismatic']}
        self._update_mimic_joints()
        self.setFocusPolicy(Qt.StrongFocus)

    @Slot(str, int)
    def set_joint_angle(self, joint_name, angle_degrees):
        """
        Setter to update the angle of a specific joint.

        The input angle is in degrees and is converted to radians for internal
        use. The angle is clamped to the joint's defined limits.

        Args:
            joint_name: The name of the joint to update.
            angle_degrees: The new angle for the joint in degrees.
        """
        if joint_name in self.joint_angles:
            joint = self.parser.joint_map[joint_name]
            rad = math.radians(angle_degrees)
            if joint.limit:
                rad = max(joint.limit.lower, min(joint.limit.upper, rad))
            self.joint_angles[joint_name] = rad
            self._update_mimic_joints()
            self.update() # Trigger a repaint to reflect the new joint angle

    def _update_mimic_joints(self):
        """Updates the angles of mimic joints based on their master joint's angle."""
        for mimic_name, info in self.parser.mimic_joints.items():
            master = info['joint']
            if master in self.joint_angles:
                self.joint_angles[mimic_name] = self.joint_angles[master] * info['multiplier'] + info['offset']

    def initializeGL(self):
        """Sets up the OpenGL rendering context."""
        glClearColor(0.2, 0.2, 0.3, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_NORMALIZE)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, (5,5,5,1))
        glLightfv(GL_LIGHT0, GL_AMBIENT,  (0.3,0.3,0.3,1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  (0.8,0.8,0.8,1))
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        # create one quadric for cylinders / spheres
        self.quad = gluNewQuadric()
        self._load_visuals()

    def _load_visuals(self):
        """Loads robot's visual geometries into OpenGL buffers or caches data."""
        for kind, vis, data in self.parser.get_visuals():
            # Load mesh .STL/.DAE/ similar into OpenGL buffers using trimesh
            if kind == 'mesh':
                try:
                    mesh = trimesh.load(data, force='mesh')
                    verts = np.array(mesh.vertices, dtype=np.float32)
                    vbo = glGenBuffers(1)
                    glBindBuffer(GL_ARRAY_BUFFER, vbo)
                    glBufferData(GL_ARRAY_BUFFER, verts.nbytes, verts, GL_STATIC_DRAW)
                    normals = np.array(mesh.vertex_normals, dtype=np.float32)
                    nbo = glGenBuffers(1)
                    glBindBuffer(GL_ARRAY_BUFFER, nbo)
                    glBufferData(GL_ARRAY_BUFFER, normals.nbytes, normals, GL_STATIC_DRAW)
                    faces = np.array(mesh.faces, dtype=np.uint32).flatten()
                    ebo = glGenBuffers(1)
                    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo)
                    glBufferData(GL_ELEMENT_ARRAY_BUFFER, faces.nbytes, faces, GL_STATIC_DRAW)
                    color = (0.8,0.8,0.8,1.0)
                    if vis.material and vis.material.color:
                        color = vis.material.color.rgba
                    # Cache the VBO IDs and other info for rendering
                    self.visual_data[id(vis)] = {'type':'mesh', 'vbo':vbo, 'nbo':nbo, 'ebo':ebo,
                                                  'count':len(faces), 'color':color}
                except Exception as e:
                    print(f"Warning: could not load mesh {data}: {e}")
                    self.visual_data[id(vis)] = None
            else:
                # For primitives (box, sphere, etc), just cache their parameters and color
                color = (0.8,0.8,0.8,1.0)
                if vis.material and vis.material.color:
                    color = vis.material.color.rgba
                self.visual_data[id(vis)] = {'type':kind, 'params':data, 'color':color}

    def resizeGL(self, w, h):
        """Handles widget resize events by updating the perspective projection."""
        glViewport(0,0,w,h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w/h if h else 1, 0.01, 50)

    def paintGL(self):
        """Renders the scene for each frame."""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # Set up camera view and transform/rotate properly, stupid thing starts off clipped through the floor
        glTranslatef(0.0, -0.4, -self.camera_distance)
        glRotatef(self.camera_pitch, 1,0,0)
        glRotatef(self.camera_yaw,   0,1,0)
        self._draw_grid()
        # Start drawing the robot from its root link
        root = self.robot.get_root()
        # Apply a base rotation to align the robot with the world frame (Y-up to Z-up) (stand it up vertically)
        base_tf = create_rotation_matrix_rpy([-math.pi/2,0,0])
        self._draw_link_and_children(root, base_tf)

    def _draw_grid(self):
        """Draws a reference grid on the XZ plane."""
        glDisable(GL_LIGHTING)
        glColor3f(0.5,0.5,0.5)
        glBegin(GL_LINES)
        for i in range(-10,11):
            glVertex3f(i*0.2,0,-2); glVertex3f(i*0.2,0,2)
            glVertex3f(-2,0,i*0.2); glVertex3f(2,0,i*0.2)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_link_and_children(self, link_name, parent_tf):
        """Recursively draws a robot link and all of its children.

        Args:
            link_name: The name of the link to draw.
            parent_tf: The 4x4 transformation matrix from the world frame to
                       this link's parent frame.
        """
        link = self.parser.link_map.get(link_name)
        if link and link.visuals:
            for vis in link.visuals:
                data = self.visual_data.get(id(vis))
                if not data: continue
                glPushMatrix()
                glMultMatrixf(parent_tf.T) # Apply parent transform
                # Apply the local transform of the visual element
                if vis.origin:
                    T = create_translation_matrix(vis.origin.xyz)
                    R = create_rotation_matrix_rpy(vis.origin.rpy)
                    glMultMatrixf((T @ R).T)
                glColor4f(*data['color'])
                if data['type'] == 'mesh':
                    # draw mesh
                    glEnableClientState(GL_VERTEX_ARRAY)
                    glEnableClientState(GL_NORMAL_ARRAY)
                    glBindBuffer(GL_ARRAY_BUFFER, data['vbo'])
                    glVertexPointer(3, GL_FLOAT, 0, None)
                    glBindBuffer(GL_ARRAY_BUFFER, data['nbo'])
                    glNormalPointer(GL_FLOAT, 0, None)
                    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, data['ebo'])
                    glDrawElements(GL_TRIANGLES, data['count'], GL_UNSIGNED_INT, None)
                    glDisableClientState(GL_VERTEX_ARRAY)
                    glDisableClientState(GL_NORMAL_ARRAY)
                elif data['type'] == 'box': # draw box
                    sx, sy, sz = data['params']
                    glPushMatrix()
                    glScalef(sx, sy, sz)
                    glutSolidCube(1.0)
                    glPopMatrix()
                elif data['type'] == 'cylinder': # draw cylinder
                    length, radius = data['params']
                    glPushMatrix()
                    # cylinder axis is along z
                    gluCylinder(self.quad, radius, radius, length, 20, 4)
                    glPopMatrix()
                elif data['type'] == 'sphere': # draw sphere
                    radius = data['params']
                    gluSphere(self.quad, radius, 20, 20)
                glPopMatrix()
        # recurse children
        for jn in self.parser.kinematic_tree.get(link_name, []):
            joint = self.parser.joint_map[jn]
            # Calculate the transform from the parent link to the child link
            static_tf = np.eye(4, dtype=np.float32)
            if joint.origin:
                T = create_translation_matrix(joint.origin.xyz)
                R = create_rotation_matrix_rpy(joint.origin.rpy)
                static_tf = T @ R
            # Apply dynamic transform based on joint type and angle
            rot_tf = np.eye(4, dtype=np.float32)
            if joint.type == 'revolute':
                rot_tf = create_rotation_matrix_axis_angle(joint.axis, self.joint_angles.get(jn, 0.0))
            elif joint.type == 'prismatic':
                # For prismatic joints, we translate along the axis (prismatic == single axis linear motion)
                translation = np.array(joint.axis) * self.joint_angles.get(jn, 0.0)
                rot_tf = create_translation_matrix(translation)
            # The final transform for the child is parent_tf @ static @ dynamic
            child_tf = parent_tf @ static_tf @ rot_tf
            self._draw_link_and_children(joint.child, child_tf)

    def mousePressEvent(self, event: QMouseEvent): # self-explanatory
        self.last_mouse_pos = event.position()

    def mouseMoveEvent(self, event: QMouseEvent): #--//--#
        if self.last_mouse_pos:
            dx = event.position().x() - self.last_mouse_pos.x()
            dy = event.position().y() - self.last_mouse_pos.y()
            if event.buttons() & Qt.LeftButton:
                self.camera_yaw   += dx * 0.5
                self.camera_pitch = max(-90, min(90, self.camera_pitch + dy*0.5))
            self.last_mouse_pos = event.position()
            self.update()

    def wheelEvent(self, event: QWheelEvent): #--//--#
        self.camera_distance = max(0.2, self.camera_distance - event.angleDelta().y()*0.002)
        self.update()

# Example window to display the OpenGL widget, and some controls
class MainWindow(QMainWindow):
    def __init__(self, parser):
        super().__init__()
        self.setWindowTitle("PySide6 URDF Visualizer")
        self.setGeometry(100,100,1200,800)
        self.parser = parser
        central = QWidget()
        self.setCentralWidget(central)
        hl = QHBoxLayout(central)
        splitter = QSplitter(Qt.Horizontal)
        hl.addWidget(splitter)
        self.gl_widget = URDFOpenGLWidget(parser)
        # Controls
        ctrl = QWidget(); vlay = QVBoxLayout(ctrl); vlay.setAlignment(Qt.AlignTop)
        vlay.addWidget(QLabel("<b>Joint Controls</b>"))
        masters = [j for j in parser.robot.joints if j.type=='revolute' and j.name not in parser.mimic_joints]
        for j in sorted(masters, key=lambda j: j.name):
            lo = int(math.degrees(j.limit.lower))
            hi = int(math.degrees(j.limit.upper))
            lbl = QLabel(f"{j.name}: 0°")
            sld = QSlider(Qt.Horizontal)
            sld.setRange(lo, hi); sld.setValue(0)
            sld.valueChanged.connect(lambda v, name=j.name, lab=lbl: (lab.setText(f"{name}: {v}°"), self.gl_widget.set_joint_angle(name, v)))
            vlay.addWidget(lbl); vlay.addWidget(sld)
        splitter.addWidget(ctrl)
        splitter.addWidget(self.gl_widget)
        splitter.setSizes([300, 900])

if __name__ == '__main__':
    app = QApplication(sys.argv)
    base = os.path.dirname(os.path.abspath(__file__))
    world_urdf_file = os.path.join(base, "..", "..", "..", "common", "mg400_description", "urdf", "world.xacro")
    mg400_urdf_file = os.path.join(base, "..", "..", "..", "common", "mg400_description", "urdf", "mg400.urdf.xacro")
    if not os.path.exists(world_urdf_file):
        print(f"Error: URDF not found at {world_urdf_file}")
        sys.exit(1)
    parser = URDFParser([world_urdf_file, mg400_urdf_file])
    if not parser.parse():
        sys.exit(1)
    win = MainWindow(parser)
    win.show()
    sys.exit(app.exec())
