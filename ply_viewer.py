"""
PLY File Viewer - Point Cloud & Mesh Viewer

Mouse Controls:
    Left button    - rotate camera
    Mouse wheel    - zoom in/out
    Middle button  - pan
    Right button   - rotate around Z axis

Hotkeys:
    O          - open file
    R          - reset camera view
    +/-        - increase/decrease point size
    1          - mode: original colors
    2          - mode: height map (Z axis)
    3          - mode: depth map (Y axis)
    4          - mode: uniform color
    B          - toggle background (dark/light)
    A          - show/hide coordinate axes
    X          - show/hide bounding box
    N          - estimate and show normals
    G          - show/hide grid
    I          - print model info to console
    P          - save screenshot
    F          - fullscreen
    H          - print help to console
    Q / Esc    - quit
"""

import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np
import os
import sys
import time


class PLYViewer:
    """PLY file viewer with Open3D GUI control panel."""

    MENU_OPEN = 1
    MENU_SAVE_SCREENSHOT = 2
    MENU_QUIT = 3
    MENU_ABOUT = 10

    def __init__(self, file_path=None):
        self.point_cloud = None
        self.original_colors = None
        self.mesh = None
        self.geometry_name = "model"
        self.axes_visible = False
        self.bbox_visible = False
        self.grid_visible = False
        self.normals_visible = False
        self.is_dark_bg = True
        self.point_size = 3.0
        self.color_mode = "original"  # original, height, depth, uniform

        # Init GUI
        gui.Application.instance.initialize()
        self.app = gui.Application.instance

        self.window = self.app.create_window("PLY Viewer", 1400, 900)
        self.window.set_on_close(self._on_close)

        em = self.window.theme.font_size

        # === Menu ===
        if gui.Application.instance.menubar is None:
            file_menu = gui.Menu()
            file_menu.add_item("Open File...         O", self.MENU_OPEN)
            file_menu.add_item("Save Screenshot      P", self.MENU_SAVE_SCREENSHOT)
            file_menu.add_separator()
            file_menu.add_item("Quit                 Q", self.MENU_QUIT)

            help_menu = gui.Menu()
            help_menu.add_item("About", self.MENU_ABOUT)

            menu = gui.Menu()
            menu.add_menu("File", file_menu)
            menu.add_menu("Help", help_menu)
            gui.Application.instance.menubar = menu

        self.window.set_on_menu_item_activated(self.MENU_OPEN, self._on_menu_open)
        self.window.set_on_menu_item_activated(self.MENU_SAVE_SCREENSHOT, self._on_save_screenshot)
        self.window.set_on_menu_item_activated(self.MENU_QUIT, self._on_close)
        self.window.set_on_menu_item_activated(self.MENU_ABOUT, self._on_about)

        # === 3D Scene ===
        self._scene = gui.SceneWidget()
        self._scene.scene = rendering.Open3DScene(self.window.renderer)
        self._scene.set_on_key(self._on_key)

        # === Control Panel (right side) ===
        self._panel = gui.Vert(0, gui.Margins(0.5 * em, 0.5 * em, 0.5 * em, 0.5 * em))
        self._panel_width = 280

        # -- Title --
        title = gui.Label("  Control Panel")
        title.text_color = gui.Color(0.3, 0.7, 1.0)
        self._panel.add_child(title)
        self._panel.add_child(gui.Label(""))

        # -- Open file button --
        btn_open = gui.Button("  Open PLY File  ")
        btn_open.set_on_clicked(self._on_menu_open)
        self._panel.add_child(btn_open)
        self._panel.add_child(gui.Label(""))

        # -- Point size --
        lbl_pt = gui.Label("Point Size")
        self._panel.add_child(lbl_pt)
        self._slider_point_size = gui.Slider(gui.Slider.DOUBLE)
        self._slider_point_size.set_limits(1.0, 20.0)
        self._slider_point_size.double_value = self.point_size
        self._slider_point_size.set_on_value_changed(self._on_point_size_changed)
        self._panel.add_child(self._slider_point_size)
        self._panel.add_child(gui.Label(""))

        # -- Color mode --
        lbl_color = gui.Label("Color Mode")
        self._panel.add_child(lbl_color)

        self._color_combo = gui.Combobox()
        self._color_combo.add_item("Original Colors")
        self._color_combo.add_item("Height Map (Z)")
        self._color_combo.add_item("Depth Map (Y)")
        self._color_combo.add_item("Uniform Color")
        self._color_combo.selected_index = 0
        self._color_combo.set_on_selection_changed(self._on_color_mode_changed)
        self._panel.add_child(self._color_combo)
        self._panel.add_child(gui.Label(""))

        # -- Uniform color picker --
        lbl_uniform = gui.Label("Uniform Color")
        self._panel.add_child(lbl_uniform)
        self._color_edit = gui.ColorEdit()
        self._color_edit.color_value = gui.Color(0.2, 0.6, 1.0)
        self._color_edit.set_on_value_changed(self._on_uniform_color_changed)
        self._panel.add_child(self._color_edit)
        self._panel.add_child(gui.Label(""))

        # -- Display options --
        lbl_opts = gui.Label("Display")
        self._panel.add_child(lbl_opts)

        self._cb_axes = gui.Checkbox("Axes             (A)")
        self._cb_axes.checked = False
        self._cb_axes.set_on_checked(self._on_toggle_axes)
        self._panel.add_child(self._cb_axes)

        self._cb_bbox = gui.Checkbox("Bounding Box     (X)")
        self._cb_bbox.checked = False
        self._cb_bbox.set_on_checked(self._on_toggle_bbox)
        self._panel.add_child(self._cb_bbox)

        self._cb_grid = gui.Checkbox("Grid             (G)")
        self._cb_grid.checked = False
        self._cb_grid.set_on_checked(self._on_toggle_grid)
        self._panel.add_child(self._cb_grid)

        self._cb_normals = gui.Checkbox("Normals          (N)")
        self._cb_normals.checked = False
        self._cb_normals.set_on_checked(self._on_toggle_normals)
        self._panel.add_child(self._cb_normals)
        self._panel.add_child(gui.Label(""))

        # -- Background --
        lbl_bg = gui.Label("Background")
        self._panel.add_child(lbl_bg)
        self._cb_dark_bg = gui.Checkbox("Dark Background  (B)")
        self._cb_dark_bg.checked = True
        self._cb_dark_bg.set_on_checked(self._on_toggle_bg)
        self._panel.add_child(self._cb_dark_bg)
        self._panel.add_child(gui.Label(""))

        # -- Action buttons --
        btn_reset = gui.Button("  Reset View (R)  ")
        btn_reset.set_on_clicked(self._on_reset_view)
        self._panel.add_child(btn_reset)

        btn_screenshot = gui.Button("  Screenshot (P)  ")
        btn_screenshot.set_on_clicked(self._on_save_screenshot)
        self._panel.add_child(btn_screenshot)
        self._panel.add_child(gui.Label(""))

        # -- Model info --
        self._info_label = gui.Label("No file loaded")
        self._info_label.text_color = gui.Color(0.7, 0.7, 0.7)
        self._panel.add_child(self._info_label)

        # -- Help hint --
        self._panel.add_child(gui.Label(""))
        help_label = gui.Label("H - help in console")
        help_label.text_color = gui.Color(0.5, 0.5, 0.5)
        self._panel.add_child(help_label)

        # === Layout ===
        self.window.set_on_layout(self._on_layout)
        self.window.add_child(self._scene)
        self.window.add_child(self._panel)

        # Initial scene settings
        self._apply_background()
        self._scene.scene.set_background([0.12, 0.12, 0.15, 1.0])

        # Load file if provided
        if file_path and os.path.isfile(file_path):
            self._load_file(file_path)

    def _on_layout(self, layout_context):
        """Widget layout."""
        r = self.window.content_rect
        panel_width = min(self._panel_width, r.width * 0.3)
        self._scene.frame = gui.Rect(r.x, r.y, r.width - panel_width, r.height)
        self._panel.frame = gui.Rect(r.get_right() - panel_width, r.y, panel_width, r.height)

    def _on_close(self):
        """Close window."""
        self.app.quit()
        return True

    # ============================
    #       File Loading
    # ============================

    def _on_menu_open(self):
        """Open file dialog."""
        dlg = gui.FileDialog(gui.FileDialog.OPEN, "Open PLY File", self.window.theme)
        dlg.add_filter(".ply", "PLY Files (.ply)")
        dlg.add_filter("", "All Files")
        dlg.set_on_cancel(self._on_file_dialog_cancel)
        dlg.set_on_done(self._on_file_dialog_done)
        self.window.show_dialog(dlg)

    def _on_file_dialog_cancel(self):
        self.window.close_dialog()

    def _on_file_dialog_done(self, file_path):
        self.window.close_dialog()
        self._load_file(file_path)

    def _load_file(self, file_path):
        """Load PLY file (point cloud or mesh)."""
        try:
            # Remove old geometry
            self._scene.scene.remove_geometry(self.geometry_name)
            self._remove_helpers()

            # Try as mesh first
            mesh = o3d.io.read_triangle_mesh(file_path)
            if mesh.has_triangles() and len(mesh.triangles) > 0:
                mesh.compute_vertex_normals()
                self.mesh = mesh
                self.point_cloud = None

                pcd = mesh.sample_points_uniformly(number_of_points=len(mesh.vertices))
                self.original_colors = np.asarray(mesh.vertex_colors).copy() if mesh.has_vertex_colors() else None

                mat = rendering.MaterialRecord()
                mat.shader = "defaultLit"
                self._scene.scene.add_geometry(self.geometry_name, mesh, mat)
            else:
                # Load as point cloud
                pcd = o3d.io.read_point_cloud(file_path)
                if len(pcd.points) == 0:
                    self._show_message("Error", f"File is empty or failed to load:\n{file_path}")
                    return

                self.point_cloud = pcd
                self.mesh = None
                self.original_colors = np.asarray(pcd.colors).copy() if pcd.has_colors() else None

                mat = rendering.MaterialRecord()
                mat.shader = "defaultUnlit"
                mat.point_size = self.point_size
                self._scene.scene.add_geometry(self.geometry_name, pcd, mat)

            # Setup camera
            bounds = self._scene.scene.bounding_box
            self._scene.setup_camera(60.0, bounds, bounds.get_center())

            # Restore helpers
            if self.axes_visible:
                self._add_axes()
            if self.bbox_visible:
                self._add_bbox()
            if self.grid_visible:
                self._add_grid()

            # Update info
            self._update_info(file_path)

            print(f"\n[OK] Loaded: {file_path}")

        except Exception as e:
            self._show_message("Load Error", f"Failed to load file:\n{str(e)}")
            print(f"[ERROR] {e}")

    def _update_info(self, file_path=""):
        """Update model info label."""
        info_lines = []
        name = os.path.basename(file_path) if file_path else "-"
        info_lines.append(f"File: {name}")

        if self.point_cloud is not None:
            pts = np.asarray(self.point_cloud.points)
            info_lines.append(f"Points: {len(pts):,}")
            if len(pts) > 0:
                mn = pts.min(axis=0)
                mx = pts.max(axis=0)
                sz = mx - mn
                info_lines.append(f"X: {mn[0]:.2f}..{mx[0]:.2f}")
                info_lines.append(f"Y: {mn[1]:.2f}..{mx[1]:.2f}")
                info_lines.append(f"Z: {mn[2]:.2f}..{mx[2]:.2f}")
                info_lines.append(f"Size: {sz[0]:.1f}x{sz[1]:.1f}x{sz[2]:.1f}")
            has_colors = "Yes" if self.point_cloud.has_colors() else "No"
            has_normals = "Yes" if self.point_cloud.has_normals() else "No"
            info_lines.append(f"Colors: {has_colors}")
            info_lines.append(f"Normals: {has_normals}")
        elif self.mesh is not None:
            verts = np.asarray(self.mesh.vertices)
            tris = np.asarray(self.mesh.triangles)
            info_lines.append(f"Vertices: {len(verts):,}")
            info_lines.append(f"Triangles: {len(tris):,}")
            if len(verts) > 0:
                mn = verts.min(axis=0)
                mx = verts.max(axis=0)
                sz = mx - mn
                info_lines.append(f"Size: {sz[0]:.1f}x{sz[1]:.1f}x{sz[2]:.1f}")

        self._info_label.text = "\n".join(info_lines)

    # ============================
    #       Color Modes
    # ============================

    def _apply_color_mode(self, mode=None):
        """Apply color mode to point cloud."""
        if mode:
            self.color_mode = mode

        pcd = self.point_cloud
        if pcd is None:
            return

        points = np.asarray(pcd.points)
        if len(points) == 0:
            return

        if self.color_mode == "original":
            if self.original_colors is not None and len(self.original_colors) > 0:
                pcd.colors = o3d.utility.Vector3dVector(self.original_colors)
            else:
                colors = np.full((len(points), 3), [0.2, 0.6, 1.0])
                pcd.colors = o3d.utility.Vector3dVector(colors)

        elif self.color_mode == "height":
            z = points[:, 2]
            z_norm = (z - z.min()) / (z.max() - z.min() + 1e-10)
            colors = self._colormap_turbo(z_norm)
            pcd.colors = o3d.utility.Vector3dVector(colors)

        elif self.color_mode == "depth":
            y = points[:, 1]
            y_norm = (y - y.min()) / (y.max() - y.min() + 1e-10)
            colors = self._colormap_turbo(y_norm)
            pcd.colors = o3d.utility.Vector3dVector(colors)

        elif self.color_mode == "uniform":
            c = self._color_edit.color_value
            colors = np.full((len(points), 3), [c.red, c.green, c.blue])
            pcd.colors = o3d.utility.Vector3dVector(colors)

        # Update geometry on scene
        self._scene.scene.remove_geometry(self.geometry_name)
        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        mat.point_size = self.point_size
        self._scene.scene.add_geometry(self.geometry_name, pcd, mat)

    @staticmethod
    def _colormap_turbo(values):
        """Turbo-like colormap for [0..1] array."""
        colors = np.zeros((len(values), 3))
        # Red
        colors[:, 0] = np.clip(
            np.where(values < 0.25, 0.5 + 2.0 * values,
            np.where(values < 0.5, 1.0,
            np.where(values < 0.75, 1.0 - 4.0 * (values - 0.5),
            0.0))), 0, 1)
        # Green
        colors[:, 1] = np.clip(
            np.where(values < 0.25, 2.0 * values,
            np.where(values < 0.5, 0.5 + 2.0 * (values - 0.25),
            np.where(values < 0.75, 1.0,
            1.0 - 4.0 * (values - 0.75)))), 0, 1)
        # Blue
        colors[:, 2] = np.clip(
            np.where(values < 0.25, 1.0,
            np.where(values < 0.5, 1.0 - 4.0 * (values - 0.25),
            np.where(values < 0.75, 2.0 * (values - 0.5),
            0.5 + 2.0 * (values - 0.75)))), 0, 1)
        return colors

    # ============================
    #       UI Handlers
    # ============================

    def _on_point_size_changed(self, value):
        """Change point size."""
        self.point_size = value
        if self.point_cloud is not None:
            self._apply_color_mode()

    def _on_color_mode_changed(self, name, index):
        """Switch color mode."""
        modes = ["original", "height", "depth", "uniform"]
        if 0 <= index < len(modes):
            self._apply_color_mode(modes[index])

    def _on_uniform_color_changed(self, color):
        """Uniform color changed."""
        if self.color_mode == "uniform":
            self._apply_color_mode("uniform")

    def _on_toggle_axes(self, checked):
        """Show/hide axes."""
        self.axes_visible = checked
        if checked:
            self._add_axes()
        else:
            self._scene.scene.remove_geometry("axes")

    def _on_toggle_bbox(self, checked):
        """Show/hide bounding box."""
        self.bbox_visible = checked
        if checked:
            self._add_bbox()
        else:
            self._scene.scene.remove_geometry("bbox")

    def _on_toggle_grid(self, checked):
        """Show/hide grid."""
        self.grid_visible = checked
        if checked:
            self._add_grid()
        else:
            self._scene.scene.remove_geometry("grid")

    def _on_toggle_normals(self, checked):
        """Show/hide normals."""
        self.normals_visible = checked
        if self.point_cloud is not None:
            if checked:
                if not self.point_cloud.has_normals():
                    self.point_cloud.estimate_normals(
                        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                    print("[INFO] Normals estimated")
            self._apply_color_mode()

    def _on_toggle_bg(self, checked):
        """Toggle background."""
        self.is_dark_bg = checked
        self._apply_background()

    def _apply_background(self):
        """Apply background color."""
        if self.is_dark_bg:
            self._scene.scene.set_background([0.12, 0.12, 0.15, 1.0])
        else:
            self._scene.scene.set_background([0.95, 0.95, 0.95, 1.0])

    def _on_reset_view(self):
        """Reset camera view."""
        bounds = self._scene.scene.bounding_box
        if bounds.volume() > 0:
            self._scene.setup_camera(60.0, bounds, bounds.get_center())

    def _on_save_screenshot(self):
        """Save screenshot."""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"screenshot_{timestamp}.png"

        def _on_image(image):
            o3d.io.write_image(filename, image)
            print(f"[OK] Screenshot saved: {filename}")

        self._scene.scene.scene.render_to_image(_on_image)

    def _on_about(self):
        """About dialog."""
        dlg = gui.Dialog("About")
        em = self.window.theme.font_size
        layout = gui.Vert(0, gui.Margins(em, em, em, em))
        layout.add_child(gui.Label("PLY Viewer v1.0"))
        layout.add_child(gui.Label(""))
        layout.add_child(gui.Label("Point Cloud & Mesh Viewer"))
        layout.add_child(gui.Label("with convenient controls"))
        layout.add_child(gui.Label(""))
        layout.add_child(gui.Label("Mouse: rotate / zoom / pan"))
        layout.add_child(gui.Label("H - hotkeys help in console"))
        layout.add_child(gui.Label(""))

        btn_ok = gui.Button("OK")
        btn_ok.set_on_clicked(self._on_about_ok)
        layout.add_child(btn_ok)

        dlg.add_child(layout)
        self.window.show_dialog(dlg)

    def _on_about_ok(self):
        self.window.close_dialog()

    # ============================
    #     Helper Geometry
    # ============================

    def _add_axes(self):
        """Add coordinate axes."""
        try:
            self._scene.scene.remove_geometry("axes")
        except Exception:
            pass

        bounds = self._scene.scene.bounding_box
        size = max(bounds.get_extent()) * 0.3 if bounds.volume() > 0 else 1.0
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=bounds.get_center() if bounds.volume() > 0 else [0, 0, 0])
        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        self._scene.scene.add_geometry("axes", axes, mat)

    def _add_bbox(self):
        """Add bounding box."""
        try:
            self._scene.scene.remove_geometry("bbox")
        except Exception:
            pass

        geom = self.point_cloud if self.point_cloud else self.mesh
        if geom is None:
            return

        bbox = geom.get_axis_aligned_bounding_box()
        bbox.color = (1.0, 1.0, 0.0)
        lines = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(bbox)
        lines.paint_uniform_color([1.0, 1.0, 0.0])

        mat = rendering.MaterialRecord()
        mat.shader = "unlitLine"
        mat.line_width = 2.0
        self._scene.scene.add_geometry("bbox", lines, mat)

    def _add_grid(self):
        """Add grid on XY plane."""
        try:
            self._scene.scene.remove_geometry("grid")
        except Exception:
            pass

        bounds = self._scene.scene.bounding_box
        if bounds.volume() <= 0:
            return

        extent = bounds.get_extent()
        center = bounds.get_center()
        size = max(extent[0], extent[1]) * 1.5
        step = size / 20.0

        points = []
        lines = []
        idx = 0

        half = size / 2
        z_min = bounds.get_min_bound()[2]
        n_lines = int(size / step) + 1
        for i in range(n_lines):
            y = center[1] - half + i * step
            points.append([center[0] - half, y, z_min])
            points.append([center[0] + half, y, z_min])
            lines.append([idx, idx + 1])
            idx += 2

        for i in range(n_lines):
            x = center[0] - half + i * step
            points.append([x, center[1] - half, z_min])
            points.append([x, center[1] + half, z_min])
            lines.append([idx, idx + 1])
            idx += 2

        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(points)
        ls.lines = o3d.utility.Vector2iVector(lines)
        ls.paint_uniform_color([0.3, 0.3, 0.3])

        mat = rendering.MaterialRecord()
        mat.shader = "unlitLine"
        mat.line_width = 1.0
        self._scene.scene.add_geometry("grid", ls, mat)

    def _remove_helpers(self):
        """Remove helper geometry."""
        for name in ["axes", "bbox", "grid"]:
            try:
                self._scene.scene.remove_geometry(name)
            except Exception:
                pass

    # ============================
    #        Hotkeys
    # ============================

    def _on_key(self, event):
        """Handle keyboard events."""
        if event.type != gui.KeyEvent.DOWN:
            return gui.Widget.EventCallbackResult.IGNORED

        key = event.key

        # O - open file
        if key == ord('o') or key == ord('O'):
            self._on_menu_open()
            return gui.Widget.EventCallbackResult.HANDLED

        # R - reset view
        if key == ord('r') or key == ord('R'):
            self._on_reset_view()
            return gui.Widget.EventCallbackResult.HANDLED

        # + - increase point size
        if key == ord('+') or key == ord('='):
            self.point_size = min(self.point_size + 1.0, 20.0)
            self._slider_point_size.double_value = self.point_size
            self._apply_color_mode()
            return gui.Widget.EventCallbackResult.HANDLED

        # - - decrease point size
        if key == ord('-') or key == ord('_'):
            self.point_size = max(self.point_size - 1.0, 1.0)
            self._slider_point_size.double_value = self.point_size
            self._apply_color_mode()
            return gui.Widget.EventCallbackResult.HANDLED

        # 1-4 - color modes
        if key == ord('1'):
            self._color_combo.selected_index = 0
            self._apply_color_mode("original")
            return gui.Widget.EventCallbackResult.HANDLED
        if key == ord('2'):
            self._color_combo.selected_index = 1
            self._apply_color_mode("height")
            return gui.Widget.EventCallbackResult.HANDLED
        if key == ord('3'):
            self._color_combo.selected_index = 2
            self._apply_color_mode("depth")
            return gui.Widget.EventCallbackResult.HANDLED
        if key == ord('4'):
            self._color_combo.selected_index = 3
            self._apply_color_mode("uniform")
            return gui.Widget.EventCallbackResult.HANDLED

        # B - background
        if key == ord('b') or key == ord('B'):
            self.is_dark_bg = not self.is_dark_bg
            self._cb_dark_bg.checked = self.is_dark_bg
            self._apply_background()
            return gui.Widget.EventCallbackResult.HANDLED

        # A - axes
        if key == ord('a') or key == ord('A'):
            self.axes_visible = not self.axes_visible
            self._cb_axes.checked = self.axes_visible
            self._on_toggle_axes(self.axes_visible)
            return gui.Widget.EventCallbackResult.HANDLED

        # X - bbox
        if key == ord('x') or key == ord('X'):
            self.bbox_visible = not self.bbox_visible
            self._cb_bbox.checked = self.bbox_visible
            self._on_toggle_bbox(self.bbox_visible)
            return gui.Widget.EventCallbackResult.HANDLED

        # N - normals
        if key == ord('n') or key == ord('N'):
            self.normals_visible = not self.normals_visible
            self._cb_normals.checked = self.normals_visible
            self._on_toggle_normals(self.normals_visible)
            return gui.Widget.EventCallbackResult.HANDLED

        # G - grid
        if key == ord('g') or key == ord('G'):
            self.grid_visible = not self.grid_visible
            self._cb_grid.checked = self.grid_visible
            self._on_toggle_grid(self.grid_visible)
            return gui.Widget.EventCallbackResult.HANDLED

        # I - info to console
        if key == ord('i') or key == ord('I'):
            self._print_info()
            return gui.Widget.EventCallbackResult.HANDLED

        # P - screenshot
        if key == ord('p') or key == ord('P'):
            self._on_save_screenshot()
            return gui.Widget.EventCallbackResult.HANDLED

        # H - help
        if key == ord('h') or key == ord('H'):
            self._print_help()
            return gui.Widget.EventCallbackResult.HANDLED

        # Q - quit
        if key == ord('q') or key == ord('Q'):
            self._on_close()
            return gui.Widget.EventCallbackResult.HANDLED

        return gui.Widget.EventCallbackResult.IGNORED

    # ============================
    #      Help & Info
    # ============================

    def _print_help(self):
        """Print help to console."""
        print("""
+==================================================+
|             PLY Viewer - Help                    |
+==================================================+
|  MOUSE:                                          |
|    Left button    - rotate                       |
|    Mouse wheel    - zoom                         |
|    Middle button  - pan                          |
|    Right button   - rotate around Z              |
+--------------------------------------------------+
|  HOTKEYS:                                        |
|    O   - open file                               |
|    R   - reset view                              |
|    +/- - point size                              |
|    1   - original colors                         |
|    2   - height map (Z)                          |
|    3   - depth map (Y)                           |
|    4   - uniform color                           |
|    B   - toggle background                       |
|    A   - coordinate axes                         |
|    X   - bounding box                            |
|    G   - grid                                    |
|    N   - normals                                 |
|    I   - model info                              |
|    P   - screenshot                              |
|    H   - this help                               |
|    Q   - quit                                    |
+==================================================+
""")

    def _print_info(self):
        """Print detailed info to console."""
        print("\n" + "=" * 50)
        print("  Model Info")
        print("=" * 50)
        if self.point_cloud is not None:
            pts = np.asarray(self.point_cloud.points)
            print(f"  Type: Point Cloud")
            print(f"  Points: {len(pts):,}")
            if len(pts) > 0:
                mn = pts.min(axis=0)
                mx = pts.max(axis=0)
                print(f"  X: [{mn[0]:.4f} .. {mx[0]:.4f}]")
                print(f"  Y: [{mn[1]:.4f} .. {mx[1]:.4f}]")
                print(f"  Z: [{mn[2]:.4f} .. {mx[2]:.4f}]")
                print(f"  Size: {mx[0]-mn[0]:.3f} x {mx[1]-mn[1]:.3f} x {mx[2]-mn[2]:.3f}")
            print(f"  Colors: {'Yes' if self.point_cloud.has_colors() else 'No'}")
            print(f"  Normals: {'Yes' if self.point_cloud.has_normals() else 'No'}")
        elif self.mesh is not None:
            verts = np.asarray(self.mesh.vertices)
            tris = np.asarray(self.mesh.triangles)
            print(f"  Type: Triangle Mesh")
            print(f"  Vertices: {len(verts):,}")
            print(f"  Triangles: {len(tris):,}")
            if len(verts) > 0:
                mn = verts.min(axis=0)
                mx = verts.max(axis=0)
                print(f"  X: [{mn[0]:.4f} .. {mx[0]:.4f}]")
                print(f"  Y: [{mn[1]:.4f} .. {mx[1]:.4f}]")
                print(f"  Z: [{mn[2]:.4f} .. {mx[2]:.4f}]")
        else:
            print("  No model loaded")
        print("=" * 50 + "\n")

    def _show_message(self, title, message):
        """Show message dialog."""
        dlg = gui.Dialog(title)
        em = self.window.theme.font_size
        layout = gui.Vert(0, gui.Margins(em, em, em, em))
        layout.add_child(gui.Label(message))
        layout.add_child(gui.Label(""))
        btn = gui.Button("OK")
        btn.set_on_clicked(lambda: self.window.close_dialog())
        layout.add_child(btn)
        dlg.add_child(layout)
        self.window.show_dialog(dlg)

    # ============================
    #         Run
    # ============================

    def run(self):
        """Run the application."""
        print("[PLY Viewer] Starting... Press H for help.")
        self.app.run()


def main():
    """Entry point."""
    file_path = None

    # Check command line arguments
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
        if not os.path.isfile(file_path):
            print(f"[ERROR] File not found: {file_path}")
            file_path = None
    else:
        # Try to find .ply file in current directory
        ply_files = [f for f in os.listdir('.') if f.endswith('.ply')]
        if not ply_files:
            # Search in parent directory
            parent = os.path.dirname(os.path.abspath(__file__))
            parent_dir = os.path.dirname(parent)
            ply_files_parent = [os.path.join(parent_dir, f)
                                for f in os.listdir(parent_dir) if f.endswith('.ply')]
            if ply_files_parent:
                file_path = ply_files_parent[0]
                print(f"[INFO] Found PLY file: {file_path}")
        else:
            file_path = ply_files[0]
            print(f"[INFO] Found PLY file: {file_path}")

    viewer = PLYViewer(file_path)
    viewer.run()


if __name__ == "__main__":
    main()
