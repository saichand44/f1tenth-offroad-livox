import bpy
import bmesh
import math
import mathutils

class SingleBumpTrack:
    def __init__(self, start_position=(0.0, 0.0, 0.0), start_orientation=(0.0, 0.0, 0.0)):
        # -------------------------------
        # PARAMETERS
        # -------------------------------
        # Flat section parameters
        self.flat_width = 4.0       # meters
        self.flat_length = 2.0      # meters

        # Sinusoidal bump parameters
        self.bump_height = 3.0      # meters
        self.bump_width = 4.0       # meters (same as flat to avoid sudden edge offset)
        self.bump_length = 25.0     # meters

        # Resolution (mesh quality)
        self.verts_x = 40           # Along width
        self.verts_per_meter = 10   # Controls resolution along length

        # Starting transformation parameters
        self.start_position = start_position
        self.start_orientation = start_orientation  # (roll, pitch, yaw) in radians

        # These will store the final position and orientation at the end of the track.
        self.final_position = None
        self.final_orientation = None

    def create_track(self, export_filepath=None):
        # -------------------------------
        # CALCULATED PARAMETERS
        # -------------------------------
        total_length = self.flat_length + self.bump_length + self.flat_length
        verts_y = int(total_length * self.verts_per_meter)
        dx = self.flat_width / (self.verts_x - 1)
        dy = total_length / (verts_y - 1)

        # Prepare starting transformation from the given position and orientation.
        start_pos = mathutils.Vector(self.start_position)
        start_rot = mathutils.Euler(self.start_orientation, 'XYZ').to_matrix().to_4x4()

        # -------------------------------
        # CREATE MESH
        # -------------------------------
        mesh = bpy.data.meshes.new("SingleBumpTrack")
        obj = bpy.data.objects.new("SingleBumpTrack", mesh)
        bpy.context.collection.objects.link(obj)
        bm = bmesh.new()

        # Helper: Calculate height at a given Y using cosine easing.
        def height_at_y(y):
            if y < self.flat_length:
                return 0.0
            elif y < self.flat_length + self.bump_length:
                t = (y - self.flat_length) / self.bump_length  # normalized [0,1]
                # Cosine easing: smooth start and end (zero slope at both ends)
                return self.bump_height * 0.5 * (1 - math.cos(2 * math.pi * t))
            else:
                return 0.0

        # Create vertices.
        # Note: The track is centered along X by subtracting half the flat_width.
        verts = []
        for j in range(verts_y):
            y = j * dy
            z = height_at_y(y)
            for i in range(self.verts_x):
                x = i * dx - self.flat_width / 2.0  # center the track along X
                local_coord = mathutils.Vector((x, y, z))
                global_coord = start_rot @ local_coord + start_pos
                vert = bm.verts.new((global_coord.x, global_coord.y, global_coord.z))
                verts.append(vert)

        bm.verts.ensure_lookup_table()

        # Create faces.
        for j in range(verts_y - 1):
            for i in range(self.verts_x - 1):
                v1 = verts[j * self.verts_x + i]
                v2 = verts[j * self.verts_x + i + 1]
                v3 = verts[(j + 1) * self.verts_x + i + 1]
                v4 = verts[(j + 1) * self.verts_x + i]
                bm.faces.new([v1, v2, v3, v4])

        # Finalize mesh.
        bm.to_mesh(mesh)
        bm.free()

        # -------------------------------
        # Calculate Final Position and Orientation
        # -------------------------------
        # The final point on the track is at local coordinates:
        #   X = 0 (midway along width, since the track is centered),
        #   Y = total_length,
        #   Z = height_at_y(total_length) (which is 0 if the ending flat section is at 0).
        final_local = mathutils.Vector((0, total_length, height_at_y(total_length)))
        final_global = start_rot @ final_local + start_pos

        # The local tangent direction of the track is along the positive Y axis: (0, 1, 0).
        # Its yaw is calculated using atan2; math.atan2(1, 0) equals π/2.
        local_yaw = math.atan2(1, 0)  # π/2
        global_yaw = self.start_orientation[2] + local_yaw
        final_orient = (self.start_orientation[0], self.start_orientation[1], global_yaw)

        # Store the final position and orientation in class members.
        self.final_position = final_global
        self.final_orientation = final_orient

        print("Final position (x, y, z):", self.final_position)
        print("Final orientation (roll, pitch, yaw):", self.final_orientation)

        # Export as OBJ if an export filepath is provided.
        if export_filepath:
            bpy.ops.object.select_all(action='DESELECT')
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj
            bpy.ops.export_scene.obj(filepath=export_filepath, use_selection=True)

# If running this module directly, create the track with custom starting parameters.
if __name__ == "__main__":
    export_path = "bump_track.obj"  # Update the path accordingly.
    # Example: starting at (0, 0, 0) with orientation (roll=0, pitch=0, yaw=0)
    track = SingleBumpTrack(start_position=(0.0, 0.0, 0.0), start_orientation=(0.0, 0.0, 0.0))
    track.create_track(export_filepath=export_path)
