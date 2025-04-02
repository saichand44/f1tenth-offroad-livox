import bpy
import bmesh
import math
import mathutils

class OffCamberBankedTrack:
    def __init__(self, start_position=(0.0, 0.0, 0.0), start_orientation=(0.0, 0.0, 0.0)):
        # -------------------------------
        # PARAMETERS
        # -------------------------------
        self.track_width = 5.0             # Total width of the track
        self.straight_length = 10.0         # Length of each straight section
        self.radius = 20.0                # Centerline radius of the arc
        self.arc_angle_deg = 120.0          # Arc angle in degrees
        self.turn_direction = 'RIGHT'       # 'LEFT' or 'RIGHT'
        self.segments_straight = 10        # Resolution for straight sections
        self.segments_arc = 20             # Resolution for arc section

        # Banking params
        self.bank_width = 5.0              # Portion of track width to be banked (outer side)
        self.bank_height = 10.0            # Max elevation at outer edge

        # Additional parameter for mesh resolution
        self.num_width_divs = 30  # higher = smoother bank

        # Starting transformation parameters
        self.start_position = start_position
        self.start_orientation = start_orientation  # (roll, pitch, yaw) in radians

        # These will store the final position and orientation at the end of the mesh
        self.final_position = None
        self.final_orientation = None

    def create_track(self, export_filepath=None):
        # -------------------------------
        # HELPER FUNCTIONS
        # -------------------------------
        def normalize(vx, vy):
            length = math.hypot(vx, vy)
            return (vx / length, vy / length) if length > 1e-9 else (0.0, 0.0)

        def smoothstep(t):
            return 0.5 * (1 - math.cos(math.pi * t))  # cosine easing in [0,1]

        def get_center_and_direction(i):
            total_rows = self.segments_straight + self.segments_arc + self.segments_straight
            is_left = self.turn_direction.upper() == 'LEFT'
            sign = 1.0 if is_left else -1.0

            # First Straight
            if i < self.segments_straight:
                t = i / (self.segments_straight - 1)
                cx = 0.0
                cy = t * self.straight_length
                dx = 0.0
                dy = 1.0
                return (cx, cy), (dx, dy), 'straight', 0.0

            # Arc
            elif i < self.segments_straight + self.segments_arc:
                j = i - self.segments_straight
                t = j / (self.segments_arc - 1)
                arc_angle_rad = math.radians(self.arc_angle_deg)

                if is_left:
                    a_start = math.pi
                    a_end = math.pi - arc_angle_rad
                    center_x = self.radius
                else:
                    a_start = 0.0
                    a_end = arc_angle_rad
                    center_x = -self.radius

                a = a_start + t * (a_end - a_start)
                center_y = self.straight_length

                cx = center_x + self.radius * math.cos(a)
                cy = center_y + self.radius * math.sin(a)
                dx = math.cos(a + sign * math.pi / 2)
                dy = math.sin(a + sign * math.pi / 2)
                return (cx, cy), (dx, dy), 'arc', t

            # Second Straight
            else:
                j = i - (self.segments_straight + self.segments_arc)
                t = j / (self.segments_straight - 1)

                arc_angle_rad = math.radians(self.arc_angle_deg)
                if is_left:
                    a_end = math.pi - arc_angle_rad
                    center_x = self.radius
                else:
                    a_end = arc_angle_rad
                    center_x = -self.radius

                center_y = self.straight_length
                cx_arc_end = center_x + self.radius * math.cos(a_end)
                cy_arc_end = center_y + self.radius * math.sin(a_end)

                dx_arc_end = math.cos(a_end + sign * math.pi / 2)
                dy_arc_end = math.sin(a_end + sign * math.pi / 2)

                cx = cx_arc_end - t * self.straight_length * dx_arc_end
                cy = cy_arc_end - t * self.straight_length * dy_arc_end
                dx = dx_arc_end
                dy = dy_arc_end
                return (cx, cy), (dx, dy), 'straight', 0.0

        # -------------------------------
        # Prepare transformation from start parameters
        # -------------------------------
        start_pos = mathutils.Vector(self.start_position)
        start_rot = mathutils.Euler(self.start_orientation, 'XYZ').to_matrix().to_4x4()

        # -------------------------------
        # BUILD MESH
        # -------------------------------
        bm = bmesh.new()
        row_points = []

        total_rows = self.segments_straight + self.segments_arc + self.segments_straight
        is_left = self.turn_direction.upper() == 'LEFT'
        sign = 1.0 if is_left else -1.0

        half_width = self.track_width / 2
        width_step = self.track_width / (self.num_width_divs - 1)

        for i in range(total_rows):
            (cx, cy), (dx, dy), section_type, arc_t = get_center_and_direction(i)
            dx, dy = normalize(dx, dy)
            nx, ny = -dy * sign, dx * sign

            row_verts = []
            for j in range(self.num_width_divs):
                offset = -half_width + j * width_step
                # Local coordinates before transformation
                local_x = cx + nx * offset
                local_y = cy + ny * offset

                # Elevation (z) calculation remains in local coordinates
                z = 0.0
                if section_type == 'arc':
                    flat_edge = half_width - self.bank_width
                    if offset > flat_edge:
                        bank_offset = offset - flat_edge
                        t_width = bank_offset / self.bank_width  # [0, 1] across banked portion
                        z = self.bank_height * smoothstep(t_width) * smoothstep(arc_t) * smoothstep(1 - arc_t)
                local_coord = mathutils.Vector((local_x, local_y, z))
                # Transform to global coordinates using start position and orientation
                global_coord = start_rot @ local_coord + start_pos

                v = bm.verts.new((global_coord.x, global_coord.y, global_coord.z))
                row_verts.append(v)

            row_points.append(row_verts)

        # -------------------------------
        # CREATE FACES
        # -------------------------------
        for i in range(total_rows - 1):
            for j in range(self.num_width_divs - 1):
                v1 = row_points[i][j]
                v2 = row_points[i][j + 1]
                v3 = row_points[i + 1][j + 1]
                v4 = row_points[i + 1][j]
                bm.faces.new([v1, v2, v3, v4])

        # -------------------------------
        # FINALIZE OBJECT
        # -------------------------------
        mesh = bpy.data.meshes.new("OffCamberBankedTrack")
        obj = bpy.data.objects.new("OffCamberBankedTrack", mesh)
        bpy.context.collection.objects.link(obj)
        bm.to_mesh(mesh)
        bm.free()

        # -------------------------------
        # Calculate final position and orientation in global space
        # -------------------------------
        final_index = total_rows - 1
        (final_center, final_direction, _, _) = get_center_and_direction(final_index)
        # final_center is computed as the midline (midpoint along track width)
        local_mid = mathutils.Vector((final_center[0], final_center[1], 0.0))
        global_mid = start_rot @ local_mid + start_pos

        # Compute local yaw from the direction vector and add the start yaw
        local_yaw = math.atan2(final_direction[1], final_direction[0])
        global_yaw = self.start_orientation[2] + local_yaw

        # Use the start roll and pitch for the final orientation (assuming the track remains flat)
        final_orient = (self.start_orientation[0], self.start_orientation[1], global_yaw)

        # Store in class members for further chaining of track sections
        self.final_position = global_mid
        self.final_orientation = final_orient

        print("Final position (x, y, z):", self.final_position)
        print("Final orientation (roll, pitch, yaw):", self.final_orientation)

        # Export if a filepath is provided
        if export_filepath:
            bpy.ops.object.select_all(action='DESELECT')
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj
            bpy.ops.export_scene.obj(filepath=export_filepath, use_selection=True)

# If running this module directly, create the track with custom start parameters and export if desired
if __name__ == "__main__":
    export_path = "off_camber_banked_track.obj"
    # Example: Start at position (10, 5, 0) with orientation (roll=0, pitch=0, yaw=pi/4)
    track = OffCamberBankedTrack(start_position=(10.0, 5.0, 0.0), start_orientation=(0.0, 0.0, 0.0))
    track.create_track(export_filepath=export_path)
