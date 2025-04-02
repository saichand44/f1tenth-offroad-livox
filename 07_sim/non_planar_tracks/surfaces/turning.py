import bpy
import bmesh
import math
import mathutils

class TurningTrack:
    def __init__(self, start_position=(0.0, 0.0, 0.0), start_orientation=(0.0, 0.0, 0.0)):
        # -------------------------------
        # 2) PARAMETERS
        # -------------------------------
        self.track_width = 2.0          # Width of the track
        self.straight_length = 5.0      # Length of each straight section
        self.radius = 5.0               # Center-line radius of the arc
        self.arc_angle_deg = 60.0      # Arc angle in degrees
        self.turn_direction = 'LEFT'   # Options: 'LEFT' or 'RIGHT'
        self.segments_straight = 10     # Resolution of straight sections
        self.segments_arc = 20          # Resolution of arc section

        # Starting transformation parameters
        self.start_position = start_position
        self.start_orientation = start_orientation  # (roll, pitch, yaw) in radians

        # These will store the final position and orientation for chaining track sections.
        self.final_position = None
        self.final_orientation = None

    def create_track(self, export_filepath=None):
        # -------------------------------
        # 3) HELPER FUNCTIONS
        # -------------------------------
        def normalize(vx, vy):
            length = math.hypot(vx, vy)
            return (vx / length, vy / length) if length > 1e-9 else (0.0, 0.0)

        def get_center_and_direction(i):
            total_rows = self.segments_straight + self.segments_arc + self.segments_straight

            is_left = self.turn_direction.upper() == 'RIGHT'
            sign = 1.0 if is_left else -1.0

            # First Straight
            if i < self.segments_straight:
                t = i / (self.segments_straight - 1)
                cx = 0.0
                cy = t * self.straight_length
                dx = 0.0
                dy = 1.0
                return (-cx, -cy), (-dx, -dy)

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
                return (-cx, -cy), (-dx, -dy)

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
                return (-cx, -cy), (-dx, -dy)

        # -------------------------------
        # Prepare starting transformation
        # -------------------------------
        start_pos = mathutils.Vector(self.start_position)
        start_rot = mathutils.Euler(self.start_orientation, 'XYZ').to_matrix().to_4x4()

        # -------------------------------
        # 4) BUILD THE TRACK MESH
        # -------------------------------
        bm = bmesh.new()
        row_points = []
        total_rows = self.segments_straight + self.segments_arc + self.segments_straight

        is_left = self.turn_direction.upper() == 'LEFT'
        sign = 1.0 if is_left else -1.0

        for i in range(total_rows):
            (cx, cy), (dx, dy) = get_center_and_direction(i)
            dx, dy = normalize(dx, dy)
            # Compute the normal from the direction vector.
            nx, ny = -dy * sign, dx * sign

            # Compute left and right offsets from the midline.
            left_x = cx + nx * (-self.track_width * 0.5)
            left_y = cy + ny * (-self.track_width * 0.5)
            right_x = cx + nx * (self.track_width * 0.5)
            right_y = cy + ny * (self.track_width * 0.5)

            # Transform local coordinates to global using the starting transformation.
            global_left = start_rot @ mathutils.Vector((left_x, left_y, 0.0)) + start_pos
            global_right = start_rot @ mathutils.Vector((right_x, right_y, 0.0)) + start_pos

            vL = bm.verts.new((global_left.x, global_left.y, global_left.z))
            vR = bm.verts.new((global_right.x, global_right.y, global_right.z))
            row_points.append((vL, vR))

        bm.verts.ensure_lookup_table()

        # -------------------------------
        # 5) CONNECT FACES
        # -------------------------------
        for i in range(total_rows - 1):
            vL1, vR1 = row_points[i]
            vL2, vR2 = row_points[i + 1]
            bm.faces.new([vL1, vR1, vR2, vL2])

        # -------------------------------
        # 6) CREATE MESH OBJECT
        # -------------------------------
        mesh = bpy.data.meshes.new("TurningTrack")
        obj = bpy.data.objects.new("TurningTrack", mesh)
        bpy.context.collection.objects.link(obj)
        bm.to_mesh(mesh)
        bm.free()

        # -------------------------------
        # Calculate Final Position and Orientation
        # -------------------------------
        # Get the last row's midline (local center point)
        (final_center, final_direction) = get_center_and_direction(total_rows - 1)
        final_local = mathutils.Vector((final_center[0], final_center[1], 0.0))
        final_global = start_rot @ final_local + start_pos

        # Calculate local yaw from the direction vector.
        local_yaw = math.atan2(final_direction[1], final_direction[0])
        global_yaw = self.start_orientation[2] + local_yaw
        final_orient = (self.start_orientation[0], self.start_orientation[1], global_yaw)

        self.final_position = final_global
        self.final_orientation = final_orient

        print("Final position (x, y, z):", self.final_position)
        print("Final orientation (roll, pitch, yaw):", self.final_orientation)

        # -------------------------------
        # Optional: Export as OBJ if filepath is provided
        # -------------------------------
        if export_filepath:
            bpy.ops.object.select_all(action='DESELECT')
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj
            bpy.ops.export_scene.obj(filepath=export_filepath, use_selection=True)

# If running this module directly, create the track and export it if desired.
if __name__ == "__main__":
    export_path = "turning_track.obj"  # Update the path accordingly
    # Example: starting at (0, 0, 0) with orientation (roll=0, pitch=0, yaw=0)
    track = TurningTrack(start_position=(0.0, 0.0, 0.0), start_orientation=(0.0, 0.0, 0.0))
    track.create_track(export_filepath=export_path)
