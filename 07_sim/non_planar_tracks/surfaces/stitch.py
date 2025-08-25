import bpy
import bmesh
import json
import math
import sys
import os
import numpy as np
from mathutils import Vector
import matplotlib.pyplot as plt

# Add current directory and parent directory to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, current_dir)
sys.path.insert(0, parent_dir)

# Import track classes (adjust import paths if needed)
from off_camber_bank import OffCamberBankedTrack
from on_camber_bank import OnCamberBankedTrack
from single_bump import SingleBumpTrack
from turning import TurningTrack

def extract_centerline_from_track(track_instance, track_type):
    """Extract centerline points from any track type"""
    centerline_points = []
    
    if track_type == "SingleBumpTrack":
        # For bump tracks, centerline is just (0, y, height)
        total_length = track_instance.flat_length + track_instance.bump_length + track_instance.flat_length
        segments = int(total_length * track_instance.verts_per_meter)
        dy = total_length / (segments - 1)
        
        for i in range(segments):
            y = i * dy
            x = 0.0  # centerline at x=0
            # Simple height calculation
            if y < track_instance.flat_length:
                z = 0.0
            elif y < track_instance.flat_length + track_instance.bump_length:
                bump_y = y - track_instance.flat_length
                t = bump_y / track_instance.bump_length
                z = track_instance.bump_height * 0.5 * (1 - math.cos(2 * math.pi * t))
            else:
                z = 0.0
            centerline_points.append([x, y, z])
    
    elif track_type in ["OffCamberBankedTrack", "OnCamberBankedTrack"]:
        # For banked tracks, use the get_center_and_direction logic
        total_rows = track_instance.segments_straight + track_instance.segments_arc + track_instance.segments_straight
        is_left = track_instance.turn_direction.upper() == 'LEFT'
        sign = 1.0 if is_left else -1.0
        
        for i in range(total_rows):
            # First Straight
            if i < track_instance.segments_straight:
                t = i / (track_instance.segments_straight - 1)
                cx = 0.0
                cy = t * track_instance.straight_length
            
            # Arc
            elif i < track_instance.segments_straight + track_instance.segments_arc:
                j = i - track_instance.segments_straight
                t = j / (track_instance.segments_arc - 1)
                arc_angle_rad = math.radians(track_instance.arc_angle_deg)

                if is_left:
                    a_start = math.pi
                    a_end = math.pi - arc_angle_rad
                    center_x = track_instance.radius
                else:
                    a_start = 0.0
                    a_end = arc_angle_rad
                    center_x = -track_instance.radius

                a = a_start + t * (a_end - a_start)
                center_y = track_instance.straight_length

                cx = center_x + track_instance.radius * math.cos(a)
                cy = center_y + track_instance.radius * math.sin(a)
            
            # Second Straight
            else:
                j = i - (track_instance.segments_straight + track_instance.segments_arc)
                t = j / (track_instance.segments_straight - 1)

                arc_angle_rad = math.radians(track_instance.arc_angle_deg)
                if is_left:
                    a_end = math.pi - arc_angle_rad
                    center_x = track_instance.radius
                else:
                    a_end = arc_angle_rad
                    center_x = -track_instance.radius

                center_y = track_instance.straight_length
                cx_arc_end = center_x + track_instance.radius * math.cos(a_end)
                cy_arc_end = center_y + track_instance.radius * math.sin(a_end)

                dx_arc_end = math.cos(a_end + sign * math.pi / 2)
                dy_arc_end = math.sin(a_end + sign * math.pi / 2)

                cx = cx_arc_end - t * track_instance.straight_length * dx_arc_end
                cy = cy_arc_end - t * track_instance.straight_length * dy_arc_end
            
            centerline_points.append([cx, cy, 0.0])  # Z=0 for banked tracks centerline

            

            
    
    return np.array(centerline_points)

def main():
    # -----------------------------
    # Clear the scene
    # -----------------------------
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    
    # -----------------------------
    # Load the track configuration JSON file.
    # (Adjust the path if needed)
    # -----------------------------
    json_path = os.path.join(parent_dir, "track_config", "track1.json")
    with open(json_path, "r") as f:
        params = json.load(f)
    
    # -----------------------------
    # Mapping from JSON track_type to track class.
    # -----------------------------
    track_classes = {
        "OffCamberBankedTrack": OffCamberBankedTrack,  
        "OnCamberBankedTrack": OnCamberBankedTrack,
        "SingleBumpTrack": SingleBumpTrack,
        "TurningTrack": TurningTrack,
    }
    
    # -----------------------------
    # Initialize starting parameters.
    # -----------------------------
    current_start_pos = (0, 0, 0)
    current_start_orient = (0.0, 0.0, 0.0)  # (roll, pitch, yaw) in radians
    
    # Initialize complete centerline
    complete_centerline = []
    
    # -----------------------------
    # Iterate over track sections in the JSON configuration.
    # -----------------------------
    for section_name, track_params in params.items():
        # Extract track_type from the current section.
        track_type = track_params.get("track_type")
        if track_type not in track_classes:
            print(f"Skipping section {section_name} due to unknown track_type: {track_type}")
            continue
        
        track_class = track_classes[track_type]
        
        # Create the track instance with the current starting position and orientation.
        track_instance = track_class(start_position=current_start_pos, start_orientation=current_start_orient)
        
        # Update track instance parameters from the JSON (skip track_type).
        for key, value in track_params.items():
            if key != "track_type":
                setattr(track_instance, key, value)
        
        # Create the track (this adds a new mesh object to the scene).
        track_instance.create_track()
        
        # Extract centerline from this track section
        local_centerline = extract_centerline_from_track(track_instance, track_type)
        
        # Transform centerline to global coordinates
        import mathutils
        start_pos = mathutils.Vector(track_instance.start_position)
        start_rot = mathutils.Euler(track_instance.start_orientation, 'XYZ').to_matrix().to_4x4()
        
        for point in local_centerline:
            local_coord = mathutils.Vector(point)
            global_coord = start_rot @ local_coord + start_pos
            complete_centerline.append([global_coord.x, global_coord.y, global_coord.z])
        
        # Update the starting parameters for the next track using the final position/orientation of the current track.
        current_start_pos = track_instance.final_position
        current_start_orient = (
            track_instance.final_orientation[0],
            track_instance.final_orientation[1],
            track_instance.final_orientation[2] + math.pi / 2
        )
    
    # Save centerline to CSV
    complete_centerline = np.array(complete_centerline)

    #plot centerline for verification
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(complete_centerline[:,0], complete_centerline[:,1], complete_centerline[:,2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Stitched Track Centerline')
    plt.show()  




    centerline_path = os.path.join(parent_dir, "track_centerline.csv")
    np.savetxt(centerline_path, complete_centerline, delimiter=",", header="x,y,z", comments="")
    print(f"Saved centerline to: {centerline_path}")
    
    # -----------------------------
    # (Optional) Join all tracks into one object.
    # -----------------------------
    bpy.ops.object.select_all(action='DESELECT')
    for obj in bpy.data.objects:
        obj.select_set(True)
    if bpy.data.objects:
        bpy.context.view_layer.objects.active = bpy.data.objects[0]
    bpy.ops.object.join()
    
    # -----------------------------
    # Export the stitched tracks as a single OBJ file.
    # -----------------------------
    export_path = os.path.join(parent_dir, "stitched_tracks.obj")  # Export to parent directory
    bpy.ops.export_scene.obj(filepath=export_path, use_selection=True)
    print("Exported stitched tracks to", export_path)

if __name__ == "__main__":
    main()
