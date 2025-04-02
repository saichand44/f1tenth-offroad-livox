import bpy
import bmesh
import json
import math
from mathutils import Vector

# Import track classes (adjust import paths if needed)
from surfaces.off_camber_bank import OffCamberBankedTrack
from surfaces.on_camber_bank import OnCamberBankedTrack
from surfaces.single_bump import SingleBumpTrack
from surfaces.turning import TurningTrack

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
    json_path = "track_config/track1.json"
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
        
        # Update the starting parameters for the next track using the final position/orientation of the current track.
        current_start_pos = track_instance.final_position
        current_start_orient = (
            track_instance.final_orientation[0],
            track_instance.final_orientation[1],
            track_instance.final_orientation[2] + math.pi / 2
        )
    
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
    export_path = "stitched_tracks.obj"  # Adjust export path if desired.
    bpy.ops.export_scene.obj(filepath=export_path, use_selection=True)
    print("Exported stitched tracks to", export_path)

if __name__ == "__main__":
    main()
