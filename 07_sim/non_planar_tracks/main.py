import bpy
import bmesh
import os
import sys
import json

from surfaces.off_camber_bank import OffCamberBankedTrack
from surfaces.on_camber_bank import OnCamberBankedTrack
from surfaces.single_bump import SingleBumpTrack
from surfaces.turning import TurningTrack 

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

def disable_scene_cleanup():
    # Monkey-patch cleanup functions so subsequent track creations don't clear the scene.
    bpy.ops.object.select_all = lambda action: None
    bpy.ops.object.delete = lambda use_global: None

def main():
    # Clear the scene once before any track creation.
    clear_scene()
    
    # Load the track parameters JSON file.
    with open("./track_config/track1.json", "r") as f:
        params = json.load(f)
    
    # Lookup dictionary mapping JSON keys to track classes.
    track_classes = {
        "OffCamberBankedTrack": OffCamberBankedTrack,
        "OnCamberBankedTrack": OnCamberBankedTrack,
        "SingleBumpTrack": SingleBumpTrack,
        "TurningTrack": TurningTrack,
                    }
    
    first_track = True
    
    # Iterate over each track defined in the JSON file.
    for track_name, track_params in params.items():
        if track_name in track_classes:
            # Instantiate the appropriate track class.
            track_instance = track_classes[track_name]()
            # Update instance parameters from the JSON.
            for key, value in track_params.items():
                setattr(track_instance, key, value)
            
            # If this is the first track, let its cleanup run, then disable cleanup.
            if first_track:
                track_instance.create_track()
                first_track = False
                disable_scene_cleanup()
            else:
                track_instance.create_track()
    
    # Export the entire scene as a single OBJ file.
    export_path = "all_tracks.obj"  # <-- Update to your desired export location.
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.export_scene.obj(filepath=export_path, use_selection=True)
    print("Exported all tracks to", export_path)

if __name__ == "__main__":
    main()