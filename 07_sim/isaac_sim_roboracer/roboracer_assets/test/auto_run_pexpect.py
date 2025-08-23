#!/usr/bin/env python3
"""
Alternative automated script using pexpect for better control signal handling.
This version uses pexpect which can send keyboard input more reliably.
"""

import os
import sys
import time
from threading import Thread

def run_simulation_with_pexpect(angle, num_envs=200):
    """
    Run simulation using pexpect for better signal handling.
    """
    try:
        import pexpect
    except ImportError:
        print("Error: pexpect not installed. Install with: pip install pexpect")
        return False
    
    script_path = "/home/saichand/ros2_ws/src/f1tenth-offroad-livox/07_sim/isaac_sim_roboracer/roboracer_assets/test/create_robot_scene2.py"
    data_file = f"data{angle}"
    test_dir = "/home/saichand/ros2_ws/src/f1tenth-offroad-livox/07_sim/isaac_sim_roboracer/roboracer_assets/test"
    expected_file_path = os.path.join(test_dir, f"{data_file}.npz")
    
    cmd = f"./isaaclab.sh -p {script_path} --num_envs {num_envs} --ground_angle {angle} --data_file {data_file}"
    
    print(f"\n{'='*60}")
    print(f"Starting simulation for angle: {angle}°")
    print(f"Data file: {data_file}")
    print(f"Expected output: {expected_file_path}")
    print(f"Command: {cmd}")
    print(f"{'='*60}")
    
    # Remove existing file if it exists
    if os.path.exists(expected_file_path):
        print(f"[AUTOMATION] Removing existing file: {expected_file_path}")
        os.remove(expected_file_path)
    
    try:
        # Start the process with pexpect
        child = pexpect.spawn(cmd, timeout=None)
        child.logfile_read = sys.stdout.buffer  # Log output to stdout
        
        # Monitor for file creation in a separate thread
        def monitor_file():
            while child.isalive():
                if os.path.exists(expected_file_path):
                    print(f"\n[AUTOMATION] Data file detected: {os.path.basename(expected_file_path)}")
                    print("[AUTOMATION] Sending Ctrl+C...")
                    child.sendintr()  # Send Ctrl+C
                    time.sleep(2)
                    if child.isalive():
                        print("[AUTOMATION] Still alive, sending EOF...")
                        child.sendeof()
                        time.sleep(2)
                    if child.isalive():
                        print("[AUTOMATION] Still alive, terminating...")
                        child.terminate()
                    break
                time.sleep(1)
        
        monitor_thread = Thread(target=monitor_file, daemon=True)
        monitor_thread.start()
        
        # Wait for the process to complete
        child.expect(pexpect.EOF, timeout=1800)  # 30 minute timeout
        
        # Check if file was created
        if os.path.exists(expected_file_path):
            file_size = os.path.getsize(expected_file_path) / (1024 * 1024)
            print(f"\n✓ Simulation completed successfully for angle {angle}°")
            print(f"  Generated file: {os.path.basename(expected_file_path)} ({file_size:.2f} MB)")
            return True
        else:
            print(f"\n✗ Simulation failed for angle {angle}° - no data file generated")
            return False
    
    except pexpect.exceptions.TIMEOUT:
        print(f"\n✗ Simulation timed out for angle {angle}°")
        child.terminate()
        return False
    except Exception as e:
        print(f"\n✗ Error running simulation for angle {angle}°: {e}")
        if 'child' in locals():
            child.terminate()
        return False

def main():
    """Main automation function using pexpect"""
    # Check if pexpect is available
    try:
        import pexpect
    except ImportError:
        print("Error: This script requires pexpect.")
        print("Install it with: pip install pexpect")
        print("Or use the regular auto_run_simulation.py script instead.")
        sys.exit(1)
    
    # Check if we're in the right directory
    if not os.path.exists("./isaaclab.sh"):
        print("Error: Please run this script from the IsaacLab directory:")
        print("cd ~/custom-kgs/IsaacLab")
        sys.exit(1)
    
    angles = list(range(0, 21))  # 0 to 20 degrees
    num_envs = 200
    
    print("Isaac Lab Automated Data Collection (with pexpect)")
    print("=" * 55)
    print(f"Angles to run: {angles}")
    print(f"Total simulations: {len(angles)}")
    print(f"Environments per simulation: {num_envs}")
    
    # Confirm before starting
    try:
        response = input(f"\nRun {len(angles)} simulations? [y/N]: ").strip().lower()
        if response not in ['y', 'yes']:
            print("Cancelled.")
            sys.exit(0)
    except KeyboardInterrupt:
        print("\nCancelled.")
        sys.exit(0)
    
    successful = 0
    failed = 0
    start_time = time.time()
    
    print(f"\nStarting automation at {time.strftime('%H:%M:%S')}")
    
    try:
        for i, angle in enumerate(angles, 1):
            print(f"\n[{i}/{len(angles)}] Processing angle {angle}°...")
            
            success = run_simulation_with_pexpect(angle, num_envs)
            
            if success:
                successful += 1
                print(f"✓ Completed {angle}° ({successful}/{len(angles)} successful so far)")
            else:
                failed += 1
                print(f"✗ Failed {angle}° ({failed} failures so far)")
            
            # Brief pause between simulations
            if i < len(angles):
                print(f"Waiting 5 seconds before next simulation...")
                time.sleep(5)
    
    except KeyboardInterrupt:
        print(f"\n\nAutomation stopped by user after {successful + failed} simulations")
    
    # Final summary
    total_time = time.time() - start_time
    hours = int(total_time // 3600)
    minutes = int((total_time % 3600) // 60)
    
    print(f"\n{'='*60}")
    print(f"AUTOMATION COMPLETE")
    print(f"{'='*60}")
    print(f"Successful: {successful}")
    print(f"Failed: {failed}")
    print(f"Duration: {hours}h {minutes}m")
    
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
