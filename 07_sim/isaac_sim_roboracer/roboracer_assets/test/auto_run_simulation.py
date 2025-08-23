#!/usr/bin/env python3
"""
Automated script to run Isaac Lab simulation for angles 0-20 degrees.
This script automatically sends Ctrl+C when it detects the completion message.
"""

import subprocess
import signal
import time
import os
import sys
from threading import Timer, Thread

def monitor_file_creation(expected_file_path, process, check_interval=1.0):
    """
    Monitor for file creation and send Ctrl+C when the file is created.
    Returns True if file was created and Ctrl+C was sent, False otherwise.
    """
    while process.poll() is None:  # While process is still running
        if os.path.exists(expected_file_path):
            print(f"\n[AUTOMATION] Data file detected: {os.path.basename(expected_file_path)}")
            print("[AUTOMATION] Sending termination signal...")
            try:
                # Try multiple methods to terminate the process
                # Method 1: Send SIGINT (Ctrl+C) to process group
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                time.sleep(2)
                
                # Method 2: If still running, try SIGTERM
                if process.poll() is None:
                    print("[AUTOMATION] SIGINT didn't work, trying SIGTERM...")
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    time.sleep(2)
                
                # Method 3: If still running, try direct process termination
                if process.poll() is None:
                    print("[AUTOMATION] SIGTERM didn't work, trying process.terminate()...")
                    process.terminate()
                    time.sleep(2)
                
                # Method 4: If still running, force kill
                if process.poll() is None:
                    print("[AUTOMATION] terminate() didn't work, trying SIGKILL...")
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    time.sleep(1)
                
                print("[AUTOMATION] Termination signal sent successfully")
                return True
                
            except ProcessLookupError:
                print("[AUTOMATION] Process already terminated")
                return False
            except Exception as e:
                print(f"[AUTOMATION] Error sending termination signal: {e}")
                return False
        time.sleep(check_interval)
    return False

def run_simulation(angle, num_envs=200):
    """
    Run a single simulation with the given angle.
    Returns True if successful, False otherwise.
    """
    # Use absolute path instead of ~ which doesn't expand in subprocess
    script_path = "/home/saichand/ros2_ws/src/f1tenth-offroad-livox/07_sim/isaac_sim_roboracer/roboracer_assets/test/create_robot_scene2.py"
    data_file = f"data{angle}"
    
    # Expected output file path
    test_dir = "/home/saichand/ros2_ws/src/f1tenth-offroad-livox/07_sim/isaac_sim_roboracer/roboracer_assets/test"
    expected_file_path = os.path.join(test_dir, f"{data_file}.npz")
    
    cmd = [
        "./isaaclab.sh", "-p", script_path,
        "--num_envs", str(num_envs),
        "--ground_angle", str(angle),
        "--data_file", data_file
    ]
    
    print(f"\n{'='*60}")
    print(f"Starting simulation for angle: {angle}°")
    print(f"Data file: {data_file}")
    print(f"Expected output: {expected_file_path}")
    print(f"Command: {' '.join(cmd)}")
    print(f"{'='*60}")
    
    # Remove existing file if it exists to avoid false detection
    if os.path.exists(expected_file_path):
        print(f"[AUTOMATION] Removing existing file: {expected_file_path}")
        os.remove(expected_file_path)
    
    try:
        # Start the process
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1,
            preexec_fn=os.setsid  # Create a new process group
        )
        
        # Start file monitoring thread
        file_monitor_thread = Thread(
            target=monitor_file_creation, 
            args=(expected_file_path, process),
            daemon=True
        )
        file_monitor_thread.start()
        
        # Track if we've sent Ctrl+C
        ctrl_c_sent = False
        
        # Read output line by line (for logging purposes)
        while True:
            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break
            if output:
                line = output.strip()
                print(line)
                
                # Check if the file monitoring thread detected the file
                if os.path.exists(expected_file_path) and not ctrl_c_sent:
                    print(f"[AUTOMATION] File detected in main loop, waiting for termination...")
                    ctrl_c_sent = True
                    # Give the file monitor thread time to send the signal
                    time.sleep(3)
                    # If process is still running after signal, break the loop
                    if process.poll() is None:
                        print("[AUTOMATION] Process still running, breaking output loop...")
                        break
        
        # Wait for process to complete with timeout
        try:
            return_code = process.wait(timeout=10)  # Wait up to 10 seconds
        except subprocess.TimeoutExpired:
            print("[AUTOMATION] Process didn't terminate within timeout, force killing...")
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                return_code = process.wait(timeout=5)
            except:
                return_code = -9  # Force killed
        
        # Check if file was created successfully
        if os.path.exists(expected_file_path):
            file_size = os.path.getsize(expected_file_path) / (1024 * 1024)  # MB
            print(f"✓ Simulation completed successfully for angle {angle}°")
            print(f"  Generated file: {os.path.basename(expected_file_path)} ({file_size:.2f} MB)")
            print(f"  Process return code: {return_code}")
            return True
        else:
            print(f"✗ Simulation failed for angle {angle}° - no data file generated")
            print(f"  Process return code: {return_code}")
            return False
            
    except KeyboardInterrupt:
        print(f"\n[AUTOMATION] Interrupted by user for angle {angle}°")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except:
            pass
        return False
    except Exception as e:
        print(f"✗ Error running simulation for angle {angle}°: {e}")
        return False

def main():
    """Main automation function"""
    # Configuration
    angles = list(range(0, 21))  # 0 to 20 degrees
    num_envs = 200
    
    print("Isaac Lab Automated Data Collection")
    print("=" * 50)
    print(f"Angles to run: {angles}")
    print(f"Total simulations: {len(angles)}")
    print(f"Environments per simulation: {num_envs}")
    print(f"Current directory: {os.getcwd()}")
    
    # Check if we're in the right directory
    if not os.path.exists("./isaaclab.sh"):
        print("\nError: isaaclab.sh not found in current directory!")
        print("Please run this script from the IsaacLab directory:")
        print("cd ~/custom-kgs/IsaacLab")
        print("python3 ~/ros2_ws/src/f1tenth-offroad-livox/07_sim/isaac_sim_roboracer/roboracer_assets/test/auto_run_simulation.py")
        sys.exit(1)
    
    # Confirm before starting
    try:
        response = input(f"\nRun {len(angles)} simulations? [y/N]: ").strip().lower()
        if response not in ['y', 'yes']:
            print("Cancelled.")
            sys.exit(0)
    except KeyboardInterrupt:
        print("\nCancelled.")
        sys.exit(0)
    
    # Track results
    successful = 0
    failed = 0
    start_time = time.time()
    
    print(f"\nStarting automation at {time.strftime('%H:%M:%S')}")
    print("Use Ctrl+C to stop the entire automation\n")
    
    try:
        for i, angle in enumerate(angles, 1):
            print(f"\n[{i}/{len(angles)}] Processing angle {angle}°...")
            
            success = run_simulation(angle, num_envs)
            
            if success:
                successful += 1
                print(f"✓ Completed {angle}° ({successful}/{len(angles)} successful so far)")
            else:
                failed += 1
                print(f"✗ Failed {angle}° ({failed} failures so far)")
            
            # Brief pause between simulations
            if i < len(angles):
                print(f"Waiting 3 seconds before next simulation...")
                time.sleep(3)
    
    except KeyboardInterrupt:
        print(f"\n\nAutomation stopped by user after {successful + failed} simulations")
    
    # Final summary
    total_time = time.time() - start_time
    hours = int(total_time // 3600)
    minutes = int((total_time % 3600) // 60)
    seconds = int(total_time % 60)
    
    print(f"\n{'='*60}")
    print(f"AUTOMATION COMPLETE")
    print(f"{'='*60}")
    print(f"Total simulations attempted: {successful + failed}")
    print(f"Successful: {successful}")
    print(f"Failed: {failed}")
    print(f"Duration: {hours}h {minutes}m {seconds}s")
    print(f"End time: {time.strftime('%H:%M:%S')}")
    
    # Check generated files
    test_dir = os.path.expanduser("~/ros2_ws/src/f1tenth-offroad-livox/07_sim/isaac_sim_roboracer/roboracer_assets/test")
    print(f"\nChecking generated files in {test_dir}:")
    
    for angle in angles[:successful + failed]:  # Only check angles we attempted
        filename = f"data{angle}.npz"
        filepath = os.path.join(test_dir, filename)
        if os.path.exists(filepath):
            size_mb = os.path.getsize(filepath) / (1024 * 1024)
            print(f"  ✓ {filename} ({size_mb:.1f} MB)")
        else:
            print(f"  ✗ {filename} (missing)")
    
    if failed == 0 and successful == len(angles):
        print(f"\n🎉 All {successful} simulations completed successfully!")
    elif successful > 0:
        print(f"\n✓ {successful} simulations completed successfully.")
        if failed > 0:
            print(f"⚠️  {failed} simulations failed.")
    else:
        print(f"\n❌ All simulations failed.")
    
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
