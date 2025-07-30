#!/usr/bin/env python3
"""
Script to process ROS 2 topic echo files from two distance sensors and convert to CSV format.
"""

import re
import argparse
from datetime import datetime, timedelta
import math

global base_timestamp
base_timestamp = datetime(2025, 7, 24, 18, 20, 33)

def parse_sensor_file(filepath):
    """Parse a sensor file and extract timestamp and range data."""
    data = []
    
    with open(filepath, 'r') as file:
        content = file.read()
    
    # Split by --- separator
    messages = content.split('---')
    
    for message in messages:
        if not message.strip():
            continue
            
        # Extract timestamp
        sec_match = re.search(r'sec:\s*(\d+)', message)
        
        # Extract range - more precise pattern for floating point numbers
        # First try to match .inf or .nan explicitly
        inf_nan_match = re.search(r'^range:\s*(\.inf|\.nan|inf|nan)\s*$', message, re.MULTILINE | re.IGNORECASE)
        if inf_nan_match:
            # Skip infinity and NaN values
            range_str = "-1"
        else:
            # Then try to match valid floating point numbers
            range_match = re.search(r'^range:\s*(-?\d+\.?\d*(?:[eE][+-]?\d+)?)\s*$', message, re.MULTILINE)
            range_str = range_match.group(1)

        sec = int(sec_match.group(1))
        
        
        try:
            range_val = float(range_str)
                
            data.append({
                'timestamp': sec,
                'range': range_val
            })
            
        except ValueError:
            # Skip if we can't convert to float
            print(f"Warning: Could not parse range value: '{range_str}'")
            continue
    
    return data

def find_valid_ranges(sensor0_data, sensor1_data):
    """
    Find sets of valid (non-infinity) range values from both sensors.
    Groups data by time windows where both sensors have valid readings.
    """
    valid_sets = []
    
    # Create time windows and find overlapping valid readings
    i, j = 0, 0
    
    # Sync timestamps
    while i < len(sensor0_data) and j < len(sensor1_data):
        if sensor0_data[i]['timestamp'] < sensor1_data[j]['timestamp']:
            i += 1
        elif sensor0_data[i]['timestamp'] > sensor1_data[j]['timestamp']:
            j += 1
        else:
            break


    while i < len(sensor0_data) and j < len(sensor1_data):
        if sensor0_data[i]['range'] >= 0 and sensor1_data[j]['range'] >= 0:
                # Advance while both sensors have valid readings in this time window
                while (i < len(sensor0_data) and 
                    j < len(sensor1_data) and
                    sensor0_data[i]['range'] >= 0 and 
                    sensor1_data[j]['range'] >= 0):
                    i += 1
                    j += 1
                
                # Take the last valid readings from this sequence

                valid_sets.append({
                    'sensor0': sensor0_data[i-1],
                    'sensor1': sensor1_data[j-1],
                    'timestamp': max(sensor0_data[i-1]['timestamp'], sensor1_data[j-1]['timestamp'])
                })
        else:
            i += 1
            j += 1
    
    
    return valid_sets

def format_timestamp(base_dt, first_ros_timestamp, current_ros_timestamp):
    """Convert ROS timestamp to formatted string starting from base_timestamp."""
    # Base time: 2025-07-24_18-20-33
    
    
    # Calculate elapsed time from first measurement
    elapsed_seconds = current_ros_timestamp - first_ros_timestamp
    
    # Add elapsed time to base
    result_dt = base_dt + timedelta(seconds=elapsed_seconds)
    
    return result_dt.strftime("%Y-%m-%d_%H-%M-%S")

def get_speed_and_acc_scale(run_number):
    """Calculate SpeedAndAccScale based on run number."""
    speed_array = [0.2, 0.4, 0.8, 1.0]
    speed_cycle = run_number // 4  # Floor division
    current_speed = speed_array[speed_cycle % 4]  # Should be % 4, not % 5
    return current_speed

def get_start_position(run_number):
    """Calculate StartPosition based on run number."""
    position_array = ["home", "right_test_start", "left_test_start", "max_left_test"]
    start_position = position_array[run_number % 4]
    return start_position

def process_sensor_files(sensor0_file, sensor1_file, output_file):
    """Main processing function to convert sensor files to CSV."""
    
    print(f"Processing {sensor0_file} and {sensor1_file}...")
    
    # Parse both sensor files
    sensor0_data = parse_sensor_file(sensor0_file)
    sensor1_data = parse_sensor_file(sensor1_file)
    
    print(f"Parsed {len(sensor0_data)} readings from sensor 0")
    print(f"Parsed {len(sensor1_data)} readings from sensor 1")
    
    # Find valid range sets
    valid_sets = find_valid_ranges(sensor0_data, sensor1_data)
    
    print(f"Found {len(valid_sets)} valid measurement sets")
    
    # Write to CSV
    with open(output_file, 'w') as csv_file:
        # Write header
        csv_file.write("Timestamp,SpeedAndAccScale,StartPosition,Sensor0_Dist,Sensor1_Dist\n")
        if valid_sets:

            first_timestamp = valid_sets[0]['timestamp']
            # Write data
            for run, data_set in enumerate(valid_sets):
                timestamp = format_timestamp(base_timestamp, first_timestamp, data_set['timestamp'])
                speed_scale = get_speed_and_acc_scale(run)
                start_pos = get_start_position(run)
                sensor0_dist = data_set['sensor0']['range']
                sensor1_dist = data_set['sensor1']['range']
                
                csv_file.write(f"{timestamp},{speed_scale:.6f},{start_pos},{sensor0_dist:.6f},{sensor1_dist:.6f}\n")
    
    print(f"Output written to {output_file}")
    print(f"Processed {len(valid_sets)} runs")

def main():
    parser = argparse.ArgumentParser(description='Convert ROS 2 sensor topic files to CSV format')
    parser.add_argument('sensor0_file', help='Path to sensor 0 topic echo file')
    parser.add_argument('sensor1_file', help='Path to sensor 1 topic echo file')
    parser.add_argument('-o', '--output', default='sensor_results.csv', 
                       help='Output CSV file (default: sensor_results.csv)')
    
    args = parser.parse_args()
    
    try:
        process_sensor_files(args.sensor0_file, args.sensor1_file, args.output)
    except FileNotFoundError as e:
        print(f"Error: Could not find file - {e}")
    except Exception as e:
        print(f"Error processing files: {e}")

if __name__ == '__main__':
    main()
