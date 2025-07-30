import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import sys
import os
from pathlib import Path

# --- Script Configuration ---
DEFAULT_FILENAME = 'results.csv'  # Default filename to load if no argument is provided
# Conversion factor for units
MICROMETERS_PER_METER = 1_000_000

def load_and_prepare_data(filepath):
    """
    Loads sensor data, converts it to micrometers, and prepares it for analysis.

    Args:
        filepath (str): The path to the CSV file.

    Returns:
        pandas.DataFrame: A DataFrame with prepared data, or None if file not found.
    """
    try:
        df = pd.read_csv(filepath)
        print(f"Successfully loaded {filepath}")

        data_dir = os.path.dirname(os.path.abspath(filepath))
        
        print(f"\nConverting distance units from meters to micrometers (1m = {MICROMETERS_PER_METER}µm)...")
        
        # Convert distance columns to micrometers
        df['Sensor0_Dist_um'] = df['Sensor0_Dist'] * MICROMETERS_PER_METER
        df['Sensor1_Dist_um'] = df['Sensor1_Dist'] * MICROMETERS_PER_METER
        
        df['Test_Index'] = df.index

        first_valid_index = df['Sensor0_Dist_um'].first_valid_index()

        # Separate Sensor Differences to first reading
        df['Sensor0_Diff_to_start_um'] = df['Sensor0_Dist_um'] - df['Sensor0_Dist_um'].iloc[first_valid_index]
        df['Sensor1_Diff_to_start_um'] = df['Sensor1_Dist_um'] - df['Sensor1_Dist_um'].iloc[first_valid_index]

        # Relative differences to first reading
        df['Sensor_Diff_um'] = df['Sensor0_Dist_um'] - df['Sensor1_Dist_um']
        df['Sensor_Diff_to_start_um'] = df['Sensor_Diff_um'] - df['Sensor_Diff_um'].iloc[first_valid_index]

    
        print("\nData Description (all distance units now in micrometers):")
        print(df[['Sensor0_Dist_um', 'Sensor1_Dist_um']].describe())
        print(df[['Sensor0_Diff_to_start_um', 'Sensor1_Diff_to_start_um']].describe())
        print(df[['Sensor_Diff_um', 'Sensor_Diff_to_start_um']].describe())

        # Commanded joint positions vs actual joint positions
        if 'Cmd_J1' in df.columns:
            df['Cmd_J1_Diff_to_start_rad'] = df['Cmd_J1'] - df['Cmd_J1'].iloc[first_valid_index]
            df['Cmd_J2_Diff_to_start_rad'] = df['Cmd_J2'] - df['Cmd_J2'].iloc[first_valid_index]
            df['Cmd_J3_Diff_to_start_rad'] = df['Cmd_J3'] - df['Cmd_J3'].iloc[first_valid_index]
            df['Cmd_J4_Diff_to_start_rad'] = df['Cmd_J4'] - df['Cmd_J4'].iloc[first_valid_index]

            df['J1_Diff_to_Cmd_rad'] = df['Cmd_J1'] - df['Act_J1']
            df['J2_Diff_to_Cmd_rad'] = df['Cmd_J2'] - df['Act_J2']
            df['J3_Diff_to_Cmd_rad'] = df['Cmd_J3'] - df['Act_J3']
            df['J4_Diff_to_Cmd_rad'] = df['Cmd_J4'] - df['Act_J4']
            df['J_Sum_Diff_to_Cmd_rad'] = (
                df['J1_Diff_to_Cmd_rad'] +
                df['J2_Diff_to_Cmd_rad'] +
                df['J3_Diff_to_Cmd_rad'] +
                df['J4_Diff_to_Cmd_rad']
            )

            df['J_Sum_1__to_3_Diff_to_Cmd_rad'] = (
                df['J1_Diff_to_Cmd_rad'] +
                df['J2_Diff_to_Cmd_rad'] +
                df['J3_Diff_to_Cmd_rad']
            )

            print("\nJoint Position Errors (in radians):")
            print(df[['Cmd_J1_Diff_to_start_rad', 'Cmd_J2_Diff_to_start_rad', 'Cmd_J3_Diff_to_start_rad', 'Cmd_J4_Diff_to_start_rad']].describe())
            print(df[['J1_Diff_to_Cmd_rad', 'J2_Diff_to_Cmd_rad', 'J3_Diff_to_Cmd_rad', 'J4_Diff_to_Cmd_rad']].describe())
            print(df[['J_Sum_Diff_to_Cmd_rad']].describe())


        return df, data_dir

    except FileNotFoundError:
        print(f"Error: The file '{filepath}' was not found.")
        print("Please make sure the CSV file is in the same directory as this script.")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

def save_plot(fig, plot_name, data_dir, show_plot=True):
    """Save plot to the same directory as the data file."""
    plot_filename = os.path.join(data_dir, f"{plot_name}.png")
    fig.savefig(plot_filename, dpi=300, bbox_inches='tight')
    print(f"Plot saved: {plot_filename}")
    
    if show_plot:
        plt.show()

def plot_sensor_readings_over_time(df, data_dir, show_plot=True):
    """Plot raw sensor readings (in µm) over the test sequence."""
    plt.style.use('seaborn-v0_8-whitegrid')
    plt.figure(figsize=(16, 9))
    
    plt.plot(df['Test_Index'], df['Sensor0_Dist_um'], label='Sensor 0 Distance', marker='.', linestyle='-', alpha=0.7)
    plt.plot(df['Test_Index'], df['Sensor1_Dist_um'], label='Sensor 1 Distance', marker='.', linestyle='-', alpha=0.7)
    
    plt.title('Sensor Readings Over Time (in Micrometers)')
    plt.xlabel('Test Index')
    plt.ylabel('Distance Reading (µm)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    save_plot(plt, 'sensor_readings', data_dir, show_plot)

def plot_sensor_difference_over_time(df, data_dir, show_plot=True):
    """Plot the difference between Sensor 0 and Sensor 1 to first position (in µm) over time."""
    plt.figure(figsize=(16, 9))
    
    mean_diff = df['Sensor_Diff_to_start_um'].mean()

    # plt.plot(df['Test_Index'], df['Sensor0_Diff_to_start_um'], label='Difference (Sensor0 - Start)', marker='^', linestyle='--')
    # plt.plot(df['Test_Index'], df['Sensor1_Diff_to_start_um'], label='Difference (Sensor1 - Start)', marker='s', linestyle='--')
    plt.plot(df['Test_Index'], df['Sensor_Diff_to_start_um'], label='Summed Difference to start ((Sensor0 - Sensor1) - start)', marker='o', linestyle='--')
    plt.axhline(mean_diff, color='r', linestyle='--', label=f'Mean Summed Difference ({mean_diff:.1f} µm)')
    
    print()

    plt.title('Sensor Difference to start Over Time (in Micrometers)')
    plt.xlabel('Test Index')
    plt.ylabel('Difference in Distance (µm)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    save_plot(plt, 'sensor_difference', data_dir, show_plot)

def plot_readings_by_speed(df, data_dir, show_plot=True):
    """Create boxplots to show sensor reading distributions by speed (in µm)."""
    plt.figure(figsize=(16, 9))
    
    # Plot for Sensor Difference
    # plt.subplot(1, 2, 1)
    plt.subplot(1, 1, 1)
    sns.boxplot(x='SpeedAndAccScale', y='Sensor_Diff_to_start_um', data=df, hue='SpeedAndAccScale', palette='viridis')
    plt.title('Sensor Difference vs. Speed')
    plt.xlabel('Speed and Acceleration Scale')
    plt.ylabel('Difference (µm)')
    
    # Plot for Average Sensor Reading
    # plt.subplot(1, 2, 2)
    # sns.boxplot(x='SpeedAndAccScale', y='Sensor_Diff_Avg_um', data=df, palette='plasma')
    # plt.title('Average Sensor Reading vs. Speed')
    # plt.xlabel('Speed and Acceleration Scale')
    # plt.ylabel('Average Distance (µm)')
    
    plt.suptitle('Sensor Performance by Speed/Acceleration Scale (in Micrometers)', fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    save_plot(plt, 'sensor_difference_by_speed', data_dir, show_plot)

def plot_readings_by_position(df, data_dir, show_plot=True):
    """Create violin plots to show sensor reading distributions by start position (in µm)."""
    plt.figure(figsize=(16, 9))

    sns.violinplot(x='StartPosition', y='Sensor_Diff_to_start_um', data=df, hue='StartPosition', palette='muted', inner='quartile')
    sns.stripplot(x='StartPosition', y='Sensor_Diff_to_start_um', data=df, color='black', size=3, jitter=True, alpha=0.5)

    plt.title('Sensor Difference Distribution by Start Position (in Micrometers)')
    plt.xlabel('Start Position')
    plt.ylabel('Difference (µm)')
    plt.xticks(rotation=30, ha='right')
    plt.grid(True)
    plt.tight_layout()
    save_plot(plt, 'sensor_difference_by_position', data_dir, show_plot)

def plot_combined_analysis(df, data_dir, show_plot=True):
    """Create a multi-faceted plot showing relationships between variables (drift in µm)."""
    g = sns.relplot(
        data=df,
        x='Test_Index',
        y='Sensor_Diff_to_start_um',
        hue='StartPosition',
        size='SpeedAndAccScale',
        style='StartPosition',
        palette='tab10',
        height=7,
        aspect=1.5,
        sizes=(50, 250)
    )
    
    g.figure.suptitle('Comprehensive View: Sensor Drift (µm) by Position, Speed, and Time', y=1.03)
    g.set(xlabel='Test Index', ylabel='Sensor Difference (µm)')
    plt.grid(True)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    save_plot(g.figure, 'combined_analysis', data_dir, show_plot)


def plot_commanded_positions_and_errors(df, data_dir, show_plot=True):
    """Plot commanded joint positions and their corresponding errors over time."""
    # Check if joint command columns exist
    if 'Cmd_J1' not in df.columns:
        print("Joint command data not available for position analysis.")
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 9))
    
    # Plot 1: Commanded Joint Positions Over Time
    ax1 = axes[0, 0]
    ax1.plot(df['Test_Index'], df['Cmd_J1_Diff_to_start_rad'], label='J1 Command', marker='.', alpha=0.7)
    ax1.plot(df['Test_Index'], df['Cmd_J2_Diff_to_start_rad'], label='J2 Command', marker='.', alpha=0.7)
    ax1.plot(df['Test_Index'], df['Cmd_J3_Diff_to_start_rad'], label='J3 Command', marker='.', alpha=0.7)
    ax1.plot(df['Test_Index'], df['Cmd_J4_Diff_to_start_rad'], label='J4 Command', marker='.', alpha=0.7)
    ax1.set_title('Commanded Joint Positions Over Time')
    ax1.set_xlabel('Test Index')
    ax1.set_ylabel('Joint Position (rad)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Individual Joint Errors
    ax2 = axes[0, 1]
    ax2.plot(df['Test_Index'], df['J1_Diff_to_Cmd_rad'], label='J1 Diff to Commanded', marker='.', alpha=0.7)
    ax2.plot(df['Test_Index'], df['J2_Diff_to_Cmd_rad'], label='J2 Diff to Commanded', marker='.', alpha=0.7)
    ax2.plot(df['Test_Index'], df['J3_Diff_to_Cmd_rad'], label='J3 Diff to Commanded', marker='.', alpha=0.7)
    ax2.plot(df['Test_Index'], df['J4_Diff_to_Cmd_rad'], label='J4 Diff to Commanded', marker='.', alpha=0.7)
    ax2.set_title('Individual Joint Command Errors Over Time')
    ax2.set_xlabel('Test Index')
    ax2.set_ylabel('Error (rad)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Commanded vs Actual Positions (stacked subplots for clarity)
    ax3 = axes[1, 0]
    # Show range of commanded positions
    joint_ranges = {
        'J1': (df['Cmd_J1_Diff_to_start_rad'].min(), df['Cmd_J1_Diff_to_start_rad'].max()),
        'J2': (df['Cmd_J2_Diff_to_start_rad'].min(), df['Cmd_J2_Diff_to_start_rad'].max()),
        'J3': (df['Cmd_J3_Diff_to_start_rad'].min(), df['Cmd_J3_Diff_to_start_rad'].max()),
        'J4': (df['Cmd_J4_Diff_to_start_rad'].min(), df['Cmd_J4_Diff_to_start_rad'].max())
    }
    
    joints = list(joint_ranges.keys())
    min_vals = [joint_ranges[j][0] for j in joints]
    max_vals = [joint_ranges[j][1] for j in joints]
    ranges = [max_vals[i] - min_vals[i] for i in range(len(joints))]
    
    bars = ax3.bar(joints, ranges, bottom=min_vals, alpha=0.7, color=['tab:blue', 'tab:orange', 'tab:green', 'tab:red'])
    ax3.set_title('Joint Command Range During Execution')
    ax3.set_ylabel('Joint Position (rad)')
    ax3.grid(True, alpha=0.3, axis='y')
    
    # Add range annotations
    for i, (joint, bar) in enumerate(zip(joints, bars)):
        height = ranges[i]
        ax3.annotate(f'{height:.3f} rad', 
                    xy=(bar.get_x() + bar.get_width()/2, min_vals[i] + height/2),
                    ha='center', va='center', fontweight='bold')
    
    # Plot 4: Sum Errors Over Time
    ax4 = axes[1, 1]
    ax4.plot(df['Test_Index'], df['J_Sum_1__to_3_Diff_to_Cmd_rad'], 
             label='J1-J3 Sum Diff to Commanded', marker='o', linestyle='--', alpha=0.7)
    ax4.plot(df['Test_Index'], df['J_Sum_Diff_to_Cmd_rad'], 
             label='J1-J4 Sum Diff to Commanded', marker='s', linestyle='-', alpha=0.7)

    # Add mean lines
    mean_j123 = df['J_Sum_1__to_3_Diff_to_Cmd_rad'].mean()
    mean_j1234 = df['J_Sum_Diff_to_Cmd_rad'].mean()
    ax4.axhline(mean_j123, color='tab:blue', linestyle=':', alpha=0.8, 
                label=f'J1-J3 Mean ({mean_j123:.4f} rad)')
    ax4.axhline(mean_j1234, color='tab:orange', linestyle=':', alpha=0.8, 
                label=f'J1-J4 Mean ({mean_j1234:.4f} rad)')
    
    ax4.set_title('Cumulative Joint Command Errors Over Time')
    ax4.set_xlabel('Test Index')
    ax4.set_ylabel('Sum Error (rad)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.suptitle('Joint Command Positions and Tracking Errors Analysis', fontsize=16, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    save_plot(plt, 'joint_error_analysis', data_dir, show_plot)

def plot_failed_runs_by_position(df, data_dir, show_plot=True):
    """Plot the number of failed runs per start position (NaN values in Sensor0_Dist_um)."""
    
    # Check if StartPosition column exists
    if 'StartPosition' not in df.columns:
        print("StartPosition column not available for failure analysis.")
        return
    
    # Check if Sensor0_Dist_um column exists
    if 'Sensor0_Dist_um' not in df.columns:
        print("Sensor0_Dist_um column not available for failure analysis.")
        return
    
    # Count failed runs (NaN values) per start position
    failed_runs = df[df['Sensor0_Dist_um'].isna()].groupby('StartPosition').size()
    
    # Count total runs per start position
    total_runs = df.groupby('StartPosition').size()
    
    # Calculate success rate
    success_runs = total_runs - failed_runs.reindex(total_runs.index, fill_value=0)
    failure_rate = (failed_runs.reindex(total_runs.index, fill_value=0) / total_runs * 100)
    
    # Create subplot layout
    fig, axes = plt.subplots(1, 2, figsize=(16, 9))
    
    # Plot 2: Total runs vs Failed runs
    ax2 = axes[0]
    x_pos = range(len(total_runs))
    width = 0.35
    
    bars1 = ax2.bar([x - width/2 for x in x_pos], success_runs.values, width, 
                    label='Successful Runs', color='green', alpha=0.7)
    bars2 = ax2.bar([x + width/2 for x in x_pos], failed_runs.reindex(total_runs.index, fill_value=0).values, width,
                    label='Failed Runs', color='red', alpha=0.7)
    
    ax2.set_title('Successful vs Failed Runs by Start Position')
    ax2.set_xlabel('Start Position')
    ax2.set_ylabel('Number of Runs')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(total_runs.index, rotation=45)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Failure rate percentage
    ax3 = axes[1]
    failure_rate.plot(kind='bar', ax=ax3, color='orange', alpha=0.7)
    ax3.set_title('Failure Rate by Start Position')
    ax3.set_xlabel('Start Position')
    ax3.set_ylabel('Failure Rate (%)')
    ax3.grid(True, alpha=0.3)
    ax3.tick_params(axis='x', rotation=45)
    
    # Add percentage labels on bars
    for i, v in enumerate(failure_rate.values):
        ax3.text(i, v + 0.5, f'{v:.1f}%', ha='center', va='bottom', fontweight='bold')
    
    
    # Print summary to console
    print(f"\n--- Failure Analysis Summary ---")
    print(f"\nFailure rate by start position:")
    for pos in total_runs.index:
        failed_count = failed_runs.get(pos, 0)
        total_count = total_runs[pos]
        rate = (failed_count / total_count * 100) if total_count > 0 else 0
        print(f"  {pos}: {failed_count}/{total_count} ({rate:.1f}%)")
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    save_plot(plt, 'failed_runs_by_position', data_dir, show_plot)

def main():
    """Main function to run the analysis."""
    
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = DEFAULT_FILENAME
    
    print(f"Analyzing file: {filename}")
    
    sensor_df, data_dir = load_and_prepare_data(filename)
    
    if sensor_df is None:
        sys.exit(1)
    
    # Add option to show plots or just save them
    show_plots = input("Show plots interactively? (y/n, default=y): ").lower().strip()
    show_plots = show_plots != 'n'
        
    print(f"\n--- Generating Plots (saving to: {data_dir}) ---")
    if show_plots:
        print("Close each plot window to see the next one.")
    else:
        print("Plots will be saved automatically without displaying.")

    plot_sensor_readings_over_time(sensor_df, data_dir, show_plots)
    plot_failed_runs_by_position(sensor_df, data_dir, show_plots)
    plot_sensor_difference_over_time(sensor_df, data_dir, show_plots)
    plot_readings_by_speed(sensor_df, data_dir, show_plots)
    plot_readings_by_position(sensor_df, data_dir, show_plots)
    plot_combined_analysis(sensor_df, data_dir, show_plots)
    plot_commanded_positions_and_errors(sensor_df, data_dir, show_plots)

    print("\n--- Analysis Complete ---")
    print(f"All plots saved to: {data_dir}")


if __name__ == '__main__':
    main()