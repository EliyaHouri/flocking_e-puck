import json
import subprocess
import re
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import os

# Default parameter values
default_values = {
    "repulsion_force": 15.0,
    "speed_factor": 2.0,
    "enemy_speed": 2.0,
    "gain": 10.0,
    "target_distance": 20.0,
}

# Function to update the ARGoS file with new parameters, number of robots, and enemies
def update_argos_file(config, argos_file_path, output_argos_file_path, num_robots, num_enemies):
    with open(argos_file_path, 'r') as file:
        content = file.read()

    # Merge default values with the current test configuration
    for key, value in {**default_values, **config}.items():
        content = re.sub(fr'{key}=".*?"', f'{key}="{value}"', content)

    # Remove all e-puck robots
    content = re.sub(r'<e-puck2 .*?</e-puck2>', '', content, flags=re.DOTALL)

    # Add regular robots dynamically
    robots = []
    for i in range(num_robots):
        x = -1.0 + (i % 5) * 0.4
        y = -1.0 + (i // 5) * 0.4
        robots.append(
            f'<e-puck2 id="epuck_{i}" rab_range="0.5" rab_data_size="3">'
            f'<body position="{x},{y},0" orientation="0,0,0"/>'
            f'<controller config="fdc" />'
            f'</e-puck2>'
        )

    # Add enemy robots dynamically
    enemies = []
    base_x = 1.45
    base_y = 1.0
    for j in range(num_enemies):
        y = base_y + j * 1.5
        enemies.append(
            f'<e-puck2 id="enemy_robot_{j}" rab_range="0.5" rab_data_size="3">'
            f'<body position="{base_x},{y},0" orientation="0,0,0"/>'
            f'<controller config="enemy" />'
            f'</e-puck2>'
        )

    # Insert robots and enemies into the arena
    content = re.sub(r'(</arena>)', '\n'.join(robots + enemies) + r'\1', content)

    # Save the modified content to a new .argos file
    with open(output_argos_file_path, 'w') as file:
        file.write(content)

    print(f"Updated ARGoS file written to {output_argos_file_path} with {num_robots} robots and {num_enemies} enemies.")

# Function to run the ARGoS simulation
def run_simulation(argos_file_path):
    print(f"Running simulation with {argos_file_path}...")
    if os.path.exists("results.txt"):
        os.remove("results.txt")  # Clear previous results
    process = subprocess.run(['argos3', '-c', argos_file_path], capture_output=True, text=True)
    if process.returncode != 0:
        print(f"Simulation failed with error: {process.stderr.strip()}")
    else:
        print(f"Simulation completed successfully.")

# Function to parse the simulation output
def parse_simulation_output():
    if not os.path.exists("results.txt"):
        print("Error: results.txt not found. Simulation might have failed.")
        return {}, {}, 0

    with open('results.txt', 'r') as file:
        lines = file.readlines()
        distance_data = {}
        collision_data = {}
        total_collisions = 0

        mode = None
        for line in lines:
            if line.startswith("Distance Traveled:"):
                mode = "distance"
            elif line.startswith("Collision Count:"):
                mode = "collision"
            elif line.startswith("Total Collisions:"):
                total_collisions = int(line.split(":")[1].strip())
                mode = None
            elif mode == "distance":
                robot_id, distance = line.split(":")
                distance_data[robot_id.strip()] = float(distance.strip())
            elif mode == "collision":
                robot_id, collisions = line.split(":")
                collision_data[robot_id.strip()] = int(collisions.strip())

    return collision_data, distance_data, total_collisions

# Function to perform a parameter test
def parameter_test(argos_file_path, output_argos_file_path, param_combinations, num_robots, num_enemies):
    results = []
    collision_heatmap_data = []
    distance_data = []

    for config in param_combinations:
        # Update the ARGoS file
        update_argos_file(config, argos_file_path, output_argos_file_path, num_robots, num_enemies)

        # Run the simulation
        run_simulation(output_argos_file_path)

        # Parse the simulation results
        collision_counts, distances, total_collisions = parse_simulation_output()

        if not collision_counts and not distances and total_collisions == 0:
            print("Simulation failed or results are invalid.")
            continue

        # Store data for heatmap and metrics
        results.append((config, total_collisions, np.mean(list(distances.values())) if distances else 0))
        collision_heatmap_data.append([collision_counts.get(f"epuck_{i}", 0) for i in range(num_robots)])
        distance_data.append(np.mean(list(distances.values())) if distances else 0)

        print(f"Config: {config} | Total Collisions: {total_collisions}, Avg Distance: {np.mean(list(distances.values())) if distances else 0}")

    # Generate plots
    plot_collision_heatmap(param_combinations, collision_heatmap_data, num_robots)
    plot_metrics(param_combinations, results)

# Function to plot a heatmap for collision counts
def plot_collision_heatmap(param_combinations, collision_heatmap_data, num_robots):
    if not collision_heatmap_data:
        print("No data available for collision heatmap.")
        return

    plt.figure(figsize=(10, 8))
    sns.heatmap(
        np.array(collision_heatmap_data).T,
        annot=True,
        fmt="d",
        cmap="RdBu",
        xticklabels=[f"P1={config['repulsion_force']}, P2={config['speed_factor']}" for config in param_combinations],
        yticklabels=[f"epuck_{i}" for i in range(num_robots)],
    )
    plt.title("Collision Count per Robot")
    plt.xlabel("Parameter Configurations")
    plt.ylabel("Robots")
    plt.xticks(rotation=45, ha="right")
    plt.tight_layout()
    plt.show()

# Function to plot total collisions and average distance traveled
def plot_metrics(param_combinations, results):
    configs = [f"P1={config['repulsion_force']}, P2={config['speed_factor']}" for config, _, _ in results]
    total_collisions = [r[1] for r in results]
    avg_distances = [r[2] for r in results]

    # Plot Total Collisions
    plt.figure(figsize=(10, 6))
    plt.bar(range(len(total_collisions)), total_collisions, color="blue")
    plt.title("Effect of Parameters on Total Collisions")
    plt.xticks(range(len(total_collisions)), configs, rotation=45, ha="right")
    plt.ylabel("Total Collisions")
    plt.tight_layout()
    plt.show()

    # Plot Average Distance Traveled
    plt.figure(figsize=(10, 6))
    plt.bar(range(len(avg_distances)), avg_distances, color="green")
    plt.title("Effect of Parameters on Distance Traveled")
    plt.xticks(range(len(avg_distances)), configs, rotation=45, ha="right")
    plt.ylabel("Average Distance Traveled")
    plt.tight_layout()
    plt.show()

# Main function
def main():
    argos_file_path = "src/experiments/epuck2_flocking.argos"
    output_argos_file_path = "src/experiments/temp_epuck2_flocking.argos"

    # Number of robots
    num_robots = 9  # Change this to modify the number of regular robots
    num_enemies = 3  # Change this to modify the number of enemy robots

    # Define parameter combinations
    param_combinations = [
        {"repulsion_force": 10, "speed_factor": 1},
        {"repulsion_force": 10, "speed_factor": 2},
        {"repulsion_force": 20, "speed_factor": 1},
        {"repulsion_force": 20, "speed_factor": 2},
    ]

    # Run parameter tests
    parameter_test(argos_file_path, output_argos_file_path, param_combinations, num_robots, num_enemies)

# Run the main function
if __name__ == "__main__":
    main()

