import json
import subprocess
import re
import matplotlib.pyplot as plt
import os

# Step 1: Define default parameter values
default_values = {
    "repulsion_force": 15.0,
    "speed_factor": 2.0,
    "enemy_speed": 2.0,
    "gain": 10.0,
    "target_distance": 15.0,
}

# Step 2: Modify .argos file based on the configuration, number of robots, and enemies
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

# Step 3: Run the ARGoS simulation and measure the time taken
def run_simulation(argos_file_path):
    print(f"Running simulation with {argos_file_path}...")
    if os.path.exists("results.txt"):
        os.remove("results.txt")  # Clear previous results
    process = subprocess.run(['argos3', '-c', argos_file_path], capture_output=True, text=True)
    if process.returncode != 0:
        print(f"Simulation failed with error: {process.stderr.strip()}")
    else:
        print(f"Simulation completed successfully.")

# Step 4: Parse the experiment results file
def parse_simulation_output():
    if not os.path.exists("results.txt"):
        print("Error: results.txt not found. Simulation might have failed.")
        return None, None, None

    with open('results.txt', 'r') as file:
        lines = file.readlines()
        killed = int(lines[0].split()[1])
        reached_target = int(lines[1].split()[1])
        ticks = int(lines[2].split()[1])
    return killed, reached_target, ticks

# Step 5: Perform a parameter test
def parameter_test(argos_file_path, output_argos_file_path, tests, num_robots, num_enemies):
    results = {key: [] for key in tests.keys()}
    for param_name, param_values in tests.items():
        print(f"Testing parameter: {param_name}")
        for value in param_values:
            config = {param_name: value}
            update_argos_file(config, argos_file_path, output_argos_file_path, num_robots, num_enemies)

            # Run the simulation
            run_simulation(output_argos_file_path)

            # Parse the simulation results
            killed, reached_target, ticks = parse_simulation_output()

            if killed is None or reached_target is None or ticks is None:
                print(f"Simulation failed or results are invalid for {param_name}={value}.")
                continue

            # Store results
            results[param_name].append((value, killed, reached_target, ticks))
            print(f"{param_name}={value} | Killed: {killed}, Reached Target: {reached_target}, Ticks: {ticks}")

    # Plot results for all parameters
    for param_name in tests.keys():
        plot_results(param_name, results[param_name])

# Step 6: Plot results
def plot_results(param_name, results):
    param_values = [r[0] for r in results]
    killed_counts = [r[1] for r in results]
    reached_counts = [r[2] for r in results]
    ticks = [r[3] for r in results]

    # Plot Killed and Reached Target
    plt.figure(figsize=(10, 6))
    plt.plot(param_values, killed_counts, marker='o', label='Killed by Enemy', color='red')
    plt.plot(param_values, reached_counts, marker='s', label='Reached Target', color='green')
    plt.xlabel(param_name.capitalize())
    plt.ylabel('Number of Robots')
    plt.title(f"Effect of {param_name.capitalize()} on Robot Outcomes")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Plot Ticks
    plt.figure(figsize=(10, 6))
    plt.plot(param_values, ticks, marker='d', label='Simulation Ticks', color='blue')
    plt.xlabel(param_name.capitalize())
    plt.ylabel('Ticks')
    plt.title(f"Effect of {param_name.capitalize()} on Simulation Ticks")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# Main function
def main():
    argos_file_path = 'src/experiments/epuck2_flocking.argos'
    output_argos_file_path = 'src/experiments/temp_epuck2_flocking.argos'
    num_robots = 9  # Number of regular robots
    num_enemies = 2  # Number of enemy robots

    # Define tests for the parameters
    tests = {
        "repulsion_force": [10, 20, 30, 40],
        "speed_factor": [1, 2, 3, 4],
        "enemy_speed": [1, 2, 3, 4],
        "gain": [1, 10, 100, 1000],
        "target_distance": [15, 20, 25, 30],
    }

    # Run the parameter tests
    parameter_test(argos_file_path, output_argos_file_path, tests, num_robots, num_enemies)

if __name__ == "__main__":
    main()

