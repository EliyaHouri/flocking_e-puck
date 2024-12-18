import json
import subprocess
import re
import os
import random
import csv

# Parameter ranges
parameter_ranges = {
    "repulsion_force": (10, 30),
    "speed_factor": (1, 4),
    "enemy_speed": (1, 3),
    "gain": (10, 1000),
    "target_distance": (15, 30),
    "num_robots": (1, 15),
    "num_enemies": (1, 4),
    "exponent": (3, 10),
    "light_strength": (0.3, 1.0),
    "friend_repulsion_force": (5, 20),  # New parameter for friend repulsion
    "enemy_repulsion_force": (10, 30),  # New parameter for enemy repulsion
}


# Default parameter values
default_values = {
    "repulsion_force": 15.0,
    "speed_factor": 2.0,
    "enemy_speed": 2.0,
    "gain": 10.0,
    "target_distance": 15.0,
    "num_robots": 9,
    "num_enemies": 2,
    "exponent": 6.0,
    "light_strength": 0.5
}

TICK_CAP = 20000  # Maximum allowed ticks (simulation timeout in seconds)

# Step 1: Modify .argos file based on the configuration
def update_argos_file(config, argos_file_path, output_argos_file_path):
    with open(argos_file_path, 'r') as file:
        content = file.read()

    # Merge default values with the current configuration
    for key, value in {**default_values, **config}.items():
        content = re.sub(fr'{key}=".*?"', f'{key}="{value}"', content)

    # Remove all existing robots from the .argos file
    content = re.sub(r'<e-puck2 .*?</e-puck2>', '', content, flags=re.DOTALL)

    # Add regular robots dynamically
    robots = []
    for i in range(config["num_robots"]):
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
    for j in range(config["num_enemies"]):
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

    print(f"Updated ARGoS file written to {output_argos_file_path} with {config['num_robots']} robots, "
          f"{config['num_enemies']} enemies, and light_strength={config['light_strength']}.")

# Step 2: Run the ARGoS simulation with timeout
def run_simulation(argos_file_path, tick_cap):
    print(f"Running simulation with {argos_file_path} (timeout: {tick_cap} seconds)...")
    if os.path.exists("results.txt"):
        os.remove("results.txt")  # Clear previous results
    
    try:
        # Add timeout to the subprocess run
        process = subprocess.run(['argos3', '-c', argos_file_path], 
                                 capture_output=True, text=True, timeout=tick_cap)
        if process.returncode != 0:
            print(f"Simulation failed with error: {process.stderr.strip()}")
            return False
    except subprocess.TimeoutExpired:
        print(f"Simulation timed out after {tick_cap} seconds.")
        return False
    return True

# Step 3: Parse the experiment results file
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

# Step 4: Generate random parameter configurations
def generate_random_configurations(num_samples):
    configurations = []
    for _ in range(num_samples):
        config = {
            "repulsion_force": round(random.uniform(*parameter_ranges["repulsion_force"]), 2),
            "speed_factor": round(random.uniform(*parameter_ranges["speed_factor"]), 2),
            "enemy_speed": round(random.uniform(*parameter_ranges["enemy_speed"]), 2),
            "gain": round(random.uniform(*parameter_ranges["gain"]), 2),
            "target_distance": round(random.uniform(*parameter_ranges["target_distance"]), 2),
            "num_robots": random.randint(*parameter_ranges["num_robots"]),
            "num_enemies": random.randint(*parameter_ranges["num_enemies"]),
            "exponent": round(random.uniform(*parameter_ranges["exponent"]), 2),
            "light_strength": round(random.uniform(0.3, 1.0), 2),
            "friend_repulsion_force": round(random.uniform(*parameter_ranges["friend_repulsion_force"]), 2),
            "enemy_repulsion_force": round(random.uniform(*parameter_ranges["enemy_repulsion_force"]), 2),
        }
        configurations.append(config)
    return configurations


# Step 5: Run simulations and save results
def run_simulations(argos_file_path, output_argos_file_path, configurations, output_csv, tick_cap):
    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["repulsion_force", "speed_factor", "enemy_speed", "gain", "target_distance", 
                         "num_robots", "num_enemies", "exponent", "light_strength", "killed", "reached_target", "ticks"])

        for config in configurations:
            update_argos_file(config, argos_file_path, output_argos_file_path)
            success = run_simulation(output_argos_file_path, tick_cap)
            if not success:
                continue

            killed, reached_target, ticks = parse_simulation_output()
            if killed is None or reached_target is None or ticks is None:
                continue

            writer.writerow([config["repulsion_force"], config["speed_factor"], config["enemy_speed"], 
                             config["gain"], config["target_distance"], config["num_robots"], 
                             config["num_enemies"], config["exponent"], config["light_strength"],
                             killed, reached_target, ticks])
            print(f"Results: {config} | Killed: {killed}, Reached Target: {reached_target}, Ticks: {ticks}")

# Main function
def main():
    argos_file_path = 'src/experiments/epuck2_flocking.argos'
    output_argos_file_path = 'src/experiments/temp_epuck2_flocking.argos'
    output_csv = 'simulation_results.csv'

    # Number of random configurations
    num_samples = 3000  # Adjust as needed
    tick_cap = TICK_CAP  # Timeout in seconds

    # Generate random configurations
    configurations = generate_random_configurations(num_samples)

    # Run simulations and save results
    run_simulations(argos_file_path, output_argos_file_path, configurations, output_csv, tick_cap)

if __name__ == "__main__":
    main()

