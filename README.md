# Swarm Robotics: Optimizing Behavior for Survival in the Presence of Enemies

## Project Overview
**Platform:** [ARGoS Simulation Framework](https://argos-sim.info/)  
**Robotic Platform:** E-puck2  

This project focuses on optimizing swarm robotic behavior in a simulated environment where friendly robots must survive in the presence of adversarial enemy agents. The goal is to develop effective flocking behaviors that balance survival, target-seeking, and enemy avoidance. The project leverages the ARGoS simulation framework and supports large-scale testing with dynamically adjustable parameters.

This work is based on the integration of **Daniel H. Stolfi**'s contributions to integrating the **E-puck2** with ARGoS. However, significant additional integrations and adaptations were necessary to fully implement the survival-based swarm behaviors. My work focused on refining the simulation dynamics, enhancing modularity, and developing a robust parameter tuning and data collection pipeline while ensuring a scalable and adaptive framework. This contribution builds upon prior foundations while introducing new mechanisms for improved efficiency and analysis.

## Key Features
### 🏗 Dynamic and Modular Architecture
- **Independent Controllers:** Separate controllers for friendly and enemy robots.
- **Loop Functions:** Global simulation management (collision detection, robot removal, data logging).
- **Configurable Parameters:** Adjustable via ARGoS configuration file for seamless experimentation.

### 🔄 Survival-Oriented Optimization
- Robots optimize their movement strategies to avoid enemy collisions while maintaining flocking behavior.
- A **Lennard-Jones potential model** is used for flocking and repulsion forces.
- Adaptive repulsion forces differentiate friend-to-friend interactions from friend-to-enemy interactions.

### 📊 Scalability and Automation
- Supports large-scale testing with customizable numbers of friendly and enemy robots.
- Python-based automation pipeline runs thousands of simulations, collects results, and logs data into CSV files.

## Implementation Details
### 🚀 Goals Achieved
1. **Enemy-Aware Flocking Behavior**
   - Robots demonstrate dynamic flocking while avoiding enemies.
   - Target-seeking and wall avoidance behaviors integrated.
2. **Dynamic and Modular Structure**
   - Separate controllers for friendly and enemy robots.
   - Loop functions handle global event management (collisions, removals, logging).
   - Fully configurable parameters in the ARGoS configuration file.
3. **Behavior Visualization with LEDs**
   - Green LEDs for friendly robots, red LEDs for enemy robots.
4. **Robot Removal Logic**
   - Robots removed upon collision with enemies or upon reaching the target.
   - These behaviors can be toggled for different experimental setups.
5. **Large-Scale Parameter Testing**
   - Python automation adjusts parameters dynamically.
   - Runs simulations and logs results, analyzing survival rates under different conditions.

## 🛠 Technical Components
- **Controllers (C++):** Implement individual robot behaviors.
- **Loop Functions:** Handle global simulation logic such as collision detection and performance monitoring.
- **Python Automation Pipeline:** Automates simulation runs, parameter tuning, and data collection.
- **CSV Logging:** Stores survival metrics, allowing for future optimization and analysis.

### 📊 Data Collection
The project evaluates various parameters to optimize robotic survival rates:
- **Repulsion Forces (Friend-Friend, Friend-Enemy)**
- **Speed Factors** (Friendly & Enemy robots)
- **Lennard-Jones Model Parameters** (Gain, Exponent)
- **Target Distance & Light Strength**
- **Number of Robots and Enemies** (Tested with up to 15 friendly robots and 4 enemy robots)
- **Key Metrics Collected:**
  - Number of robots killed (collided with enemies)
  - Number of robots that reached the target
  - Total simulation ticks

## 🚀 Results and Achievements
✔ Implemented a modular, scalable simulation framework.  
✔ Developed survival-oriented flocking behavior in the presence of adversaries.  
✔ Collected extensive datasets for future analysis.  
✔ Visualized behavior using LED differentiation.  

## 🔮 Future Work
- **Parameter Optimization with Machine Learning:** Identify optimal survival parameters using AI models.
- **Reinforcement Learning for Movement:** Replace potential-field-based movement with reinforcement learning for adaptive strategies.
- **Adaptive Repulsion Forces:** Dynamically adjust repulsion based on density or enemy proximity.
- **3D Simulation Expansion:** Extend the model to simulate behavior in a 3D environment.
- **Physical Deployment:** Implement and test the algorithms on real-world E-puck robots.

## 🔗 GitHub Repository
📌 [Project Repository](https://github.com/EliyaHouri/flocking_e-puck)  
The source code, Python scripts, and ARGoS configuration files are available in the repository.

## 📢 Contact
If you have any questions or would like to collaborate, feel free to reach out via GitHub!

---
### 🌟 Why This Project Matters
This research contributes to swarm robotics, particularly in survival-driven behavior modeling. By combining modular architecture, automated large-scale testing, and adaptive control, this project lays the foundation for future research in reinforcement learning and real-world swarm deployment.

