import numpy as np
import random

# Q-learning setup
actions = ["W", "S", "A", "D", "SPACE"]
q_table = {}

def get_q_value(state, action):
    return q_table.get((state, action), 0)

def set_q_value(state, action, value):
    q_table[(state, action)] = value

def choose_action(state, epsilon=0.1):
    if random.random() < epsilon:
        return random.choice(actions)
    else:
        q_values = [get_q_value(state, action) for action in actions]
        max_value = max(q_values)
        best_actions = [actions[i] for i, q in enumerate(q_values) if q == max_value]
        return random.choice(best_actions)

def update_q_table(state, action, reward, next_state, alpha=0.1, gamma=0.9):
    current_q = get_q_value(state, action)
    max_q_next = max(get_q_value(next_state, a) for a in actions)
    new_q = current_q + alpha * (reward + gamma * max_q_next - current_q)
    set_q_value(state, action, new_q)

# Simulation
env = GameEnvironment(grid_size=10, ball_positions=[(2, 2), (5, 5), (7, 8)])
for episode in range(1000):
    state = env.reset()
    for _ in range(100):
        action = choose_action(state)
        next_state, reward = env.step(action)
        update_q_table(state, action, reward, next_state)
        state = next_state

# Print final Q-values (for demonstration purposes)
print(q_table)