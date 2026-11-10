# W0045E-Assignment1

# Steering Behaviors – Unreal Engine 5.8

## Project Description

This project was created in Unreal Engine 5.8 using C++ as part of a game programming assignment.

The goal was to implement and test different steering behaviors that control how AI agents move, follow targets, and avoid obstacles.

## Task 1 – Basic Steering Behaviors

The following behaviors were implemented:

- **Seek:** Moves towards a target.
- **Flee:** Moves away from a target.
- **Pursue:** Predicts and follows a moving target.
- **Evade:** Predicts and moves away from a moving target.
- **Arrive:** Slows down when approaching a target.
- **Wander:** Moves around with gradual random direction changes.

## Task 2 – Advanced Steering Behaviors

The following behaviors were implemented:

- **Path Following:** Follows four predefined waypoints.
- **Separation:** Keeps distance from nearby agents.
- **Collision Avoidance:** Attempts to avoid collisions with moving agents.
- **Obstacle Avoidance:** Steers around stationary obstacles.
- **Wall Avoidance:** Steers away from the platform boundaries.

## Task 3 – Combining Steering Behaviors

For Task 3, I chose to combine multiple steering behaviors.

The agent can follow a predefined path while also avoiding obstacles, other agents, and platform boundaries.

The combined behavior uses a priority system where avoiding walls, obstacles, and collisions takes priority over following the path.

## Implementation

The steering behaviors are implemented in C++ using separate functions in `SteeringAgent.cpp`.

The project uses Unreal Engine's `FVector` for movement calculations. Agent movement is updated every frame using `Tick()` and `DeltaTime`.

The simulation contains two agents, four path points, one stationary obstacle, and a platform. A hard boundary was also added to prevent agents from leaving the platform.

## How to Test

1. Open the project in Unreal Engine 5.8.
2. Open `SteeringLevel`.
3. Select `BP_SteeringAgent` or `TargetAgent`.
4. Under **Details → Steering**, select the desired behavior.
5. Assign the required target, path points, other agents, or obstacles.
6. Press **Play** to run the simulation.

The Combined behavior demonstrates multiple steering behaviors working together.

## Running the Executable

A packaged Windows version is included with the submission.

To run the simulation, open the packaged build folder and launch `SteeringBehaviors.exe`.

Keep the other packaged files and folders together with the executable.

## Conclusion

All three tasks have been implemented and tested. The project demonstrates how simple steering calculations can be used to create different movement behaviors and how they can be combined to produce more complex agent movement.