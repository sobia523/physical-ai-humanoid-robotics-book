# Reproducible Exercise Scenarios for Nav2 Bipedal Navigation

## Scenario 1: Basic Navigation in Open Environment

### Objective
Demonstrate basic Nav2 functionality with bipedal humanoid robot in a simple environment.

### Environment Setup
- **Environment:** Simple rectangular room (10m x 8m) with no obstacles
- **Robot Position:** Start at coordinates (1, 1) facing 0° (East)
- **Goal Position:** Navigate to coordinates (8, 6)
- **Map:** Empty room with clear floor space

### Expected Behavior
1. Robot should plan a direct path from start to goal
2. Navigation should complete successfully without recovery behaviors
3. Path execution time: 60-90 seconds
4. Robot should maintain stable upright posture throughout navigation

### Success Criteria
- [ ] Navigation completes within 120 seconds
- [ ] Robot reaches goal with position accuracy ±0.2m
- [ ] No balance failures during navigation
- [ ] Path follows expected trajectory

### Configuration Required
```yaml
# Use basic bipedal-nav2-config.yaml
# Ensure linear velocity: 0.5 m/s
# Ensure angular velocity: 0.4 rad/s
```

### Verification Steps
1. Launch Isaac Sim with simple room environment
2. Spawn bipedal robot at starting position
3. Set navigation goal using RViz or command
4. Monitor navigation progress and completion
5. Record path efficiency and stability metrics

## Scenario 2: Navigation with Static Obstacles

### Objective
Test Nav2 obstacle avoidance capabilities with bipedal-specific constraints.

### Environment Setup
- **Environment:** Same 10m x 8m room with 3 static obstacles
- **Obstacles:**
  - Box 1: 2m x 2m at (4, 2)
  - Box 2: 1.5m x 1.5m at (6, 4)
  - Box 3: 1m x 3m at (3, 6)
- **Robot Position:** Start at (1, 1)
- **Goal Position:** Navigate to (8, 7)

### Expected Behavior
1. Robot should detect obstacles using perception system
2. Path planner should generate route around obstacles
3. Navigation should account for robot's minimum turning radius
4. Costmap inflation should prevent collisions

### Success Criteria
- [ ] Navigation completes successfully around obstacles
- [ ] Robot maintains safe distance from obstacles (≥0.5m)
- [ ] Path avoids all obstacles without collision
- [ ] Navigation completes within 180 seconds

### Configuration Required
```yaml
# Use bipedal-nav2-config.yaml with:
# inflation_radius: 0.7
# robot_radius: 0.3
# local/global costmap resolution: 0.05
```

### Verification Steps
1. Set up environment with obstacles
2. Verify obstacle detection in costmaps
3. Execute navigation and monitor path planning
4. Validate obstacle avoidance behavior
5. Record metrics on path efficiency

## Scenario 3: Tight Space Navigation

### Objective
Test navigation through narrow passages with bipedal stability requirements.

### Environment Setup
- **Environment:** Corridor with narrowing sections
- **Corridor Dimensions:**
  - Start: 2m width, narrows to 0.8m, expands to 2m
  - Length: 10m total
- **Robot Position:** Start at corridor entrance
- **Goal Position:** Navigate to end of corridor

### Expected Behavior
- Robot should navigate through narrow sections carefully
- Velocity should reduce in tight spaces
- Balance should remain stable despite constrained movement

### Success Criteria
- [ ] Robot successfully navigates through 0.8m passage
- [ ] No collisions with corridor walls
- [ ] Navigation completes without recovery behaviors
- [ ] Robot maintains stability throughout passage

### Configuration Required
```yaml
# Use bipedal-nav2-config.yaml with adjustments:
# reduced linear velocity: 0.2 m/s in narrow sections
# increased inflation: 0.6m to account for stability margin
# reduced minimum turning radius: 0.25m
```

### Verification Steps
1. Create corridor environment with variable width
2. Configure robot for careful navigation in tight spaces
3. Execute navigation through narrow section
4. Monitor for stability and collision avoidance
5. Record performance metrics

## Scenario 4: Dynamic Obstacle Avoidance

### Objective
Test Nav2 dynamic obstacle avoidance for bipedal humanoid robots.

### Environment Setup
- **Environment:** 10m x 8m room with moving obstacles
- **Dynamic Obstacles:**
  - Human model moving at 0.5 m/s across robot path
  - Moving cart at 0.3 m/s in perpendicular direction
- **Robot Position:** Start at (1, 1)
- **Goal Position:** Navigate to (8, 6)

### Expected Behavior
1. Robot should detect moving obstacles
2. Navigation should pause or reroute appropriately
3. Robot should resume navigation when safe
4. Dynamic obstacle avoidance should maintain stability

### Success Criteria
- [ ] Robot avoids all dynamic obstacles safely
- [ ] Navigation completes despite dynamic obstacles
- [ ] No collisions with moving obstacles
- [ ] Navigation completes within 240 seconds

### Configuration Required
```yaml
# Use bipedal-nav2-config.yaml with dynamic obstacle settings:
# obstacle_layer/track_unknown_space: false
# local costmap update frequency: 10Hz
# controller frequency: 20Hz for responsive avoidance
```

### Verification Steps
1. Set up environment with dynamic obstacles
2. Configure dynamic obstacle detection parameters
3. Execute navigation with moving obstacles
4. Monitor avoidance behavior and safety
5. Record success rate and navigation efficiency

## Scenario 5: Multi-Goal Navigation

### Objective
Test sequential navigation to multiple goals with bipedal considerations.

### Environment Setup
- **Environment:** 12m x 10m room with multiple waypoints
- **Goals:**
  - Goal 1: (3, 2)
  - Goal 2: (7, 4)
  - Goal 3: (5, 8)
  - Goal 4: (2, 7)
- **Robot Position:** Start at (1, 1)

### Expected Behavior
1. Robot should navigate to each goal in sequence
2. System should replan efficiently between goals
3. Balance should be maintained throughout multi-goal navigation

### Success Criteria
- [ ] All goals reached in sequence
- [ ] Total navigation time under 300 seconds
- [ ] No balance failures during transitions
- [ ] Efficient path planning between goals

### Configuration Required
```yaml
# Use bipedal-nav2-config.yaml with:
# goal_checker xy_tolerance: 0.25m
# controller frequency: 20Hz for smooth transitions
# appropriate timeout values for multi-goal scenarios
```

### Verification Steps
1. Define multiple goal sequence
2. Execute multi-goal navigation
3. Monitor each goal achievement
4. Track efficiency of transitions
5. Record overall performance metrics

## Scenario 6: Recovery Behavior Testing

### Objective
Test Nav2 recovery behaviors for bipedal humanoid robots.

### Environment Setup
- **Environment:** Room with areas designed to trigger recovery
- **Challenging Areas:**
  - Narrow passage that may cause oscillation
  - Area with insufficient space for turn
  - Dead-end passage
- **Robot Position:** Start in recovery-triggering area

### Expected Behavior
1. Robot should detect navigation problems
2. Recovery behaviors should activate appropriately
3. Robot should recover and continue navigation
4. Recovery should maintain bipedal stability

### Success Criteria
- [ ] Recovery behaviors activate when needed
- [ ] Robot successfully recovers from navigation issues
- [ ] No damage to robot during recovery
- [ ] Navigation completes after recovery

### Configuration Required
```yaml
# Use bipedal-recovery-config.yaml with:
# spin recovery: spin_dist 1.57
# backup recovery: backup_dist 0.3m
# wait recovery: wait_duration 2.0s
```

### Verification Steps
1. Set up challenging environment
2. Execute navigation in difficult areas
3. Monitor recovery behavior activation
4. Validate successful recovery
5. Record recovery performance metrics

## Scenario 7: Long-Duration Navigation

### Objective
Test sustained navigation performance with bipedal robots.

### Environment Setup
- **Environment:** Large 20m x 15m environment with multiple features
- **Navigation Task:** Visit 10 different locations over 10 minutes
- **Path:** Predefined route covering various terrain types

### Expected Behavior
1. Robot should maintain localization accuracy over time
2. Navigation performance should remain consistent
3. System resources should remain stable
4. Bipedal stability should be maintained throughout

### Success Criteria
- [ ] All 10 locations visited successfully
- [ ] Localization accuracy maintained (±0.3m)
- [ ] No performance degradation over time
- [ ] Stable resource usage throughout

### Configuration Required
```yaml
# Use optimized bipedal-nav2-config.yaml for long-duration:
# AMCL resample_interval: 2
# Controller frequency: 20Hz
# Appropriate timeout values for extended operation
```

### Verification Steps
1. Set up long-duration navigation task
2. Monitor system performance over time
3. Track localization accuracy
4. Record resource usage patterns
5. Validate sustained stability

## General Verification Procedures

### Pre-Scenario Checklist
- [ ] Robot URDF and transforms validated
- [ ] Sensor calibration completed
- [ ] Navigation configuration loaded
- [ ] Environment loaded and verified
- [ ] Safety measures in place

### Post-Scenario Analysis
- [ ] Navigation performance metrics recorded
- [ ] Path efficiency calculated
- [ ] Stability metrics evaluated
- [ ] Resource usage analyzed
- [ ] Success criteria validated
- [ ] Issues documented for improvement