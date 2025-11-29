| #  | Category                     | Function Signature                                                                           | Description                          | Comment      | Responsible |
| -- | ---------------------------- | -------------------------------------------------------------------------------------------- | ------------------------------------ | ------------ | ----------- |
| 1  | 🚀 Navigation (Action-based) | `goThroughPoses(self, poses, behavior_tree='')`                                            | Navigate through multiple poses      | ✅ Done      | -           |
| 2  |                              | `goToPose(self, pose, behavior_tree='')`                                                   | Navigate to a single pose            | ✅ Done      | -           |
| 3  |                              | `followWaypoints(self, poses)`                                                             | Follow a sequence of waypoints       | ✅ Done      | -           |
| 4  |                              | `followGpsWaypoints(self, gps_poses)`                                                      | Follow GPS-based waypoints           | ✅ Done      | -           |
| 5  |                              | `spin(self, spin_dist=1.57, time_allowance=10)`                                            | Rotate the robot                     | ✅ Done      | -           |
| 6  |                              | `backup(self, backup_dist=0.15, backup_speed=0.025, time_allowance=10)`                    | Move backward                        | ✅ Done      | -           |
| 7  |                              | `driveOnHeading(self, dist=0.15, speed=0.025, time_allowance=10)`                          | Drive in a straight line             | ✅ Done      | -           |
| 8  |                              | `assistedTeleop(self, time_allowance=30)`                                                  | Assistive teleoperation              | ✅ Done      | -           |
| 9  |                              | `followPath(self, path, controller_id='', goal_checker_id='')`                             | Follow a specific path               | ✅ Done      | -           |
| 10 | 🧠 Planning & Smoothing      | `getPath(self, start, goal, planner_id='', use_start=False)`                               | Compute path from start to goal      | ✅ Done      | -           |
| 11 |                              | `getPathThroughPoses(self, start, goals, planner_id='', use_start=False)`                  | Compute path through multiple goals  | ✅ Done      | -           |
| 12 |                              | `smoothPath(self, path, smoother_id='', max_duration=2.0, check_for_collision=False)`      | Smooth a given path                  | ✅ Done      | -           |
| 13 | ⚙️ Lifecycle & Control     | `cancelTask(self)`                                                                         | Cancel the current task              | ✅ Done      |             |
| 14 |                              | `isTaskComplete(self)`                                                                     | Check if task has completed          | ✅ Done      |             |
| 15 |                              | `getFeedback(self)`                                                                        | Get feedback from the action server  | ✅ Done      |             |
| 16 |                              | `getResult(self)`                                                                          | Get result of the completed task     |              |             |
| 17 |                              | `waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl')`                    | Wait until navigation is active      | ✅ Done      |             |
| 18 |                              | `lifecycleStartup(self)`                                                                   | Start lifecycle-managed nodes        | ✅ Done      |             |
| 19 |                              | `lifecycleShutdown(self)`                                                                  | Shutdown lifecycle-managed nodes     | ✅ Done      |             |
| 20 |                              | `changeMap(self, map_filepath)`                                                            | Change the active map                | ✅ Done      |             |
| 21 |                              | `clearAllCostmaps(self)`                                                                   | Clear both local and global costmaps | ✅ Done      |             |
| 22 |                              | `clearLocalCostmap(self)`                                                                  | Clear the local costmap              | ✅ Done      |             |
| 23 |                              | `clearGlobalCostmap(self)`                                                                 | Clear the global costmap             | ✅ Done      |             |
| 24 |                              | `getGlobalCostmap(self)`                                                                   | Retrieve the global costmap          | ✅ Done      |             |
| 25 |                              | `getLocalCostmap(self)`                                                                    | Retrieve the local costmap           | ✅ Done      |             |
| 26 | 🔧 Utility / Internal        | `__init__(self, node_name='basic_navigator', namespace='')`                                | Constructor                          | Not Required |             |
| 27 |                              | `destroyNode(self)`                                                                        | Destroy node (alias)                 | Not Required |             |
| 28 |                              | `destroy_node(self)`                                                                       | Destroy node                         | Not Required |             |
| 29 |                              | `setInitialPose(self, initial_pose)`                                                       | Set the initial pose                 | ✅ Done      |             |
| 30 |                              | `_getPathImpl(self, start, goal, planner_id='', use_start=False)`                          | Internal path computation            | ✅ Done      |             |
| 31 |                              | `_getPathThroughPosesImpl(self, start, goals, planner_id='', use_start=False)`             | Internal multi-goal path computation | ✅ Done      |             |
| 32 |                              | `_smoothPathImpl(self, path, smoother_id='', max_duration=2.0, check_for_collision=False)` | Internal path smoothing              | ✅ Done      |             |
| 33 |                              | `_waitForNodeToActivate(self, node_name)`                                                  | Wait for a node to become active     | ✅ Done      |             |
| 34 |                              | `_waitForInitialPose(self)`                                                                | Wait until initial pose is received  | ✅ Done      |             |
| 35 |                              | `_amclPoseCallback(self, msg)`                                                             | Callback for AMCL pose               | ✅ Done      |             |
| 36 |                              | `_feedbackCallback(self, msg)`                                                             | Callback for action feedback         | ✅ Done      |             |
| 37 |                              | `_setInitialPose(self)`                                                                    | Publish the initial pose             | ✅ Done      |             |
|    |                              |                                                                                              |                                      |              |             |

| Priority | Function Name                                    | Reason                                              |
| -------: | ------------------------------------------------ | --------------------------------------------------- |
|        1 | `waitForNodeToActivate`                        | Required by multiple future functions               |
|        2 | `lifecycleStartup`                             | Builds directly on top of `waitForNodeToActivate` |
|        3 | `waitUntilNav2Active`                          | Combines wait for localization and navigation nodes |
|        4 | `cancelTask`                                   | Useful for safety and timeout management            |
|        5 | `getResult`                                    | Complements your existing `getFeedback()`         |
|        6 | `clearLocalCostmap`, `getLocalCostmap`, etc. | Smaller and self-contained — quick to add later    |

**Now**

- functions

**Next**

- examples
- result cb
- enum running task
