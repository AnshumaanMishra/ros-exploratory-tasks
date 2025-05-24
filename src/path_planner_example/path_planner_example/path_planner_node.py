#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from create_plan_msgs.srv import CreatePlan
from nav2_simple_commander.robot_navigator import BasicNavigator


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__("path_planner_node")
        # self.basic_navigator = BasicNavigator()  # Can be uncommented to get Global Costmap in create_plan_cb

        # Creating a new service "create_plan", which is called by our Nav2 C++ planner plugin
        # to receive a plan from us.
        self.srv = self.create_service(CreatePlan, 'create_plan', self.create_plan_cb)

    def create_plan_cb(self, request, response):
        # Getting all the information to plan the path
        goal_pose = request.goal
        start_pose = request.start
        time_now = self.get_clock().now().to_msg()
        # global_costmap = self.basic_navigator.getGlobalCostmap()  # Can be uncommented to get Global CostMap

        response.path = create_straight_plan(start_pose, goal_pose, time_now)
        return response

def create_straight_plan(start, goal, time_now):
    """ 
    Creates a straight plan between start and goal points.
    Does not use the global costmap to plan around obstacles, as normal planners would.
    """
    path = Path()

    # Set the frame_id and stamp for the Path header. Use frame_id from the goal header,
    #  and time_now for the current time.
    path.header.frame_id = goal.header.frame_id
    path.header.stamp = time_now

    # Let's create a straight plan between our start and goal poses.
    # It is not enough if we provide only the start and end positions as a path.
    # For controller to follow path correctly, we will need to provide also
    # points along this straight path with small intervals. There is a function
    # "interpolate_coordinates" implemented for you that does this. It only needs
    # the coordinates in a tuple format, for example:
    # interpolate_coordinates((0, 0), (0, 0.5))
    # This will give you coordinates between these two points with 0.1 interval:
    # [(0.0, 0.0), (0.0, 0.1), (0.0, 0.2), (0.0, 0.3), (0.0, 0.4), (0.0, 0.5)]
    # Interpolate the coordinates between start and goal positions
    interpolated_coordinates = interpolate_coordinates(
        (start.pose.position.x, start.pose.position.y),
        (goal.pose.position.x, goal.pose.position.y),
    )
    
    # Loop through these interpolated coordinates and create a new PoseStamped()
    #  message for each of them. You can set the same stamp and frame_id as for the Path().
    #  Finally, add all of these points into the path.poses -array.
    for point in interpolated_coordinates:
        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.header.stamp = time_now
        pose.header.frame_id = goal.header.frame_id
        path.poses.append(pose)

    return path

def interpolate_coordinates(start, end, increment=0.1):
    """
    Interpolate coordinates between two points with a fixed increment.
    This method calculates the coordinates of the points on the straight-line path that we are computing.
    
    Args:
        start (tuple): Starting coordinate (x1, y1).
        end (tuple): Ending coordinate (x2, y2).
        increment (float): Distance between interpolated points.

    Returns:
        list: List of interpolated points as (x, y) tuples.
    """
    x1, y1 = start
    x2, y2 = end

    # Calculate total distance using the Euclidean formula
    dx = x2 - x1
    dy = y2 - y1
    distance = (dx ** 2 + dy ** 2) ** 0.5

    # Calculate the number of steps
    num_steps = int(distance / increment)

    # Generate interpolated points
    points = []
    for i in range(num_steps + 1):  # +1 to include the end point
        t = i / num_steps  # Normalized step (0.0 to 1.0)
        x = x1 + t * dx  # Linear interpolation for x
        y = y1 + t * dy  # Linear interpolation for y
        points.append((x, y))

    return points


def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()

    try:
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass

    path_planner_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
