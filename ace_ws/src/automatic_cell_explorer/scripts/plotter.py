import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rosgraph_msgs.msg import Clock


class Plotter(Node):

    def __init__(self):
        super().__init__("plotter_node")

        self.voxel_counts = [0]
        self.sub_voxel_count = self.create_subscription(
            Float64, "/voxel_count", self.unknown_voxel_callback, 10
        )

        self.nbv_time = [0]
        self.sub_nbv_time = self.create_subscription(
            Float64, "/nbv_time", self.nbv_time_callback, 10
        )

        self.simulation_times = [0]
        start_time = self.get_clock().now().seconds_nanoseconds()

        self.start_time = start_time[0] + start_time[1] * 1e-9

        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1)

        # Matplotlib initialization Plot 1
        (self.ln1,) = self.ax1.plot([], [], "b-")
        self.ax1.set_xlim(0, 3)
        self.ax1.set_ylim(0, 100)
        self.ax1.set_title("Total Explored Volume")
        self.ax1.set_xlabel("Planning Iterations")
        self.ax1.set_ylabel("Explored Volume [%]")

        # Plot 2
        (self.ln2,) = self.ax2.plot([], [], "r-")
        self.ax2.set_xlim(0, 3)
        self.ax2.set_ylim(0, 25000)
        self.ax2.set_title("Time to calculate NBV")
        self.ax2.set_xlabel("Planning Iterations")
        self.ax2.set_ylabel("ms")

        (self.ln3,) = self.ax3.plot([], [], "b-")
        self.ax3.set_xlim(0, 3)
        self.ax3.set_ylim(0, 100)
        self.ax3.set_title("Total Explored Volume vs Simulation Time")
        self.ax3.set_xlabel("Simulation Time [min]")
        self.ax3.set_ylabel("Explored Volume [%]")

    def update_plot(self, frame):
        if len(self.voxel_counts) > 3:
            self.ax1.set_xlim(0, len(self.voxel_counts))
        x_values1 = range(0, len(self.voxel_counts))
        self.ln1.set_data(x_values1, self.voxel_counts)

        if len(self.nbv_time) > 3:
            self.ax2.set_xlim(0, len(self.nbv_time))
        x_values2 = range(0, len(self.nbv_time))
        self.ln2.set_data(x_values2, self.nbv_time)

        if len(self.simulation_times) > 3:
            self.ax3.set_xlim(
                0, self.simulation_times[-1]
            )  # Extend the x-axis to fit time
        self.ln3.set_data(self.simulation_times, self.voxel_counts)

        return self.ln1, self.ln2, self.ln3

    def unknown_voxel_callback(self, msg):
        self.voxel_counts.append(msg.data)
        current_time = self.get_clock().now().seconds_nanoseconds()
        time_in_sec = current_time[0] + current_time[1] * 1e-9
        time_passed = time_in_sec - self.start_time
        time_in_min = time_passed / 60
        self.simulation_times.append(time_in_min)
        self.get_logger().info(
            f"Received voxel volume: {msg.data} and time is {time_in_min}"
        )

    def nbv_time_callback(self, msg):
        self.nbv_time.append(msg.data)
        self.get_logger().info(f"Received nbv_time data: {msg.data}")

    def plot_init(self):
        return self.ln1, self.ln2, self.ln3

    def ros_spin_once(self):
        """Spin ROS2 once to handle callbacks."""
        rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    plotter = Plotter()

    # Create the animation for the plot
    ani = FuncAnimation(
        plotter.fig, plotter.update_plot, init_func=plotter.plot_init, blit=False
    )

    # Use the Matplotlib timer to periodically call ros_spin_once and update the plot
    def update_ros_and_plot():
        plotter.ros_spin_once()  # Spin ROS once to handle incoming messages
        plt.draw()  # Force Matplotlib to update the plot

    # Call the `update_ros_and_plot` function every 100ms
    plotter.fig.canvas.new_timer(
        interval=100, callbacks=[(update_ros_and_plot, (), {})]
    ).start()

    # Start the Matplotlib event loop (non-blocking ROS spin with animation)
    plt.show()

    # Cleanup the ROS2 node after the plot is closed
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
