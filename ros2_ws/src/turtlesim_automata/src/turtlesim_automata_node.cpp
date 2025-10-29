#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cstdlib>
#include <ctime>
#include <cmath>

// Defines the TurtlesimAutomata class, inheriting from rclcpp::Node
class TurtlesimAutomata : public rclcpp::Node
{
public:
    // Constructor for TurtlesimAutomata
    TurtlesimAutomata() : Node("turtlesim_automata")
    {
    	rotating = true;
    	walk = false; // used for moving the turtle
    	period = 1.f;
    	
        // Creates a publisher for sending velocity commands to the turtle
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
        // Creates a subscription to the turtle's position
        subscription = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtlesimAutomata::updateCurrentPosition, this, std::placeholders::_1));
        
        // Creates a timer to periodically call the move_turtle method
        timer = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&TurtlesimAutomata::move_turtle, this)); // Timer interval set to 50ms
        
        // Seeds the random number generator
        std::srand(std::time(nullptr));
        
        // REMOVED THE LOOP - TELEPORT SERVICE SINCE IT WAS BREAKING DOWN THE CHANGE OF DIRECTION AT THE BEGINNING OF THE SIMULATION
        
       
        RCLCPP_INFO(this->get_logger(), "Waiting for teleport service...");
       
	
	// Initializing the turtle with random direction
        float random_angle = 3.f + static_cast<float>(std::rand()) / RAND_MAX * 2.0 * M_PI; // Random angle between 0 and 2*pi
        RCLCPP_INFO(this->get_logger(), "Rotate to random direction %f...", random_angle);
        current_twist.linear.x = 0.f;//std::cos(random_angle); // Linear velocity in x-direction
        current_twist.linear.y = 0.f;//std::sin(random_angle); // Linear velocity in y-direction
        current_twist.angular.z = random_angle * 200.0;//0.0; // No initial angular velocity
        
	//period += static_cast<float>(std::rand()) / RAND_MAX * 3.0;
	
	current_position.theta = random_angle;  // tried many possible  cases to rotate the random angle but the mysterious machine is acting unpredictably  but not random and even worsening the simulation sometimes
      
    }

private:
    // updating the current position 
    void updateCurrentPosition(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_position = *msg;
    }

    // Method to control the turtle's movement
    void move_turtle()
    {
        // If there is an ongoing action, decrements its duration
        if (period > 0)
        {
            period --;
            // When action duration ends
            
            if (period == 0)
            {
                if (rotating)
                {
                    // Finishes rotating and start moving away from the edge
                    rotating = false;
                    walk = true;
                    current_twist.linear.x = 1.0;  // Moves away from the edge
                    current_twist.angular.z = 0.0; // Stops rotating
                    period = 10; // Moves away for 0.5 seconds (10 * 50ms)
                }
                else if (walk)
                {
                    // Finishes moving away and resume normal movement
                    walk = false;
                    current_twist.linear.x = 2.0;  // Resumes normal forward movement
                }
            }
        }
        // Checks if the turtle is near the edge
        else if (near_edge())
        {
            if (!rotating && !walk)
            {
                // If near the edge and not already rotating or moving away, starts rotating
                RCLCPP_INFO(this->get_logger(), "Edge detected!");
                rotating = true;
                current_twist.linear.x = 0.0;   // Stops moving forward
                current_twist.angular.z = -1.5708; // Rotates approximately 90 degrees (angular velocity)
                period = 20; // Rotates for 1 second (20 * 50ms)
            }
        }
        else
        {
            // Ensures the turtle doesn't go out of bounds
            current_twist.linear.x = keep_in_bounds(current_twist.linear.x, current_position.x, 'x');
            current_twist.linear.y = keep_in_bounds(current_twist.linear.y, current_position.y, 'y');
        }

        // Publish the velocity command
        publisher->publish(current_twist);
    }
    

    // Checks if the turtle is near the edge of the screen
    bool near_edge()
    {
        const double boundary = 0.3;
        double is_near = ((current_position.x < boundary) || (current_position.x > 11.0 - boundary) || (current_position.y < boundary) || (current_position.y > 11.0 - boundary));
        
        return is_near;
    }

    // Ensures the turtle's velocity keeps it within bounds
    double keep_in_bounds(double velocity, double position, char axis)
    {
        const double min_boundary = 0.3;
        const double max_boundary = 10.7;
        const double slow_down= 1.0;

        // Stops if moving out of bounds
        if ((position < min_boundary && velocity < 0) || (position > max_boundary && velocity > 0))
        {
            return 0.0;
        }

        // Slows down when near the edges
        if (axis == 'x')
        {
            if (position < min_boundary + slow_down && velocity < 0)
            {
                return 0.5; // Slows down when near left edge
            }
            else if (position > max_boundary - slow_down && velocity > 0)
            {
                return 0.5; // Slows down when near right edge
            }
        }
        else if (axis == 'y')
        {
            if (position < min_boundary + slow_down && velocity < 0)
            {
                return 0.5; // Slows down when near bottom edge
            }
            else if (position > max_boundary - slow_down && velocity > 0)
            {
                return 0.5; // Slows down when near top edge
            }
        }

        return velocity;
    }
	// Member variables for publishers, subscribers, timers, and state
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;  // Publisher for sending velocity commands to the turtle
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription;  // Subscription to receive turtle's pose information
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client;  // Client for calling teleport service to move the turtle
    rclcpp::TimerBase::SharedPtr timer;  // Timer for periodic calls to move_turtle method
    turtlesim::msg::Pose current_position;  // Current pose (position and orientation) of the turtle
    geometry_msgs::msg::Twist current_twist;  // Current velocity command for the turtle
    
    bool rotating;  // Flag indicating if the turtle is currently rotating
    bool walk;  // Flag indicating if the turtle is currently moving away from the edge
    int period;  // Duration (in timer ticks) for current action (rotating or moving away)

};

// Main function to run the TurtlesimAutomata node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // Initialize the ROS 2 node with command-line arguments
    rclcpp::spin(std::make_shared<TurtlesimAutomata>());  // Start spinning the node with TurtlesimAutomata instance
    rclcpp::shutdown();  // Shutdown the ROS 2 node
    
    return 0;  // Exit the main function with success

}

