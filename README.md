# user_input_turtle_shape 

## **Description**

**Turtle Shapes** utilizes user input for the turtle to begin drawing shapes in the `turtlesim` simulator. After completing each drawing, the turtle can be reset using the `clear` service, preparing it for the next shape. This interactive program showcases user input handling and turtle movement control in a fun and engaging way.

## **Features**

- User input for shape selection
- Turtle movement control to draw shapes
- Reset functionality using the `clear` service

## **Technologies Used**

- ROS 2 (Robot Operating System)
- Python

## **Getting Started**

### **Prerequisites**

- ROS 2 installed on your machine
- `turtlesim` package

### **Installation**

##**1. Clone the repository:**
   ```bash
   git clone https://github.com/yourusername/turtle_shapes.git
##**2. Navigate to the project directory:**

cd turtle_shapes

## **3. Build the package in your ROS 2 workspace:**

colcon build

##**4. Source your workspace:** 

source install/setup.bash

### **Usage**

##**Launch the turtlesim node:**

ros2 run turtlesim turtlesim_node

##**Run the Turtle Shapes program:**

ros2 run turtle_shapes your_node_name

Follow the prompts to enter a shape for the turtle to draw.
