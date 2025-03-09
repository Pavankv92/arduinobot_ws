import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter


class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")
        self.declare_parameter(name="simple_int_param", value=28)
        self.declare_parameter(name="simple_str_param", value="Pavan")

        # its like a trace on parameters. this function is called when the param value changed
        self.add_on_set_parameters_callback(self.param_change_change_callback)

    
    # this callback receives a list of all the params that are changed : params
    def param_change_change_callback(self, params):
        result = SetParametersResult()

        for param in params:
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"Param: simple_int_param changed to {param.value}")
                result.successful = True
            if param.name == "simple_str_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"Param: simple_str_param changed to {param.value}")
                result.successful = True

        return result
    
def main():
    rclpy.init()
    simple_parameter_node = SimpleParameter()
    rclpy.spin(simple_parameter_node)
    simple_parameter_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()





