import rclpy
from rclpy.node import Node
import serial
import time

from npk_sensor.srv import GetSoilData


# ROS2 node that communicates with the NPK sensor over RS485/Modbus
# and provides the sensor readings through a ROS service
class SoilSensorNode(Node):

    def __init__(self):
        super().__init__('soil_sensor_node')

        # Open the serial connection to the sensor
        self.ser = serial.Serial(
            '/dev/ttyAMA0',
            9600,
            timeout=1
        )

        # Create the service that other ROS nodes can call
        # to get the current soil sensor readings
        self.srv = self.create_service(
            GetSoilData,
            'get_soil_data',
            self.handle_request
        )

        self.get_logger().info("Soil Sensor Service Ready")

    # Send a Modbus command to the sensor and return its response
    def send_request(self, cmd_hex):
        try:
            self.ser.write(bytes.fromhex(cmd_hex))

            # Give the sensor time to respond
            time.sleep(0.1)

            return self.ser.read(11)

        except Exception as e:
            self.get_logger().error(f"Comm error: {e}")
            return None

    # Read a single value from the sensor response
    # The divisor is used for values that the sensor reports with scaling
    def parse_value(self, data, div=1):
        if data and len(data) >= 7:
            val = int.from_bytes(
                data[3:5],
                byteorder='big'
            )

            return val / div

        return None

    # This runs whenever another ROS node calls the soil data service
    def handle_request(self, request, response):

        # Read pH
        res_ph = self.send_request(
            "01 03 00 06 00 01 64 0B"
        )
        ph = self.parse_value(res_ph, 100)

        # Read humidity
        res_h = self.send_request(
            "01 03 00 12 00 01 24 0F"
        )
        hum = self.parse_value(res_h, 10)

        # Read temperature
        res_t = self.send_request(
            "01 03 00 13 00 01 75 CF"
        )
        temp = self.parse_value(res_t, 10)

        # Read conductivity
        res_ec = self.send_request(
            "01 03 00 15 00 01 95 CE"
        )
        ec = self.parse_value(res_ec, 1)

        # Read nitrogen, phosphorus, and potassium
        # All three values are returned in the same response
        res_npk = self.send_request(
            "01 03 00 1E 00 03 65 CD"
        )

        n = p = k = None

        if res_npk and len(res_npk) >= 11:
            n = int.from_bytes(
                res_npk[3:5],
                byteorder='big'
            )

            p = int.from_bytes(
                res_npk[5:7],
                byteorder='big'
            )

            k = int.from_bytes(
                res_npk[7:9],
                byteorder='big'
            )

        # Put the sensor readings into the ROS response
        # -1.0 is returned if a reading could not be obtained
        response.ph = float(ph) if ph is not None else -1.0
        response.moisture = float(hum) if hum is not None else -1.0
        response.temperature = float(temp) if temp is not None else -1.0
        response.conductivity = float(ec) if ec is not None else -1.0
        response.nitrogen = float(n) if n is not None else -1.0
        response.phosphorus = float(p) if p is not None else -1.0
        response.potassium = float(k) if k is not None else -1.0

        self.get_logger().info("Soil data returned")

        return response

    # Close the serial connection when the node shuts down
    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SoilSensorNode()

    # Keep the node running and wait for service requests
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
