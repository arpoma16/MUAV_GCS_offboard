#!/usr/bin/env python3
# this program receives mission from gcs and execute it in offboard mode
# the mission waypoints is (lat , long , altitude_AGL) it uses goto setpoint to go to waypoints

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TelemetryStatus,OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,VehicleLocalPosition, VehicleGlobalPosition, VehicleStatus,GotoSetpoint,SensorGps
from std_srvs.srv import Trigger, SetBool
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv
from muav_gcs_interfaces.srv import LoadMission
from enum import Enum
from muav_gcs_offboard.coordinate_transform import gps_to_ned

def fmt_float(val):
    """Format a float to 2 decimal places as string."""
    return f"{val:.2f}"

class UAVState(Enum):
    """Enumeration for UAV states."""
    IDLE = 0
    TAKEOFF = 1
    LANDING = 2
    FLYING = 3
    EMERGENCY = 4
    
class YawMode(Enum):
    """Enumeration for Yaw modes."""
    FIXED_YAW = 0
    FACE_PATH = 1
    FREE_YAW = 2
    WP_YAW = 3
    
    
class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self, drone_id=3, ns='px4_3') -> None:
        super().__init__(f'offboard_control_{drone_id}')

        # Declare and retrieve the namespace parameter
        self.declare_parameter('namespace', '')  # Default to empty namespace
        self.namespace = self.get_parameter('namespace').value
        self.ns = f'/{self.namespace}' if self.namespace else ''
        
        self.declare_parameter('MAV_SYS_ID', 1)
        self.drone_id = int( self.get_parameter('MAV_SYS_ID').value)
        #self.ns = ns
        self.get_logger().info(f"Drone {self.ns} mav sys id {self.drone_id}")



        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{self.ns}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{self.ns}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{self.ns}/fmu/in/vehicle_command', qos_profile)
        self.goto_setpoint_publisher = self.create_publisher(
            GotoSetpoint, f'{self.ns}/fmu/in/goto_setpoint', qos_profile)
        



        # Create subscribers of vehicle topics
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{self.ns}/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_global_position_subscriber = self.create_subscription(
            TelemetryStatus, f'{self.ns}/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{self.ns}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_cmdResponse_subscriber = self.create_subscription(
            VehicleCommandAck, f'{self.ns}/fmu/out/vehicle_command_ack', self.vehicle_cmdResponse_callback, qos_profile)
        self.vehicle_gps_subscriber = self.create_subscription(
            SensorGps, f'{self.ns}/fmu/out/vehicle_gps_position', self.vehicle_gps_callback, qos_profile)
        # offboard topics
        self.gcs_logger_subscriber = self.create_subscription(
            TrajectorySetpoint, f'{self.ns}/offboard/in/trajectory_setpoint', self.gcs_position_callback, qos_profile)
        self.vehicle_command_offboard_subscriber = self.create_subscription(
            VehicleCommand, f'{self.ns}/offboard/in/vehicle_command', self.vehicle_cmd_offboard_callback, qos_profile)
        self.vehicle_command_offboard_ack_publisher = self.create_publisher(
            VehicleCommandAck, f'{self.ns}/offboard/out/vehicle_command_ack', qos_profile)
                # Create services for takeoff and landing
        self.vehicle_cmd_offboard_service = self.create_service(
            LoadMission, f'{self.ns}/offboard/mission_load', self.handle_load_mission_request)
        self.vehicle_cmd_offboard_service = self.create_service(
            SetBool, f'{self.ns}/offboard/mission/start_stop', self.handle_vehicle_start_stop_mission_request)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_global_position = VehicleGlobalPosition()
        self.vehicle_status = VehicleStatus()
        self.gps_position = SensorGps()
        self.takeoff_height = -5.0
        self.default_takeoff_height = -5.0  # Default takeoff height
        self.command_position = [0.0, 0.0, self.takeoff_height]
        self.commanded_position = [0.0, 0.0, 0.0]
        self.takeoff_point = [0.0, 0.0, 0.0]
        self.wp = [[5.0, 0.0, -10.0], [5.0, 5.0, -10.0], [0.0, 5.0, -10.0], [0.0, 0.0, -10.0]]
        self.yaw = [0.0, 0.0, 0.0, 0.0]
        self.wp_index = 0
        self.yaw_mode = YawMode.FACE_PATH
        self.uav_state = UAVState.IDLE
        self.start = False


        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Timer for control commands created (100ms).')

    def handle_vehicle_start_stop_mission_request(self, request, response):
        """Handle vehicle start/stop mission requests."""
        self.start = request.data
        if self.start:
            self.get_logger().info("Starting offboard mission.")
        else:
            self.get_logger().info("Stopping offboard mission.")
        response.success = True
        return response

    def handle_load_mission_request(self, request, response):
        """Handle incoming vehicle command requests."""
        self.get_logger().info(f"Received mission load request")
        self.wp = []
        self.yaw = []
        gps_position = [0.0, 0.0, 0.0]  # Default GPS position if not availabl
        gps_position[0] = self.gps_position.latitude_deg
        gps_position[1] = self.gps_position.longitude_deg
        gps_position[2] = self.gps_position.altitude_msl_m

        # Verificar si yaw_mode está presente y es válido, usar FACE_PATH por defecto
        if hasattr(request.request, 'yaw_mode') and request.request.yaw_mode is not None:
            try:
                self.yaw_mode = YawMode(request.request.yaw_mode)
                self.get_logger().info(f"Yaw mode configurado: {self.yaw_mode.name}")
            except ValueError:
                self.get_logger().warn(f"Valor de yaw_mode inválido: {request.request.yaw_mode}, usando FACE_PATH por defecto")
                self.yaw_mode = YawMode.FACE_PATH
        else:
            self.get_logger().warn("yaw_mode no especificado, usando FACE_PATH por defecto")
            self.yaw_mode = YawMode.FACE_PATH

        # Procesar waypoints
        for i, wp in enumerate(request.request.waypoint):
            ned = gps_to_ned(wp.latitude, wp.longitude, wp.altitude,
                              gps_position[0], gps_position[1], gps_position[2])
            self.wp.append(ned)

        # Procesar valores de yaw del Float64MultiArray
        if hasattr(request.request, 'yaw') and request.request.yaw is not None and len(request.request.yaw.data) > 0:
            for i in range(len(request.request.waypoint)):
                if i < len(request.request.yaw.data):
                    self.yaw.append(request.request.yaw.data[i])
                else:
                    self.yaw.append(float('nan'))
                    self.get_logger().warn(f"Waypoint {i+1} sin yaw definido, se usará modo de yaw por defecto")
        else:
            # Si no hay datos de yaw, usar NaN para todos
            for i in range(len(request.request.waypoint)):
                self.yaw.append(float('nan'))

        for i, waypoint in enumerate(self.wp):
            yaw_str = f"{fmt_float(self.yaw[i])}" if not math.isnan(self.yaw[i]) else "auto"
            self.get_logger().info(f"Waypoint {i+1}: NED({fmt_float(waypoint[0])}, {fmt_float(waypoint[1])}, {fmt_float(waypoint[2])}) Yaw: {yaw_str}")

        response.success = True
        self.get_logger().info("Mission loaded successfully with {} waypoints.".format(len(self.wp)))
        return response
    def vehicle_gps_callback(self, gps_position):
        """Callback function for vehicle_gps_position topic subscriber."""
        self.gps_position = gps_position
        if gps_position.fix_type < 3:  # 3D fix
            self.get_logger().warn("No valid GPS fix. Using default takeoff point.")
    def vehicle_cmd_offboard_callback(self, vehicle_command):
        """Callback function for vehicle_command_offboard topic subscriber."""
        self.get_logger().info(f"Rcv vehicle command offboard: {vehicle_command.command} param1: {vehicle_command.param1} param2: {vehicle_command.param2} param3: {vehicle_command.param3} param4: {vehicle_command.param4} param5: {vehicle_command.param5} param6: {vehicle_command.param6} param7: {vehicle_command.param7}")
        self.get_logger().info(f"Command {vehicle_command.command} not handled in offboard control")
        self.publish_vehicle_command(vehicle_command.command, param1=vehicle_command.param1, param2=vehicle_command.param2, param3=vehicle_command.param3, param4=vehicle_command.param4, param5=vehicle_command.param5, param6=vehicle_command.param6, param7=vehicle_command.param7)


    def gcs_position_callback(self, gcs_position):
        """Callback function for GCS position topic subscriber."""
        # gcs_position.position is already a list of 3 floats [x, y, z]
        self.command_position[0] = float(gcs_position.position[0])
        self.command_position[1] = float(gcs_position.position[1])
        self.command_position[2] = float(gcs_position.position[2])


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
    def vehicle_global_position_callback(self, vehicle_global_position):
        """Callback function for vehicle_global_position topic subscriber."""
        self.vehicle_global_position = vehicle_global_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def vehicle_cmdResponse_callback(self, vehicle_cmdResponse):
        """Callback function for vehicle_command_ack topic subscriber."""
        self.get_logger().info(f"Cmd response: {vehicle_cmdResponse.command} result: {vehicle_cmdResponse.result}")
        if vehicle_cmdResponse.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.vehicle_command_offboard_ack_publisher.publish(vehicle_cmdResponse)
            if vehicle_cmdResponse.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info("Vehicle armed successfully")
            else:
                self.get_logger().warn("Vehicle failed to arm")
        if vehicle_cmdResponse.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF:
            self.vehicle_command_offboard_ack_publisher.publish(vehicle_cmdResponse)
            if vehicle_cmdResponse.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info("Takeoff command accepted")
            else:
                self.get_logger().warn("Takeoff command failed")
                self.uav_state = UAVState.IDLE
        if vehicle_cmdResponse.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.vehicle_command_offboard_ack_publisher.publish(vehicle_cmdResponse)
            if vehicle_cmdResponse.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info("Land command accepted")
            else:
                self.get_logger().warn("Land command failed")
                self.uav_state = UAVState.FLYING
    

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def takeoff(self, altitude: float = 5.0):
        """Send a takeoff command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,param1=1.0,param7=altitude)
        self.get_logger().info(f"Takeoff command sent with altitude: {altitude}")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float,heading: float=1.57079,vx:float =float('nan'), vy:float =float('nan'), vz:float =float('nan')) -> None:
        """Publish the trajectory setpoint."""
        self.commanded_position[0] = x
        self.commanded_position[1] = y
        self.commanded_position[2] = z

        msg = TrajectorySetpoint()
        # Ensure all values are explicitly float type
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yaw = heading
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
    
    def publish_goto_setpoint(self, x: float, y: float, z: float, heading: float=1.57079, vel_xy: float = 5.0, vel_z: float = 2.0) -> None:
        self.get_logger().info(f"Goto setpoint: pos=[{x:.2f}, {y:.2f}, {z:.2f}] heading={heading:.2f} vel_xy={vel_xy:.2f} vel_z={vel_z:.2f}")
        msg = GotoSetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.heading = heading # (optional) [rad] [-pi,pi] from North
        msg.max_horizontal_speed = vel_xy
        msg.max_vertical_speed = vel_z
        msg.max_heading_rate = 10.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if YawMode.WP_YAW == self.yaw_mode or YawMode.FACE_PATH == self.yaw_mode:
            msg.flag_control_heading = True
        
        self.goto_setpoint_publisher.publish(msg)
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.drone_id 
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def getYaw(self, wp_index):
        """Calculate dynamic yaw based on movement direction."""
        if self.yaw_mode  == YawMode.FACE_PATH:
                x = self.wp[self.wp_index][0]
                y = self.wp[self.wp_index][1]

                dx = x - self.vehicle_local_position.x
                dy = y - self.vehicle_local_position.y

                dinamic_yaw= math.atan2(dy, dx)

                return dinamic_yaw
        if self.yaw_mode == YawMode.WP_YAW:
            # Verificar que el indice sea valido y que el yaw no sea NaN
            if 0 <= wp_index < len(self.yaw):
                yaw_value = self.yaw[wp_index]*(math.pi/180)  # Convertir a radianes
                if not math.isnan(yaw_value):
                    my_yaw = yaw_value
                    self.get_logger().info(f"yaw waypoint {wp_index}: {my_yaw} radianes ({self.yaw[wp_index]} grados)")
                    return yaw_value
                else:
                    # Si el yaw es NaN, calcular yaw dinamico
                    self.get_logger().warn(f"Yaw no definido para waypoint {wp_index}, usando FACE_PATH")
                    x = self.wp[wp_index][0]
                    y = self.wp[wp_index][1]
                    dx = x - self.vehicle_local_position.x
                    dy = y - self.vehicle_local_position.y
                    return math.atan2(dy, dx)
            else:
                self.get_logger().error(f"Indice de waypoint invalido: {wp_index}")
        return 1.57079  # Default yaw (90 degrees)
        
    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        #self.get_logger().info(
        #    f"M: {self.uav_state} - s: {self.vehicle_status.nav_state} - Cmded: [{fmt_float(self.commanded_position[0])}, {fmt_float(self.commanded_position[1])}, {fmt_float(self.commanded_position[2])}] "
        #    f"- Real: [{fmt_float(self.vehicle_local_position.x)}, {fmt_float(self.vehicle_local_position.y)}, {fmt_float(self.vehicle_local_position.z)}]"
        #)

        if self.uav_state == UAVState.IDLE:
            if self.start:
                self.uav_state = UAVState.TAKEOFF
            #self.publish_position_setpoint(0.0, 0.0, 0.0)
            #self.get_logger().info("UAV is in IDLE state, holding position at (0,0,0)")

        if self.uav_state == UAVState.TAKEOFF:
            #self.get_logger().info("Takeoff command received, moving to takeoff height")
            if self.offboard_setpoint_counter == 10 :
                self.get_logger().info("Engaging offboard mode and arming the vehicle")
                self.engage_offboard_mode()
            if self.offboard_setpoint_counter == 11 :
                self.arm()
                self.publish_goto_setpoint(self.takeoff_point[0], self.takeoff_point[1], - 1.0, vel_xy=1.0, vel_z=0.5 )
            
            if self.offboard_setpoint_counter == 14 :
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                    self.get_logger().info("Vehicle is in OFFBOARD mode and armed, proceeding with takeoff")
                else:
                    self.get_logger().warn("Vehicle is not in OFFBOARD mode or not armed yet, waiting...")

            if self.offboard_setpoint_counter > 20 :
                self.publish_goto_setpoint(self.takeoff_point[0], self.takeoff_point[1], -10 )

            if self.vehicle_local_position.z < -10 + 0.5:
                self.get_logger().info("Takeoff completed")
                self.uav_state = UAVState.FLYING

            if self.offboard_setpoint_counter < 90:
                self.offboard_setpoint_counter += 1

        if self.uav_state == UAVState.FLYING:

            x = self.wp[self.wp_index][0]
            y = self.wp[self.wp_index][1]
            z = self.wp[self.wp_index][2]

            if math.sqrt((self.vehicle_local_position.x - x)**2 + (self.vehicle_local_position.y - y)**2 + (self.vehicle_local_position.z - z)**2) <= 0.3:
                self.get_logger().info(f"Reached waypoint {self.wp_index+1} at ({x}, {y}, {z})")
                if self.wp_index < len(self.wp)-1:
                    self.wp_index += 1
                else:
                    self.wp_index = 0
                    self.uav_state = UAVState.LANDING

            # Calcular heading dinámico basado en la dirección de movimiento
            yaw = self.getYaw(self.wp_index)

            self.publish_goto_setpoint(x, y, z, heading=yaw)

        if self.uav_state == UAVState.LANDING:
            if (abs(self.vehicle_local_position.x) > 0.5 or abs(self.vehicle_local_position.y) > 0.5 or self.vehicle_local_position.z > self.takeoff_height + 0.5):
                self.get_logger().info(
                    "Waiting to reach landing position (0,0,{}) | Current position: x={:.2f}, y={:.2f}, z={:.2f}".format(
                        self.takeoff_height,
                        self.vehicle_local_position.x,
                        self.vehicle_local_position.y,
                        self.vehicle_local_position.z
                    )
                )
                self.publish_goto_setpoint(0.0, 0.0, self.takeoff_height)
            else:
                self.get_logger().info("Landing...")
                self.land()
                self.uav_state = UAVState.IDLE
                self.offboard_setpoint_counter = 0
                exit(0)
                

        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.uav_state == UAVState.FLYING:
            self.uav_state = UAVState.EMERGENCY
            self.get_logger().info("Emergency: Vehicle exited OFFBOARD mode during flight")

        if self.uav_state == UAVState.EMERGENCY:
            self.publish_goto_setpoint(0.0, 0.0, self.takeoff_height)




def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
