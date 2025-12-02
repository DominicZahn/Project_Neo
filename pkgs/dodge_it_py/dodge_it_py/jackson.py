import pinocchio as pin
from pinocchio import casadi as ca_pin
from acados_template import AcadosOcp
import casadi as ca

import sys

def main() -> int:
    model = pin.buildModelFromUrdf("/home/robot/ws/pkgs/ros2_heinz/h1_gazebo_sim/ros_gz_h1_description/models/h1_ign/h1_2.urdf")
    c_model = ca_pin.Model(model)
    c_data = c_model.createData()

    print('FINISHED')
    return 0


if __name__ == "__main__":
    sys.exit(main())
