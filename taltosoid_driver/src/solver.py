
'''
TODO documentation
'''

from constants import EE_LINK, JOINT_LIST

from lively_tk import Solver, PositionMatchObjective, OrientationMatchObjective, Translation, Rotation, \
                      CollisionAvoidanceObjective, JointLimitsObjective, VelocityMinimizationObjective, \
                      AccelerationMinimizationObjective, JerkMinimizationObjective, JointMatchObjective


def read_urdf_from_file(file_name):
    try:
        with open(file_name,'r') as fin:
            data = fin.read()
    except:
        data = None
    return data

def create_solver(urdf_string):

    poseObjvs = [
        PositionMatchObjective(name="EE Position", weight=20, link=EE_LINK),
        OrientationMatchObjective(name="EE Rotation", weight=6, link=EE_LINK)
    ]

    jointObjvs = []
    for joint_name in JOINT_LIST:
        jointObjvs.append(JointMatchObjective(name="Joint Match {}".format(joint_name), weight="0", joint=joint_name))

    smoothObjvs = [
        VelocityMinimizationObjective(name="Velocity Minimization", weight=1),
        AccelerationMinimizationObjective(name="Acceleration Minimization", weight=1),
        JerkMinimizationObjective(name="Jerk Minimization", weight=1),
        JointLimitsObjective(name="Joint Limits", weight=1)
    ]

    perlinObjvs = [
        
    ]

    solver = Solver(
        urdf_string,
        [
            *poseObjvs,
            *jointObjvs,
            *smoothObjvs,
            *perlinObjvs,
            CollisionAvoidanceObjective(name="Collision Avoidance", weight=1)
        ]
    )

    #TODO figure out perlin objectives
    #TODO figure out how to do collisions
    #TODO default state?

    print(solver.objectives)

    return solver

def run_solver(solver, pose, weights=None):
    state = solver.solve(goals=[
        Translation(pose.position.x, pose.position.y, pose.position.z),
        Rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
    ], time=0.0)

    #TODO figure out how to control weights?

    print(state)

    #TODO process this state thing

    return state