from wpimath.geometry import Pose2d
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

class TargetePose():
    targetPose = Pose2d(Translation2d(3,3),Rotation2d(1.57))


    @staticmethod
    def setTargetPose(pose: Pose2d ):
        TargetePose.targetPose = pose

    
    @staticmethod
    def getTargetPose():
        return TargetePose.targetPose


    def __init__(self) -> None:
        ""

