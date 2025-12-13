import typing
from commands2 import Command, Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState,Waypoint,IdealStartingState
from wpilib import DriverStation
from wpimath.kinematics import ChassisSpeeds;
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from subsystems.Drive.driveTrainGenerate import DrivetrainGenerator
#from wpimath.geometry import Pose2d
from math import pi


import math


class DriveToPose(Subsystem):
    def __init__(self,
                _target: typing.Callable[[], Pose2d],
                _robot: typing.Callable[[], Pose2d],
                 ) -> None:
        super().__init__()

        self.driveTrain = DrivetrainGenerator.getInstance()
        self.target = _target
        self.robot = _robot
        self.targetPose=self.target()
        self.robotPose=self.robot()
        self.running=False
        self.path = None

        self.constraints = PathConstraints(
            1.75, 
            2,
            pi,
            5*pi/4)




    def initializePath(self) -> Command:
        self.targetPose=self.target()
        self.robotPose=self.robot()

#        if DriverStation.getAlliance()== DriverStation.Alliance.kRed:
#            xt = 17.55 - self.robotPose.translation().X()
#            yt = 8.051 - self.robotPose.translation().Y()
#            self.robotPose = Pose2d(Translation2d(xt,yt),self.robotPose.rotation())
            

        self.start = Pose2d(
            self.robotPose.translation(), 
            self.getPathVelocityHeading(self.getFieldVelocity()))

        """""
            self.getPathVelocityHeading(self.getFieldVelocity(), self.targetPose))

        """        


        """
        self.command = AutoBuilder.pathfindToPose(
            self.targetPose,
            self.constraints)
        """
        listOfPoses = [Pose2d(self.robotPose.translation(),
                   self.getPathVelocityHeading(self.getFieldVelocity())), self.targetPose]
        self.waypoints = PathPlannerPath.waypointsFromPoses(listOfPoses) 
            

        
        iss =  IdealStartingState(
        self.getVelocityMagnitude(self.getFieldVelocity()), self.robotPose.rotation())
        print("iss V: ",iss.velocity, "  iss R: ",iss.rotation)

        self.path =  PathPlannerPath(
            self.waypoints, 
            self.constraints,iss, 
            GoalEndState(0, self.targetPose.rotation())
        )

        self.path.preventFlipping = True

        self.printStartState()

        return (
                (AutoBuilder.followPath(self.path))
                )
            


    def printFinalState(self) -> None:  
        print("CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC")
        print("end X ",self.driveTrain.get_state().pose.translation().X())
        print("end Y ",self.driveTrain.get_state().pose.translation().Y())        



    def printStartState(self) ->None:
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        print("start X ",self.driveTrain.get_state().pose.translation().X())
        print("start Y ",self.driveTrain.get_state().pose.translation().Y())        
        print("wp1 ",self.waypoints[0].anchor.X(),"  ", self.waypoints[0].anchor.Y())
        print("wp2 ",self.waypoints[1].anchor.X(),"  ", self.waypoints[1].anchor.Y())
        print("wp1 C",self.waypoints[0].nextControl.X(),"  ", self.waypoints[0].nextControl.Y())
        print("wp2 C",self.waypoints[1].prevControl.X(),"  ", self.waypoints[1].prevControl.Y())        


    

    def getPathVelocityHeading(self,cs: ChassisSpeeds):
        if self.getVelocityMagnitude(cs) < 0.25:
            diff = Translation2d(self.targetPose.translation().X()-self.robotPose.translation().X(),
                              self.targetPose.translation().Y()-self.robotPose.translation().Y())                                      
            if diff.norm() < 0.01:
                return self.targetPose.rotation()
            else:
                return diff.angle()        
        else: return Rotation2d(cs.vx, cs.vy)
    

    def getVelocityMagnitude(self, cs:ChassisSpeeds):
        self.driveTrain.get_state().speeds
        return Translation2d(cs.vx,cs.vy).norm()
    

    """
        // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
        // but not the reverse.  However, because this transform is a simple rotation, negating the
        // angle given as the robot angle reverses the direction of rotation, and the conversion is
        // reversed.
    """

    def getFieldVelocity(self):
        robotRelativeSpeeds = self.driveTrain.kinematics.toChassisSpeeds(
            self.driveTrain.get_state().module_states)
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, 
                                                    self.robotPose.rotation())
