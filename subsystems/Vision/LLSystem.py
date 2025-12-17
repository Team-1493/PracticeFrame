from typing import List
from commands2 import Command, Subsystem
from phoenix6 import utils
from Utilities.LLH import LimelightHelpers
from Utilities.LLH import PoseEstimate
from Utilities.LLH import RawFiducial
from phoenix6 import utils
from wpilib import DriverStation
from robotstate import RobotState
from Constants import ConstantValues
from wpilib import SmartDashboard

from subsystems.Drive.driveTrainGenerate import DrivetrainGenerator

class llSystem(Subsystem):
    def __init__(self):
        self.robotState = RobotState.getInstance()
        self.driveTrain = DrivetrainGenerator.getInstance()
        self.Lconstants = ConstantValues.LeftLimelightConstants
        self.Rconstants = ConstantValues.RightLimelightConstants 

        self.max_value = 9999

        self.xyStdDevCoefficient = 0.02
        self.thetaStdDevCoefficient = self.max_value  #0.04

        self.previousLeftEstimate = PoseEstimate()
        self.previousRightEstimate = PoseEstimate()

        self.PE = PoseEstimate()

        self.configfureLimelights()


    def configfureLimelights(self):

        LimelightHelpers.set_camerapose_robotspace(
                self.Lconstants.CAMERA_NAME,
                self.Lconstants.X_OFFSET, 
                self.Lconstants.Y_OFFSET,
                self.Lconstants.Z_OFFSET,
                self.Lconstants.THETA_X_OFFSET,
                self.Lconstants.THETA_Y_OFFSET,
                self.Lconstants.THETA_Z_OFFSET)
        
        LimelightHelpers.set_camerapose_robotspace(
                self.Rconstants.CAMERA_NAME,
                self.Rconstants.X_OFFSET, 
                self.Rconstants.Y_OFFSET,
                self.Rconstants.Z_OFFSET,
                self.Rconstants.THETA_X_OFFSET,
                self.Rconstants.THETA_Y_OFFSET,
                self.Rconstants.THETA_Z_OFFSET)   

        LimelightHelpers.set_imu_mode(self.Lconstants.CAMERA_NAME, 0)
        LimelightHelpers.set_imu_mode(self.Rconstants.CAMERA_NAME, 0)    
    

    
    def update(self): 

        """ check if we are moving too fast for an accurate camera measurement """
        shouldAccept = (self.robotState.getChassisSpeedsNorm()<3 
                        and abs(self.robotState.getRotationalSpeedsRPS())<2)
        SmartDashboard.putBoolean("Accept Target",shouldAccept)

        """ set the standard deviations for use in the pose estimator """
        leftStdDev = self.max_value
        leftHeadingStdDev = self.max_value
        rightStdDev = self.max_value
        rightHeadingStdDev = self.max_value

        leftEstimate=PoseEstimate()
        rightEstimate=PoseEstimate()

        
        leftEstimate = self.pollLL(self.Lconstants.CAMERA_NAME, self.previousLeftEstimate)
        #rightEstimate = self.pollLL(self.Rconstants.CAMERA_NAME, self.previousRightEstimate)
        

        if shouldAccept and LimelightHelpers.get_tv(self.Lconstants.CAMERA_NAME):
            print ("A")
            if (leftEstimate is not None and len(leftEstimate.raw_fiducials) > 0):
                closestID,closestTagDist = self.minDist(leftEstimate.raw_fiducials)
                leftStdDev = self.xyStdDevCoefficient * (closestTagDist ** 2) / leftEstimate.tag_count
                self.leftHeadingStdDev = self.thetaStdDevCoefficient * (closestTagDist ** 2) / leftEstimate.tag_count
                
                if leftEstimate.avg_tag_dist > 3.:
                    leftStdDev =self.max_value

                print("B")
                SmartDashboard.putNumber("LL-L closest ID",closestID)    
                SmartDashboard.putNumber("LL-L closest Dist",closestTagDist)    
#            else:
#                SmartDashboard.putNumber("LL-L closest ID",-1)    
#                SmartDashboard.putNumber("LL-L closest Dist",-1)    


            if (rightEstimate is not None and len(rightEstimate.raw_fiducials) > 0):
                closestID,closestTagDist = self.minDist(rightEstimate.raw_fiducials)
                rightStdDev = self.xyStdDevCoefficient * (closestTagDist ** 2) / rightEstimate.tag_count
                self.rightHeadingStdDev = self.thetaStdDevCoefficient * (closestTagDist ** 2) / rightEstimate.tag_count
                
                if rightEstimate.avg_tag_dist > 3.0:
                    rightStdDev =self.max_value
                
                SmartDashboard.putNumber("LL-R closest ID",closestID)    
                SmartDashboard.putNumber("LL-R closest Dist",closestTagDist)    

#            else:
#                SmartDashboard.putNumber("LL-R closest ID",-1)    
#                SmartDashboard.putNumber("LL-R closest Dist",-1)    

            """""
            if leftStdDev < rightStdDev:
                self.driveTrain.add_vision_measurement(
                        leftEstimate.pose,
                        utils.fpga_to_current_time(leftEstimate.timestamp_seconds),
                        (leftStdDev, leftStdDev, leftHeadingStdDev))
                
            elif rightStdDev < leftStdDev:
                self.driveTrain.add_vision_measurement(
                        rightEstimate.pose,
                        utils.fpga_to_current_time(rightEstimate.timestamp_seconds),
                        (rightStdDev, rightStdDev, rightHeadingStdDev))
            """            
                
#        else:
#            SmartDashboard.putNumber("LL-L closest ID",-1)    
#            SmartDashboard.putNumber("LL-L closest Dist",-1)    
#            SmartDashboard.putNumber("LL-R closest ID",-1)    
#            SmartDashboard.putNumber("LL-R closest Dist",-1)    





    def minDist(self,rf:List[RawFiducial]):
        minD=9999
        minID=0
        iMax=len(rf)
        i=0
        while i<iMax:
            if rf[i].dist_to_camera<minD:
                minID=i
                minD=rf[i].dist_to_camera
                i=i+1
        return rf[minID].id,minD




    def pollLL(self,id,previousEstimate: PoseEstimate): 
        LimelightHelpers.set_robot_orientation(
                id, self.robotState.getRotationDeg(), 0, 0, 0, 0, 0)
        if (LimelightHelpers.get_tv(id)):
            if previousEstimate is not None:
                oldTimestamp =  previousEstimate.timestamp_seconds 
            else:
                oldTimestamp = self.max_value
                
            newEstimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(id)
            
            if newEstimate is not None:
                
                if newEstimate.timestamp_seconds == oldTimestamp:
                    newEstimate = None
                else:
                    previousEstimate = newEstimate
        else:
            newEstimate = None 

        return newEstimate
                


    def pollLLMT1(self,id,previousEstimate: PoseEstimate):

        if (LimelightHelpers.get_tv(id)):
            if previousEstimate is not None:
                oldTimestamp =  previousEstimate.timestamp_seconds 
            else:
                oldTimestamp = self.max_value
                
            newEstimate = LimelightHelpers.get_botpose_estimate_wpiblue(id)
            
            if newEstimate is not None:
                
                if newEstimate.timestamp_seconds == oldTimestamp:
                    newEstimate = None
                else:
                    previousEstimate = newEstimate
        else:
            newEstimate = None 
                       
        return newEstimate            
        

        
    def periodic(self):
        self.update()
                        

"""""

    


        if (shouldAccept) {
            if (leftEstimate.isPresent() && leftEstimate.get().rawFiducials.length > 0) {
                double closestTagDist = Arrays.stream(leftEstimate.get().rawFiducials)
                        .mapToDouble(fiducial -> fiducial.distToCamera)
                        .min()
                        .getAsDouble();
                leftStdDev = xyStdDevCoefficient * Math.pow(closestTagDist, 2) / leftEstimate.get().tagCount;
                leftHeadingStdDev = thetaStdDevCoefficient * Math.pow(closestTagDist, 2) / leftEstimate.get().tagCount;
                if (leftEstimate.get().avgTagDist > 3.5) leftStdDev = Double.MAX_VALUE;
            }
            if (rightEstimate.isPresent() && rightEstimate.get().rawFiducials.length > 0) {
                double closestTagDist = Arrays.stream(rightEstimate.get().rawFiducials)
                        .mapToDouble(fiducial -> fiducial.distToCamera)
                        .min()
                        .getAsDouble();
                rightStdDev = xyStdDevCoefficient * Math.pow(closestTagDist, 2) / rightEstimate.get().tagCount;
                rightHeadingStdDev =
                        thetaStdDevCoefficient * Math.pow(closestTagDist, 2) / rightEstimate.get().tagCount;
                if (rightEstimate.get().avgTagDist > 3.5) rightStdDev = Double.MAX_VALUE;
            }
            if (leftStdDev < rightStdDev) {
                drivetrain.addVisionMeasurement(
                        leftEstimate.get().pose,
                        Utils.fpgaToCurrentTime(leftEstimate.get().timestampSeconds),
                        VecBuilder.fill(leftStdDev, leftStdDev, leftHeadingStdDev));
                robotState.seenReefFaceID((int) LimelightHelpers.getFiducialID(LeftLimelightConstants.CAMERA_NAME));
            } else if (rightStdDev < leftStdDev) {
                drivetrain.addVisionMeasurement(
                        rightEstimate.get().pose,
                        Utils.fpgaToCurrentTime(rightEstimate.get().timestampSeconds),
                        VecBuilder.fill(rightStdDev, rightStdDev, rightHeadingStdDev));
                robotState.seenReefFaceID((int) LimelightHelpers.getFiducialID(RightLimelightConstants.CAMERA_NAME));
            }
        }
    }

    public Optional<PoseEstimate> pollLL(String id, PoseEstimate previousEstimate) {
        LimelightHelpers.SetRobotOrientation(
                id, robotState.getFieldToRobot().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        if (LimelightHelpers.getTV(id)) {
            double oldTimestamp = previousEstimate != null ? previousEstimate.timestampSeconds : Double.MAX_VALUE;
            PoseEstimate newEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(id);
            if (newEstimate != null) {
                Logger.recordOutput(id + " MT2 pose", newEstimate.pose);
                if (newEstimate.timestampSeconds == oldTimestamp) {
                    return Optional.empty(); // no new data
                } else {
                    previousEstimate = newEstimate;
                    return Optional.of(newEstimate);
                }
            }
        }
        return Optional.empty();
    }

    public Optional<PoseEstimate> pollLLMT1(String id, PoseEstimate previousEstimate) {
        if (LimelightHelpers.getTV(id)) {
            double oldTimestamp = previousEstimate != null ? previousEstimate.timestampSeconds : Double.MAX_VALUE;
            PoseEstimate newEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(id);
            if (newEstimate != null) {
                Logger.recordOutput(id + " MT1 pose", newEstimate.pose);
                if (newEstimate.timestampSeconds == oldTimestamp) {
                    return Optional.empty(); // no new data
                } else {
                    previousEstimate = newEstimate;
                    return Optional.of(newEstimate);
                }
            }
        }
        return Optional.empty();
    }
}

"""