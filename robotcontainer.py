#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.cmd
from commands2.button import CommandXboxController, Trigger
from wpilib import Joystick
from wpilib import XboxController
from commands2.sysid import SysIdRoutine
from telemetry import Telemetry
from phoenix6 import swerve

from wpilib import DriverStation
from wpilib import SmartDashboard

from generated.tuner_constants import TunerConstants
from subsystems.LLSystem import LLSystem
from subsystems.Drive.driveTrainGenerate import DrivetrainGenerator
from subsystems.Drive.headingcontroller import HeadingController

from Commands.driveTocommands import DriveToPose
from Commands.settargetpoae import TargetePose
from Commands.drivecommand import DriveTeleopCommand
from Auto import autogenerator
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.auto import AutoBuilder




class RobotContainer:

    def __init__(self) -> None:
        self.drivetrain = DrivetrainGenerator.getInstance()
        self.headingController = HeadingController.getInstance()
        self.limelightSytem = LLSystem()
        self._joystick = CommandXboxController(0)

        self.autoGenerator = autogenerator.AutoGenerator()
        self.autoChooser = AutoBuilder.buildAutoChooser("Autos")
        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        
        self._logger = Telemetry(TunerConstants.speed_at_12_volts)

        
        # speed_at_12_volts desired top speed
        self._max_speed = (TunerConstants.speed_at_12_volts)  
        
        self.driveToPose = DriveToPose(
                lambda: TargetePose.getTargetPose(), 
                lambda: self.drivetrain.get_state().pose
        )

        self.driveTeleopCommand = DriveTeleopCommand(self.drivetrain,
                lambda: -self._joystick.getRawAxis(1),
                lambda: -self._joystick.getRawAxis(0),
                lambda: -self._joystick.getRawAxis(4))
    
        self.a :bool

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:


        self.drivetrain.setDefaultCommand(self.driveTeleopCommand)

       
        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        """""
        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )
        """""
        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        
        
        self._joystick.button(1).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateToZero()))
        
        self._joystick.button(2).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateTo90()))

        self._joystick.button(3).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateTo180()))

        self._joystick.button(4).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateTo270()))

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        
        self._joystick.button(7).whileTrue(
            commands2.DeferredCommand(lambda:self.driveToPose.initializePath()).finallyDo
            (self.headingController.setTargetRotationInt))
        



    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()
        return None