/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {    
    private final SpeedController m_left
        = new SpeedControllerGroup(new WPI_VictorSPX(Constants.leftFrontMotor), new WPI_VictorSPX(Constants.leftBackMotor));

    private final SpeedController m_right
        = new SpeedControllerGroup(new WPI_VictorSPX(Constants.rightFrontMotor), new WPI_VictorSPX(Constants.rightBackmotor));

    private final SpeedController m_elevator
        = new SpeedControllerGroup(new VictorSP(0), new VictorSP(1));

    DifferentialDrive diffDrive;

    DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(Units.inchesToMeters(26.0));

    // DifferentialDriveOdometry odometry =
    //     new DifferentialDriveOdometry();
    
    private final Encoder m_leftEncoder = new Encoder(1, 2);
    private Encoder m_rightEncoder;

    double leftAngle = 0;
    double rightAngle = 0;

    PIDController m_pidLeft = new PIDController(1, 0 ,0);
    PIDController m_pidRight = new PIDController(.3, .05, .05);

    PIDController gyroPIDController = new PIDController(1, 0, 0);
    
    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain() {
        //m_leftEncoder.setDistancePerPulse(distancePerPulse);
        m_rightEncoder = new Encoder(3, 4);

        m_pidLeft.setTolerance(0);
        m_pidRight.setTolerance(0);

        m_pidRight.setSetpoint(20);

        System.out.println("DriveTrain constructor");

        diffDrive = new DifferentialDrive(m_left, m_right);
        diffDrive.setDeadband(.05);
        diffDrive.setMaxOutput(Constants.speedCap);
    }

    public void PIDdrive(double leftDistance, double rightDistance) {
        //m_left.set(m_pidLeft.calculate(leftAngle, 0) / 10);
        double rightEncoderAngle = m_rightEncoder.getDistance();
        double rightOutput = m_pidRight.calculate(rightEncoderAngle) / 20;
        System.out.println(rightEncoderAngle);
        m_right.set(rightOutput);
    }

    public void drive(double Speed, double Rotation) {
        diffDrive.arcadeDrive(Speed, Rotation);
    }

    public void elevator(double speed) {
        m_elevator.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
