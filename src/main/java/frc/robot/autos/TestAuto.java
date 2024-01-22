package frc.robot.autos;

import frc.robot.commands.RotateModulesToAngleCmd;
import frc.robot.commands.WaitForMillisecsCmd;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuto extends SequentialCommandGroup {
    SwerveSubsystem m_swerve;
    private static final long waitDelayMs = 200;
 
    public TestAuto(SwerveSubsystem swerve) {
        m_swerve = swerve;

        addCommands(
            new RotateModulesToAngleCmd(m_swerve, 0.0),
            new WaitForMillisecsCmd(m_swerve, waitDelayMs),
            new RotateModulesToAngleCmd(m_swerve, 45.0),
            new WaitForMillisecsCmd(m_swerve, waitDelayMs),
            new RotateModulesToAngleCmd(m_swerve, 90.0),
            new WaitForMillisecsCmd(m_swerve, waitDelayMs),
            new RotateModulesToAngleCmd(m_swerve, 135.0),
            new WaitForMillisecsCmd(m_swerve, waitDelayMs),
            new RotateModulesToAngleCmd(m_swerve, 180.0),
            new WaitForMillisecsCmd(m_swerve, waitDelayMs),
            new RotateModulesToAngleCmd(m_swerve, 225.0),
            new WaitForMillisecsCmd(m_swerve, waitDelayMs),
            new RotateModulesToAngleCmd(m_swerve, 270.0),
            new WaitForMillisecsCmd(m_swerve, waitDelayMs),
            new RotateModulesToAngleCmd(m_swerve, 315.0),
            new WaitForMillisecsCmd(m_swerve, waitDelayMs),
            new RotateModulesToAngleCmd(m_swerve, 360.0),
            new WaitForMillisecsCmd(m_swerve, waitDelayMs),
            new InstantCommand(()-> m_swerve.stop())
        );
    }
}