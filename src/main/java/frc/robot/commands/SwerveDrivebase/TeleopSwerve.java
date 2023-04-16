package frc.robot.commands.SwerveDrivebase;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * <h3>TeleopSwerve</h3>
 * 
 * Allows the driver to move the robot manually through controller input
 */
public class TeleopSwerve extends CommandBase {

    private final double STICK_DEAD_BAND = 0.1;

    private CommandXboxController m_controller;

    public double m_percentSpeed;

    private SwerveDrive m_drivebase;

    private int m_translationAxis;
    private int m_strafeAxis;
    private int m_rotationAxis;

    private boolean m_openLoop;

    /**
     * <h3>TeleopSwerve</h3>
     * 
     * Allows the driver to move the robot manually through controller input
     * 
     * @param m_drivebase        The swerve drive that moves the robot
     * @param controller      The driver's controller
     * @param translationAxis Controller translation input
     * @param strafeAxis      Controller strafe input
     * @param rotationAxis    Controller rotation input
     * @param fieldRelative   Controlls relative to orientation
     * @param openLoop        Open Loop does not use PID values to correct inputs
     * @param percentSpeed    The speed of the robot from 0.0 to 1.0
     */
    public TeleopSwerve(SwerveDrive m_drivebase, CommandXboxController controller, int translationAxis, int strafeAxis,
            int rotationAxis, boolean openLoop, double percentSpeed) {
        this.m_drivebase = m_drivebase;
        addRequirements(m_drivebase);

        m_controller = controller;
        m_translationAxis = translationAxis;
        m_strafeAxis = strafeAxis;
        m_rotationAxis = rotationAxis;
        m_openLoop = openLoop;
        m_percentSpeed = percentSpeed;

    }

    @Override
    public void execute() {
        double rAxis;
        double m_rotation;

        double yAxis = -m_controller.getHID().getRawAxis(m_translationAxis);
        double xAxis = -m_controller.getHID().getRawAxis(m_strafeAxis);
            rAxis = -m_controller.getHID().getRawAxis(m_rotationAxis);

        // Applies a deadband to the values
        yAxis = MathUtil.applyDeadband(yAxis, STICK_DEAD_BAND);
        xAxis = MathUtil.applyDeadband(xAxis, STICK_DEAD_BAND);
        rAxis = MathUtil.applyDeadband(rAxis, STICK_DEAD_BAND);

        // Reduces the speed of the robot based on m_percentSpeed
        double m_throttle = yAxis * m_percentSpeed;
        double m_strafe = xAxis * m_percentSpeed;
        m_rotation = rAxis * m_percentSpeed;

        m_drivebase.drive(m_throttle, m_strafe, m_rotation, m_openLoop);
    }
}
