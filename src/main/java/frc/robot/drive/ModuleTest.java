package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ModuleTest extends SubsystemBase {
    private SwerveModule module;
    private SwerveDriveKinematics kinematics;

    public ModuleTest(SwerveModule module, Translation2d position) {
        this.module = module;
        this.kinematics = Constants.kSwerveKinematics;
    }

    public void stop() {
        module.setDesiredState(new SwerveModuleState(), true);
    }

    public void goToAngle(double angle) {
        module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)), true);
    }

    public void driveSpinny(double output) {
        module.testDriveSpinny(output);
    }

    public void turnSpinny(double output) {
        module.testTurnSpinny(output);
    }

    public void spinnyStop() {
        module.testStopSpinny();
    }

    public void goForward() {
        setOpenLoop(new ChassisSpeeds(1, 0, 0));
    }

    public void goLeft() {
        setOpenLoop(new ChassisSpeeds(0, 1, 0));
    }

    public void strafeForwardLeft() {
        setOpenLoop(new ChassisSpeeds(Math.sqrt(2) / 2, Math.sqrt(2) / 2, 0));
    }

    public void setOpenLoop(ChassisSpeeds speeds) {
        set(speeds, true);
    }

    public void setClosedLoop(ChassisSpeeds speeds) {
        set(speeds, false);
    }

    public void setVolts(double voltage) {
        module.setDriveVoltage(voltage);
    }

    public double getModuleVelocityMPS() {
        return module.getVelocityMPS();
    }

    public void setMPS(double mps) {
        module.setDesiredState(new SwerveModuleState(mps, module.getAngleRotation2d()), false);
    }

    private void set(ChassisSpeeds speeds, boolean isOpenLoop) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        if (states.length != 4) throw new IllegalStateException("there should be only one state");

        module.setDesiredState(states[0], isOpenLoop);
    }
}
