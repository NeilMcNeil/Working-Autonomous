package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




import edu.wpi.first.wpilibj.PneumaticsModuleType;
/**
 *
 */
public class PneumaticsSubsystem extends SubsystemBase {

	Compressor compressor = null;

	public PneumaticsSubsystem() {

		// Make a new compressor based on the module
		compressor = new Compressor(PneumaticsModuleType.CTREPCM);

		if (compressor == null) {
			compressor.enableDigital();
		}
	}

	// public void disableCompressor() {
	// 	if (compressor != null) {
	// 		compressor.setClosedLoopControl(false);
	// 	}
	// }

	public void enableCompressor() {
		if (compressor == null) {
			compressor.enableDigital();
		}
	}

	// Periodically update the dashboard and any PIDs or sensors
	@Override
	public void periodic() {

		if (compressor == null) {
			SmartDashboard.putBoolean("Compressor", compressor.isEnabled());
			// SmartDashboard.putBoolean("Compressor Enabled", compressor.enableDigital());
		} else {
			SmartDashboard.putBoolean("Compressor", false);
			SmartDashboard.putBoolean("Compressor Enabled", false);
		}
	}

}