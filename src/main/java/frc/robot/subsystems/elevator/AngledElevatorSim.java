// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class AngledElevatorSim extends ElevatorSim {

    private boolean m_simulateGravity;
    private double m_minHeight;
    private double m_maxHeight;
    private double m_angleDegrees;

    /**
     * Creates a simulated angled elevator mechanism.
     * <p>
     * This class extends the ElevatorSim class provided by WPILib, but adds functionality
     * so that gravity can be simulated with an angled elevator. 
     * 
     * @param gearbox The type of and number of motors in the elevator gearbox.
     * @param gearing The gearing of the elevator (numbers greater than 1 represent reductions).
     * @param carriageMassKg The mass of the elevator carriage.
     * @param drumRadiusMeters The radius of the drum that the elevator spool is wrapped around.
     * @param minHeightMeters The min allowable height of the elevator.
     * @param maxHeightMeters The max allowable height of the elevator.
     * @param simulateGravity Whether gravity should be simulated or not.
     * @param angleDegrees The angle above the horizontal of the elevator. If the elevator was perfectly vertical, this would be 90.
     */
    public AngledElevatorSim(DCMotor gearbox, double gearing, double carriageMassKg, double drumRadiusMeters,
            double minHeightMeters, double maxHeightMeters, boolean simulateGravity, double angleDegrees) {
        super(gearbox, gearing, carriageMassKg, drumRadiusMeters, minHeightMeters, maxHeightMeters, simulateGravity);
        m_simulateGravity = simulateGravity;
        m_minHeight = minHeightMeters;
        m_maxHeight = maxHeightMeters;
        m_angleDegrees = angleDegrees;
    }

    public void setAngleDegrees(double angleDegrees) {
        m_angleDegrees = angleDegrees;
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        // Calculate updated x-hat from Runge-Kutta.
        var updatedXhat =
            NumericalIntegration.rkdp(
                (x, _u) -> {
                  Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                  if (m_simulateGravity) {
                    double gravAccel = -9.8/Math.sin(Math.toRadians(m_angleDegrees));

                    xdot = xdot.plus(VecBuilder.fill(0, gravAccel));
                  }
                  return xdot;
                },
                currentXhat,
                u,
                dtSeconds);
    
        // We check for collisions after updating x-hat.
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
          return VecBuilder.fill(m_minHeight, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
          return VecBuilder.fill(m_maxHeight, 0);
        }
        return updatedXhat;
      }

}
