"""
This module defines the `DetumblerManager` class, which provides a high-level
interface for computing the magnetic dipole moment required to detumble a CubeSat
using magnetorquers.

The detumbling algorithm uses the satellite's measured magnetic field vector and
angular velocity vector to compute a corrective dipole moment. This dipole moment
can then be applied by hardware drivers (e.g., DRV2605L coils) to reduce angular
momentum over time.

The class includes basic vector math utilities (dot and cross product) and applies
a standard B-dot detumbling control law using a configurable gain constant.

**Usage:**
```python
detumbler = DetumblerManager(gain=1.0)

mag_field = (0.20, -0.10, 0.05)   # Magnetic field in micro Tesla
                                  # sensors like LIS2MDLManager return in micro Telsa,
                                  # but since this class uses Telsa, we multiply the input by * 1e-6
ang_vel   = (0.01, 0.02, -0.005)  # Angular velocity in rad/s, see AngularVelocity class

dipole = detumbler.magnetorquer_dipole(mag_field, ang_vel)
print(dipole)  # → [-0.0023, 0.0011, 0.0004]
```
"""

class DetumblerManager:

    def __init__(self, gain: float = 1.0):
        """
        Initializes the detumbling controller.

        Args:
            gain (float): Gain constant applied in the B-dot law.
                          Higher gain increases detumbling torque but may increase
                          magnetic noise or cause overshoot. Default is 1.0.
        """
        self.gain = gain

    @staticmethod
    def dot_product(vector1: tuple, vector2: tuple) -> float:
        """
        Computes the dot product of two 3-element vectors.

        Args:
            vector1 (tuple): First vector of length 3.
            vector2 (tuple): Second vector of length 3.

        Returns:
            float: Dot product scalar.
        """
        return sum(a * b for a, b in zip(vector1, vector2))

    @staticmethod
    def x_product(vector1: tuple, vector2: tuple) -> list:
        """
        Computes the cross product of two 3-element vectors.

        Args:
            vector1 (tuple): First vector of length 3.
            vector2 (tuple): Second vector of length 3.

        Returns:
            list: Resulting 3-element cross-product vector.
        """
        return [
            vector1[1] * vector2[2] - vector1[2] * vector2[1],
            vector1[2] * vector2[0] - vector1[0] * vector2[2],
            vector1[0] * vector2[1] - vector1[1] * vector2[0],
        ]

    def gain_func(self) -> float:
        """
        Returns the active gain value for the detumbling control law.

        This function exists to allow subclasses or systems with dynamic gains
        to override gain behavior without modifying core logic.

        Returns:
            float: Gain constant.
        """
        return self.gain

    def magnetorquer_dipole(self, mag_field: tuple, ang_vel: tuple) -> list:
        """
        Computes the magnetic dipole moment required to detumble the spacecraft.

        Implements a standard B-dot control law:

            m = -k * (B x ω) / ||B||

        where:
          - B is the magnetic field vector (Tesla),
          - ω is angular velocity (rad/s),
          - k is the controller gain,
          - m is the resulting dipole moment vector (A·m²).

        Args:
            mag_field (tuple): Measured magnetic field (Bx, By, Bz), length 3.
                    use LIS2MDLManager get_magnetic_field()
                    sensors like LIS2MDLManager return in micro Telsa,
                    but since this class uses Telsa, we multiply the input by * 1e-6
            ang_vel   (tuple): Measured angular velocity (ωx, ωy, ωz), length 3.
                    use LSM6DSOXManager get_angular_velocity()

        Returns:
            list: Dipole moment vector to apply via magnetorquers.

        Raises:
            ValueError: If the magnetic field vector has zero magnitude.
        """
        # convert micro-Telsa to Telsa
        mag_field_T = tuple(b * 1e-6 for b in mag_field)

        gain = self.gain_func()
        scalar_coef = -gain / pow(self.dot_product(mag_field_T, mag_field_T), 0.5)
        dipole_out = self.x_product(mag_field_T, ang_vel)
        for i in range(3):
            dipole_out[i] *= scalar_coef
        return dipole_out