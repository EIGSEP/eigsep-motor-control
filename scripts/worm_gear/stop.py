from eigsep_motor_control import PololuMotor

motor = PololuMotor()
motor.stop(motors=["az", "alt"])
