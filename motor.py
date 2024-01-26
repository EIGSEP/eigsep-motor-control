from qwiic_dual_encoder_reader import QwiicDualEncoderReader
from qwiic_scmd import QwiicScmd


class Motor:
    def __init__(self, n_motors=2):
        self.motor = QwiicScmd()
        begin = self.motor.begin()
        assert begin, "Initalization of SCMD failed"
        assert begin, "Initalization of SCMD failed"
        for i in range(n_motors):
            self.motor.set_drive(i, 0, 0)
        self.motor.enable()
 
        self.encoder = QwiicDualEncoderReader()
        self.encoder.begin()


    def start(self, motor_id, speed, distance):
        """
        Start the motor with the given speed.

        Parameters
        ----------
        motor_id : int
            The id of the motor to start.
        speed : int
            The speed to start the motor at. Positive values are forward,
            negative values are backward. Must be between -255 and 255.
        """
        direction = 1 if speed > 0 else 0
        
        distance = abs(distance) if speed > 0 else -1*abs(distance)
        
        velocity = abs(speed)
        
        if distance != float('inf'):
            original = self.get_encoder(motor_id)%32768
            destination = (original-distance)%32768
        else:
            original = self.get_encoder(motor_id)%32768
            destination = distance
        
        overflow = 0
        if original-distance > 32768:
            overflow = 1
        elif original-distance < 0:
            overflow = 1
        else:
            overflow = 0        

        self.motor.set_drive(motor_id, direction, velocity)
        
        
        while overflow == 1:
            if speed < 0:
                if self.get_encoder(motor_id)%32768 < destination:
                    overflow = 0
            elif speed > 0:
                if self.get_encoder(motor_id)%32768 > destination:
                    overflow = 0
            else:
                break

        while(self.get_encoder(motor_id) != destination):

            print(self.get_encoder(motor_id)%32768, destination)
            if speed < 0:
                if self.get_encoder(motor_id)%32768 >= destination:
                    break
            elif speed > 0:
                if self.get_encoder(motor_id)%32768 <= destination:
                    break
            else:
                break
            
        self.stop(motor_id)
                
            

    def stop(self, motor_id):
        self.motor.set_drive(motor_id, 0, 0)

    def get_encoder(self, motor_id):
        if motor_id == 0:
            return self.encoder.count1
        elif motor_id == 1:
            return self.encoder.count2
        else:
            raise KeyError("Invalid motor id.")
