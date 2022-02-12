import concurrent.futures
from sensor_controller import *
from readerwriterlock import rwlock
sensor_module = MySensor()
interpretor_module = MyInterpretor()
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
    eSensor = executor.submit(sensor_module.get_grayscale_data, sensor_values_bus, sensor_delay)
    eInterpreter = executor.submit(interpreter_function, sensor_values_bus, interpreter_bus, interpreter_delay)
eSensor.result()
