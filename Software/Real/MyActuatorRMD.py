"""
Class representing MyActuatorRMD equipment, which includes various actuators.

Dependencies:
- Real: Module containing protocols for different actuator series.
  Install dependencies as per the respective protocol requirements.

Usage:
1. Import the required protocol modules from the Real package.
2. Create instances of different actuator series, such as X, L, or S, by providing the necessary parameters.
3. Access specific functionality or properties of each actuator series as needed.
"""

# Import the required protocol modules from the Real package
#from Real import ProtocolSSeries
from Real.ProtocolLSeries import ProtocolLSeries
#from Real import ProtocolXSeriesV2
from Real.ProtocolXSeriesV3 import ProtocolXSeriesV3

class MyActuatorRMD:

    class X:
        """
        Class representing the X series of actuators from MyActuatorRMD.
        """

        class V2:
            """
            Class representing the V2 version of the X series actuators.
            """
            def __init__(self) -> None:
                pass

        class V3(ProtocolXSeriesV3):
            """
            Class representing the V3 version of the X series actuators, inheriting from ProtocolXSeriesV3.

            Attributes:
                id (int): Identifier for the actuator.
                reducer_ratio (int): Reducer ratio for the actuator.
                can_bus: CAN bus instance for communication.
            """

            id: int
            reducer_ratio: int

            def __init__(self, id: int, reducer_ratio: int, can_bus):
                """
                Initialize the V3 actuator with the given parameters.

                Parameters:
                    id (int): Identifier for the actuator.
                    reducer_ratio (int): Reducer ratio for the actuator.
                    can_bus: CAN bus instance for communication.
                """
                self.id = id
                self.reducer_ratio = reducer_ratio
                self.can_bus = can_bus
                super().__init__(can_bus)

    class L(ProtocolLSeries):
        """
        Class representing the L series of actuators from MyActuatorRMD, inheriting from ProtocolLSeries.
        """

        def __init__(self, id: int, reducer_ratio: int, can_bus):
            """
            Initialize the L series actuator with the given parameters.

            Parameters:
                id (int): Identifier for the actuator.
                reducer_ratio (int): Reducer ratio for the actuator.
                can_bus: CAN bus instance for communication.
            """
            super().__init__(id, reducer_ratio, can_bus)

    class S:
        """
        Class representing the S series of actuators from MyActuatorRMD.
        """
        def __init__(self) -> None:
            pass
