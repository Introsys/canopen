# inspired by the NmtMaster code
import logging
import time
from ..node import RemoteNode
from ..sdo import SdoAbortedError, SdoCommunicationError

logger = logging.getLogger(__name__)


class STATE402:

    # Control word 0x6040 commands
    CW_OPERATION_ENABLED        = 0x0F
    CW_SHUTDOWN                 = 0x06
    CW_SWITCH_ON                = 0x07
    CW_QUICK_STOP               = 0x02
    CW_DISABLE_VOLTAGE          = 0x00
    CW_SWITCH_ON_DISABLED       = 0x80
    
    CW_COMMANDS = {
        'SWITCH ON DISABLED'    : CW_SWITCH_ON_DISABLED, # Reset Fault
        'DISABLE VOLTAGE'       : CW_DISABLE_VOLTAGE,
        'READY TO SWITCH ON'    : CW_SHUTDOWN,
        'SWITCHED ON'           : CW_SWITCH_ON,
        'OPERATION ENABLED'     : CW_OPERATION_ENABLED,
        'QUICK STOP ACTIVE'     : CW_QUICK_STOP
    }

    # Statusword 0x6041 bitmask and values in the list in the dictionary value
    SW_MASK = {
        'NOT READY TO SWITCH ON': [0x4F, 0x00],
        'SWITCH ON DISABLED'    : [0x4F, 0x40],
        'READY TO SWITCH ON'    : [0x6F, 0x21],
        'SWITCHED ON'           : [0x6F, 0x23],
        'OPERATION ENABLED'     : [0x6F, 0x27],
        'FAULT'                 : [0x4F, 0x08],
        'FAULT REACTION ACTIVE' : [0x4F, 0x0F],
        'QUICK STOP ACTIVE'     : [0x6F, 0x07]
    }

    # Transition path to enable the DS402 node
    NEXTSTATE2ENABLE = {
        ('START')                                                   : 'NOT READY TO SWITCH ON',
        ('FAULT', 'NOT READY TO SWITCH ON')                         : 'SWITCH ON DISABLED',
        ('SWITCH ON DISABLED')                                      : 'READY TO SWITCH ON',
        ('READY TO SWITCH ON')                                      : 'SWITCHED ON',
        ('SWITCHED ON', 'QUICK_STOP_ACTIVE', 'OPERATION_ENABLED')   : 'OPERATION ENABLED',
        ('FAULT REACTION ACTIVE')                                   : 'FAULT'
    }

    # Tansition table from the DS402 State Machine
    TRANSITIONTABLE = {
        # disable_voltage ---------------------------------------------------------------------
        ('READY TO SWITCH ON', 'SWITCH ON DISABLED'):     CW_DISABLE_VOLTAGE,   # transition 7
        ('OPERATION ENABLED', 'SWITCH ON DISABLED'):      CW_DISABLE_VOLTAGE,   # transition 9
        ('SWITCHED ON', 'SWITCH ON DISABLED'):            CW_DISABLE_VOLTAGE,   # transition 10
        ('QUICK STOP ACTIVE', 'SWITCH ON DISABLED'):      CW_DISABLE_VOLTAGE,   # transition 12
        # automatic ---------------------------------------------------------------------------
        ('NOT READY TO SWITCH ON', 'SWITCH ON DISABLED'): 0x00,                 # transition 1
        ('START', 'NOT READY TO SWITCH ON'):              0x00,                 # transition 0
        ('FAULT REACTION ACTIVE', 'FAULT'):               0x00,                 # transition 14
        # shutdown ----------------------------------------------------------------------------
        ('SWITCH ON DISABLED', 'READY TO SWITCH ON'):     CW_SHUTDOWN,          # transition 2
        ('SWITCHED ON', 'READY TO SWITCH ON'):            CW_SHUTDOWN,          # transition 6
        ('OPERATION ENABLED', 'READY TO SWITCH ON'):      CW_SHUTDOWN,          # transition 8
        # switch_on ---------------------------------------------------------------------------
        ('READY TO SWITCH ON', 'SWITCHED ON'):            CW_SWITCH_ON,         # transition 3
        ('OPERATION ENABLED', 'SWITCHED ON'):             CW_SWITCH_ON,         # transition 5
        # enable_operation --------------------------------------------------------------------
        ('SWITCHED ON', 'OPERATION ENABLED'):             CW_OPERATION_ENABLED,  # transition 4
        ('QUICK STOP ACTIVE', 'OPERATION ENABLED'):       CW_OPERATION_ENABLED,  # transition 16
        # quickstop ---------------------------------------------------------------------------
        ('READY TO SWITCH ON', 'QUICK STOP ACTIVE'):      CW_QUICK_STOP,        # transition 7
        ('SWITCHED ON', 'QUICK STOP ACTIVE'):             CW_QUICK_STOP,        # transition 10
        ('OPERATION ENABLED', 'QUICK STOP ACTIVE'):       CW_QUICK_STOP,        # transition 11
        # fault -------------------------------------------------------------------------------
        ('FAULT', 'SWITCH ON DISABLED'):                  CW_SWITCH_ON_DISABLED,       # transition 15
    }


class OPERATIONMODE:
    PROFILED_POSITION = 1
    VELOCITY = 2
    PROFILED_VELOCITY = 3
    PROFILED_TORQUE = 4
    HOMING = 6
    INTERPOLATED_POSITION = 7
    CYCLIC_SYNCHRONOUS_POSITION = 8
    CYCLIC_SYNCHRONOUS_VELOCITY = 9
    CYCLIC_SYNCHRONOUS_TORQUE = 10
    OPEN_LOOP_SCALAR_MODE = -1
    OPEN_LOOP_VECTOR_MODE = -2

    NAME = {
        PROFILED_POSITION           : 'PROFILED POSITION',
        VELOCITY                    : 'VELOCITY',
        PROFILED_VELOCITY           : 'PROFILED VELOCITY',
        PROFILED_TORQUE             : 'PROFILED TORQUE',
        HOMING                      : 'HOMING',
        INTERPOLATED_POSITION       : 'INTERPOLATED POSITION',
        CYCLIC_SYNCHRONOUS_POSITION : 'CYCLIC SYNCHRONOUS POSITION',
        CYCLIC_SYNCHRONOUS_VELOCITY : 'CYCLIC SYNCHRONOUS VELOCITY',
        CYCLIC_SYNCHRONOUS_TORQUE   : 'CYCLIC SYNCHRONOUS TORQUE',
        OPEN_LOOP_SCALAR_MODE       : 'OPEN LOOP SCALAR MODE',
        OPEN_LOOP_VECTOR_MODE       : 'OPEN LOOP VECTOR MODE',
    }


class HOMING:

    CW_START = 0x10
    CW_HALT = 0x100

    HM_ON_POSITIVE_FOLLOWING_ERROR = -8
    HM_ON_NEGATIVE_FOLLOWING_ERROR = -7
    HM_ON_POSITIVE_FOLLOWING_AND_INDEX_PULSE = -6
    HM_ON_NEGATIVE_FOLLOWING_AND_INDEX_PULSE = -5
    HM_ON_THE_POSITIVE_MECHANICAL_LIMIT = -4
    HM_ON_THE_NEGATIVE_MECHANICAL_LIMIT = -3
    HM_ON_THE_POSITIVE_MECHANICAL_LIMIT_AND_INDEX_PULSE = -2
    HM_ON_THE_NEGATIVE_MECHANICAL_LIMIT_AND_INDEX_PULSE = -1
    HM_NO_HOMING_OPERATION = 0
    HM_ON_THE_NEGATIVE_LIMIT_SWITCH_AND_INDEX_PULSE = 1
    HM_ON_THE_POSITIVE_LIMIT_SWITCH_AND_INDEX_PULSE = 2
    HM_ON_THE_POSITIVE_HOME_SWITCH_AND_INDEX_PULSE = [3, 4]
    HM_ON_THE_NEGATIVE_HOME_SWITCH_AND_INDEX_PULSE = [5, 6]
    HM_ON_THE_NEGATIVE_LIMIT_SWITCH = 17
    HM_ON_THE_POSITIVE_LIMIT_SWITCH = 18
    HM_ON_THE_POSITIVE_HOME_SWITCH = [19, 20]
    HM_ON_THE_NEGATIVE_HOME_SWITCH = [21, 22]
    HM_ON_NEGATIVE_INDEX_PULSE = 33
    HM_ON_POSITIVE_INDEX_PULSE = 34
    HM_ON_CURRENT_POSITION = 35

    STATES = {
    'IN PROGRESS'                  : [0x3400, 0x0000],
    'INTERRUPTED'                  : [0x3400, 0x0400],
    'ATTAINED'                     : [0x3400, 0x1000],
    'TARGET REACHED'               : [0x3400, 0x1400],
    'ERROR VELOCITY IS NOT ZERO'   : [0x3400, 0x2000],
    'ERROR VELOCITY IS ZERO'       : [0x3400, 0x2400]
    }


class BaseNode402(RemoteNode):
    """A CANopen CiA 402 profile slave node.

    :param int node_id:
        Node ID (set to None or 0 if specified by object dictionary)
    :param object_dictionary:
        Object dictionary as either a path to a file, an ``ObjectDictionary``
        or a file like object.
    :type object_dictionary: :class:`str`, :class:`canopen.ObjectDictionary`
    """

    def __init__(self, node_id, object_dictionary):
        super(BaseNode402, self).__init__(node_id, object_dictionary)

        self.is_statusword_configured = False
        self.is_controlword_configured = False

        self._state = 'NOT READY TO SWITCH ON'
        self.cw_pdo = None
        self.sw_last_value = None

    def setup_state402_machine(self):
        """Configured the state machine by searching for the PDO that has the
        StatusWord mappend.
        """
        # the node needs to be in pre-operational mode
        self.nmt.state = 'PRE-OPERATIONAL'
        self.pdo.read()  # read all the PDOs (TPDOs and RPDOs)
        for key, pdo in self.pdo.items():
            if pdo.enabled:
                if not self.is_statusword_configured:
                    try:
                        # try to access the object, raise exception if does't exist
                        pdo["Statusword"]
                        pdo.add_callback(self.on_statusword_callback)
                        # make sure only one statusword listner is configured by node
                        self.is_statusword_configured = True
                    except KeyError:
                        pass
                if not self.is_controlword_configured:
                    try:
                        # try to access the object, raise exception if does't exist
                        pdo["Controlword"]
                        self.cw_pdo = pdo
                        # make sure only one controlword is configured in the node
                        self.is_controlword_configured = True
                    except KeyError:
                        pass
        if not self.is_controlword_configured:
            logger.info('Controlword not configured in the PDOs of this node, using SDOs to set Controlword')
        else:
            logger.info('Control word configured in RPDO[{id}]'.format(id=key))
        if not self.is_statusword_configured:
            raise ValueError('Statusword not configured in this node. Unable to access node status.')
        else:
            logger.info('Statusword configured in TPDO[{id}]'.format(id=key))
        self.nmt.state = 'OPERATIONAL'
        self.state = 'SWITCH ON DISABLED'

    def reset_from_fault(self):
        """Reset node from fault and set it to Operation Enable state
        """

        if self.state == 'FAULT':
            self.state = 'OPERATION ENABLED'
        else:
            logger.info('The node its not at fault. Doing nothing!')

    def homing(self, timeout=30, set_new_home=True):
        """Function to execute the configured Homing Method on the node
        :param int timeout: Timeout value (default: 30)
        :param bool set_new_home: Difines if the node should set the home offset
        object (0x607C) to the current position after the homing procedure (default: true)
        """
        result = False
        previus_opm = self.get_op_mode()
        self.state = 'SWITCHED ON'
        self.set_op_mode(OPERATIONMODE.HOMING)
        # The homing process will initialize at operation enabled
        self.state = 'OPERATION ENABLED'
        homingstatus = 'IN PROGRESS'
        self.set_controlword(STATE402.CW_OPERATION_ENABLED | HOMING.CW_START)
        t = time.time() + timeout
        try:
            while homingstatus not in ('TARGET REACHED', 'ATTAINED'):
                for key, value in HOMING.STATES.items():
                    # check if the value after applying the bitmask (value[0])
                    # corresponds with the value[1] to determine the current status
                    bitmaskvalue = self.sw_last_value & value[0]
                    if bitmaskvalue == value[1]:
                        homingstatus = key
                if homingstatus in ('INTERRUPTED', 'ERROR VELOCITY IS NOT ZERO', 'ERROR VELOCITY IS ZERO'):
                   raise  RuntimeError ('Unable to home. Reason: {0}'.format(homingstatus))
                time.sleep(0.001)
                if time.time() > t:
                    raise RuntimeError('Unable to home, timeout reached')
            if set_new_home:
                offset = self.sdo[0x6063].raw
                self.sdo[0x607C].raw = offset
                logger.info('Homing offset set to {0}'.format(offset))
            logger.info('Homing mode carried out successfully.')
            result = True
        except RuntimeError as e:
            logger.info(str(e))
        finally:
            self.set_op_mode(previus_opm)
            return result

    def get_op_mode(self):
        """
        :return int: Return the operation mode stored in the object 0x6061 through SDO
        """
        return self.sdo[0x6061].raw

    def set_op_mode(self, mode):
        """
        :param int mode: TODO
        """
        try:
            logger.info('Changing Operation Mode to {0}'.format(OPERATIONMODE.NAME[mode]))
            result = False
            state = self.state
            if self.state == 'OPERATION ENABLED':
                self.state = 'SWITCHED ON'
                # to make sure the drive does not move with a old value in another mode
                # we clean all the target values for the modes
                self.sdo[0x60FF].raw = 0  # target velocity
                self.sdo[0x607A].raw = 0  # target position
                self.sdo[0x6071].raw = 0  # target torque
            # set the operation mode in an agnostic way, accessing the SDO object by ID
            self.sdo[0x6060].raw = mode
            t = time.time() + 0.5  # timeout
            while self.get_op_mode != mode:
                if time.time() > t:
                    logger.info('Timeout setting the new mode of operation at node {0}.'.format(self.id))
                    break
            result = True
        except SdoCommunicationError as e:
            logger.info('{0}'.format(str(e)))
        except Exception as e:
            logger.error('[ERROR SETTING object {0}:{1}]  {2}'.format(subobj.index, subobj.subindex, str(e)))
        finally:
            self.state = state  # set to last known state
            logger.info('Mode of operation of the node {n} is {m}.'.format(n=self.id , m=OPERATIONMODE.NAME[mode]))
        return result

    def __next_state_for_enabling(self, _from):
        """Returns the next state needed for reach the state Operation Enabled
        :param string target: Target state
        :return string: Next target to chagne
        """
        for cond, next in STATE402.NEXTSTATE2ENABLE.items():
            if _from in cond:
                return next

    def set_controlword(self, value):
        """Helper function enabling the node to send the state using PDO or SDO objects
        :param int value: State value to send in the message
        """
        if self.cw_pdo is not None:
            self.pdo['Controlword'].raw = value
            self.cw_pdo.transmit()
        else:
            self.sdo[0x6040].raw = value

    @staticmethod
    def on_statusword_callback(mapobject):
        # this function receives a map object.
        # this map object is then used for changing the
        # BaseNode402.PowerstateMachine._state by reading the statusword
        statusword = mapobject[0].raw
        mapobject.pdo_node.node.sw_last_value = statusword

        for key, value in STATE402.SW_MASK.items():
            # check if the value after applying the bitmask (value[0])
            # corresponds with the value[1] to determine the current status
            bitmaskvalue = statusword & value[0]
            if bitmaskvalue == value[1]:
                mapobject.pdo_node.node._state = key

    @property
    def state(self):
        """Attribute to get or set node's state as a string for the DS402 State Machine.

        States of the node can be one of:

        - 'NOT READY TO SWITCH ON'
        - 'SWITCH ON DISABLED'
        - 'READY TO SWITCH ON'
        - 'SWITCHED ON'
        - 'OPERATION ENABLED'
        - 'FAULT'
        - 'FAULT REACTION ACTIVE'
        - 'QUICK STOP ACTIVE'

        States to switch to can be one of:

        - 'SWITCH ON DISABLED'
        - 'DISABLE VOLTAGE'
        - 'READY TO SWITCH ON'
        - 'SWITCHED ON'
        - 'OPERATION ENABLED'
        - 'QUICK STOP ACTIVE'

        """
        if self._state in STATE402.SW_MASK.values():
            return STATE402.SW_MASK[self._state]
        else:
            return self._state

    @state.setter
    def state(self, new_state, timeout=5):
        """ Defines the state for the DS402 state machine
        :param string new_state: Target state
        :param int timeout:
        :raise RuntimeError: Occurs when the time defined to change the state is reached
        :raise TypeError: Occurs when trying to execute a ilegal transition in the sate machine
        """
        st = time.time() + timeout
        while self.state != new_state:
            try:
                if new_state == 'OPERATION ENABLED':
                    next = self.__next_state_for_enabling(self.state)
                else:
                    next = new_state
                code = STATE402.TRANSITIONTABLE[ (self.state, next) ]
                self.set_controlword(code)
                it = time.time() + 1  # wait one second
                while self.state != next:
                    if time.time() > it:
                        raise RuntimeError('Timeout when trying to change state')
                    time.sleep(0.00001)  # give some time to breathe
            except RuntimeError as e:
                logger.info(str(e))
            except KeyError:
                raise TypeError('Illegal transition from {f} to {t}'.format(f=self.state, t=new_state))
            finally:
                if time.time() > st:
                    raise RuntimeError('Timeout when trying to change state')

