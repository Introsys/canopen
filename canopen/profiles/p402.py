# inspired by the NmtMaster code
import logging
import time
from ..node import RemoteNode
from wx._animate import AC_DEFAULT_STYLE
from __builtin__ import isinstance

logger = logging.getLogger(__name__)

# status word 0x6041 bitmask and values in the list in the dictionary value
POWER_STATES = {
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
    # disable_voltage -------------------------------------------------------
    ('READY TO SWITCH ON', 'SWITCH ON DISABLED'):       0x00,  # transition 7
    ('OPERATION ENABLED', 'SWITCH ON DISABLED'):        0x00,  # transition 9
    ('SWITCHED ON', 'SWITCH ON DISABLED'):              0x00,  # transition 10
    ('QUICK STOP ACTIVE', 'SWITCH ON DISABLED'):        0x00,  # transition 12
    # automatic -------------------------------------------------------------
    ('NOT READY TO SWITCH ON', 'SWITCH ON DISABLED'):   0x00,  # transition 1
    ('START', 'NOT READY TO SWITCH ON'):                0x00,  # transition 0
    ('FAULT REACTION ACTIVE', 'FAULT'):                 0x00,  # transition 14
    # shutdown --------------------------------------------------------------
    ('SWITCH ON DISABLED', 'READY TO SWITCH ON'):       0x06,  # transition 2
    ('SWITCHED ON', 'READY TO SWITCH ON'):              0x06,  # transition 6
    ('OPERATION ENABLED', 'READY TO SWITCH ON'):        0x06,  # transition 8
    # switch_on -------------------------------------------------------------
    ('READY TO SWITCH ON', 'SWITCHED ON'):              0x07,  # transition 3
    ('OPERATION ENABLED', 'SWITCHED ON'):               0x07,  # transition 5
    # enable_operation ------------------------------------------------------
    ('SWITCHED ON', 'OPERATION ENABLED'):               0x0F,  # transition 4
    ('QUICK STOP ACTIVE', 'OPERATION ENABLED'):         0x0F,  # transition 16
    # quickstop -------------------------------------------------------------
    ('READY TO SWITCH ON', 'QUICK STOP ACTIVE'):        0x02,  # transition 7
    ('SWITCHED ON', 'QUICK STOP ACTIVE'):               0x02,  # transition 10
    ('OPERATION ENABLED', 'QUICK STOP ACTIVE'):         0x02,  # transition 11
    # fault -----------------------------------------------------------------
    ('FAULT', 'SWITCH ON DISABLED'):            [0x00, 0x80],  # transition 15
}

# Operations sodes
OPERATIONMODE = {    
    'NO MODE'                     : 0,
    'PROFILED POSITION'           : 1,
    'VELOCITY'                    : 2,
    'PROFILED VELOCITY'           : 3,
    'PROFILED TORQUE'             : 4,
    'RESERVED'                    : 5,
    'HOMING'                      : 6,
    'INTERPOLATED POSITION'       : 7,
    'CYCLIC SYNCHRONOUS POSITION' : 8,
    'CYCLIC SYNCHRONOUS VELOCITY' : 9,
    'CYCLIC SYNCHRONOUS TORQUE'   : 10
}

# homing controlword bit maks
HOMING_COMMANDS = {
    'START' : 0x10,
    'HALT'  : 0x100
}

# homing statusword bit masks
HOMING_STATES = {
    'IN PROGRESS'                           : [0x3400, 0x0000],
    'INTERRUPTED'                           : [0x3400, 0x0400],
    'ATTAINED TARGET NOT REACHED'           : [0x3400, 0x1000],
    'SUCCESSFULLY'                          : [0x3400, 0x1400],
    'ERROR OCCURRED VELOCITY IS NOT ZERO'   : [0x3400, 0x2000],
    'ERROR OCCURRED VELOCITY IS ZERO'       : [0x3400, 0x2400]
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
        self.powerstate_402 = PowerStateMachine(self)
        self.powerstate_402.network = self.network
        self.is_statusword_configured = False
        self.is_controlword_configured = False
        
        
        self.mode_handler = ModeHandler(self)
        self.mode_handler.network = self.network
        

    def setup_powerstate_machine(self):
        """Configured the state machine by searching for the PDO that has the
        StatusWord mappend.
        """
        # the node needs to be in pre-operational mode
        self.nmt.state = 'PRE-OPERATIONAL'
        self.pdo.read() # read all the PDOs (TPDOs and RPDOs)
        for key, pdo in self.pdo.items():
            if pdo.enabled:
                if not self.is_statusword_configured:
                    try:
                        # try to access the object, raise exception if does't exist
                        pdo["Statusword"]
                        pdo.add_callback(self.powerstate_402.on_powerstate_callback)
                        pdo.add_callback(self.mode_handler.on_statusword_callback)
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




    def reset_from_fault(self):
        pass

    def homing(self):
        pass

    def change_mode(self, mode):
        pass




class PowerStateMachine(object):
    """A CANopen CiA 402 Power State machine. Listens to state changes
    of the DS402 Power State machine by means of the Statusword.

    - Controlword 0x6040 causes transitions
    - Statusword 0x6041 gives the current state

    """

    def __init__(self, node):
        self.id = node.id
        self.node = node
        self._state = 'NOT READY TO SWITCH ON'
        self.cw_pdo = None

    def __next_state_for_enabling(self, _from):
        """Returns the next state needed for enabling the motor
        :param string target: Target state
        :return string: Next target to chagne
        """
        for cond, next in NEXTSTATE2ENABLE.items():
            if _from in cond:
                return next

    @staticmethod
    def on_powerstate_callback(mapobject):
        # this function receives a map object.
        # this map object is then used for changing the
        # BaseNode402.PowerstateMachine._state by reading the statusword
        statusword = mapobject[0].raw
        for key, value in POWER_STATES.items():
    		# check if the value after applying the bitmask (value[0])
    		# corresponds with the value[1] to determine the current status
            bitmaskvalue = statusword & value[0]
            if bitmaskvalue == value[1]:
                mapobject.pdo_node.node.powerstate_402._state = key

    @property
    def state(self):
        """Attribute to get or set node's state as a string.

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
        if self._state in POWER_STATES.values():
            return POWER_STATES[self._state]
        else:
            return self._state

    @state.setter
    def state(self, new_state):
        """
        :param string new_state: Holds the target state
        """
        gt = time.time() + 15
        while self.state != new_state:
            try:
                if new_state == 'OPERATION ENABLED':
                    next = self.__next_state_for_enabling(self.state)
                else:
                    next = new_state
                code = TRANSITIONTABLE[ (self.state, next) ]
                if isinstance(code, list):
                    for n in code:
                        self.node.sdo[0x6040].raw = n
                        time.sleep(0.001)
                else:
                    self.node.sdo[0x6040].raw = code
                it = time.time() + 1  # wait one second
                while self.state != next:
                    if time.time() > it:
                        raise RuntimeError('Timeout when trying to change state')
                    time.sleep(0.0001)
            except RuntimeError as e:
                print(e)
            except KeyError:
                raise('Illegal transaction from {f} to {t}'.format(f=self.state, t=_to))
            finally:
                if time.time() > gt:
                    raise RuntimeError('Timeout when trying to change state')




class ModeHandler(object):
    
    def __init__(self, node):
        self.id = node.id
        self.node = node
        self.cw_pdo = None
        self._state = ''


    @staticmethod
    def on_statusword_callback(mapobject):
        # this function receives a map object.
        # this map object is then used for changing the
        # BaseNode402.PowerstateMachine._state by reading the statusword
        statusword = mapobject[0].raw
        
        print 'STATUSWORD: {0}'.format(statusword)

    @property
    def state(self):
        return self._state


    @state.setter
    def state(self, new_state):
        self._state = new_state
        print 'State {0}'.format(self._state)

# EOF
