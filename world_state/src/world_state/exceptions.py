class ObjectMasterException(Exception):
    """Exceptions realted to the object master. """
    def __init__(self, tp, additional_info=None):
        self.types={}
        self.types["UNKNOWN_CAT"]="Object class not known."
        self.types["UNKNOWN_INSTANCE"]="Object instance not known."

        self.tp = tp
        self.additional_info=additional_info
        
    def __str__(self):
        if self.additional_info:
            return '\n\n' + str(self.types[self.tp] + '\n'+str(self.additional_info))
        else:
            return '\n\n' +repr(self.types[self.tp] )
        
    
    
class StateException(Exception):
    """Exceptions realted to the world state. """
    def __init__(self, tp, additional_info=None):
        self.types={}
        self.types["NO_POSE"]="Object has no pose."
        self.types["NO_OBSERVATION"]="Observation has not got this topic."

        self.tp = tp
        self.additional_info=additional_info
        
    def __str__(self):
        if self.additional_info:
            return '\n\n' + str(self.types[self.tp] + '\n'+str(self.additional_info))
        else:
            return '\n\n' +repr(self.types[self.tp] )
        
    
    