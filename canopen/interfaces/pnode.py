from canopen.profiles import p402


class PNode(Node402):
    
    def __init__(self, node_id, object_dictionary):
        super(PNode, self).__init__(node_id, object_dictionary)
    
    
    def __del__(self):
        pass
    
    
    
    
    