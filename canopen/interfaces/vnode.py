from canopen.profiles import p402


class VNode(Node402):
    
    def __init__(self, node_id, object_dictionary):
        super(VNode, self).__init__(node_id, object_dictionary)
    
    
    def __del__(self):
        pass
    
    
    
    
    