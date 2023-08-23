#!/usr/bin/env python


class NodeStatus(object):
    '''
        For enumerating node statuses. 枚举
    '''
    SUCCESS = 0
    RUNNING = 1
    FAILURE = 2


class Node(object):
    '''
        The base Node.
    '''

    def __init__(self, name, *args, **kwargs):
        super(Node, self).__init__(*args, **kwargs)
        self.name = name
        self.type = None

        # self.status has default value None, but should be assigned by self.run() before use it.
        self.status = None
        return

    def run(self):
        pass

    def reset(self):
        self.status = None
        return


class LeafNode(Node):
    def __init__(self, name, *args, **kwargs):
        super(LeafNode, self).__init__(name, *args, **kwargs)
        self.type = "Leaf"
        return


class InternalNode(Node):
    def __init__(self, name, children, *args, **kwargs):
        super(InternalNode, self).__init__(name, *args, **kwargs)
        self.type = "Internal"

        self.children = children
        return

    def reset(self):
        super(InternalNode, self).reset()
        for c in self.children:
            c.reset()
        return


class InternalNode_withMemory(InternalNode): # ？？
    def __init__(self, name, children, *args, **kwargs):
        super(InternalNode_withMemory, self).__init__(name, children, *args, **kwargs)
        self.type = "Internal_M"

        self.current = 0
        return

    def reset(self):
        super(InternalNode_withMemory, self).reset()
        self.current = 0
        return


class DecoratorNode(Node):
    def __init__(self, name, child, *args, **kwargs):
        super(DecoratorNode, self).__init__(name, *args, **kwargs)
        self.type = "Decorator"

        self.child = child
        return

    def reset(self):
        super(DecoratorNode, self).reset()
        self.child.reset()
        return


class Selector(InternalNode):
    def __init__(self, name, children, *args, **kwargs):
        super(Selector, self).__init__(name, children, *args, **kwargs)
        self.type = "Selector"
        return

    def run(self):
        for c in self.children:
            c.status = c.run()

            if c.status != NodeStatus.FAILURE:
                return c.status
        return NodeStatus.FAILURE


class Sequence(InternalNode):
    def __init__(self, name, children, *args, **kwargs):
        super(Sequence, self).__init__(name, children, *args, **kwargs)
        self.type = "Sequence"
        return

    def run(self):
        for c in self.children:
            c.status = c.run()

            if c.status != NodeStatus.SUCCESS:
                return c.status
        return NodeStatus.SUCCESS


class Selector_withMemory(InternalNode_withMemory): # ？？
    def __init__(self, name, children, *args, **kwargs):
        super(Selector_withMemory, self).__init__(name, children, *args, **kwargs)
        self.type = "Selector_M"
        return

    def run(self):
        for index in range(self.current, len(self.children)):
            self.children[index].status = self.children[index].run()

            if self.children[index].status != NodeStatus.FAILURE:
                self.current = index
                return self.children[index].status
        self.current = 0
        return NodeStatus.FAILURE


class Sequence_withMemory(InternalNode_withMemory):
    def __init__(self, name, children, *args, **kwargs):
        super(Sequence_withMemory, self).__init__(name, children, *args, **kwargs)
        self.type = "Sequence_M"
        return

    def run(self):
        for index in range(self.current, len(self.children)):
            self.children[index].status = self.children[index].run()

            if self.children[index].status != NodeStatus.SUCCESS:
                self.current = index
                return self.children[index].status
        self.current = 0
        return NodeStatus.SUCCESS


class DecoratorNode_NOT(DecoratorNode): # 改变状态
    '''
        Change NodeStatus.SUCCESS to NodeStatus.FAILURE, and vice versa.
    '''

    def __init__(self, name, child, *args, **kwargs):
        super(DecoratorNode_NOT, self).__init__(name, child, *args, **kwargs)
        self.type = "Decorator_NOT"
        return

    def run(self):
        self.child.status = self.child.run()

        if self.child.status == NodeStatus.SUCCESS:
            return NodeStatus.FAILURE
        if self.child.status == NodeStatus.RUNNING:
            return NodeStatus.RUNNING
        if self.child.status == NodeStatus.FAILURE:
            return NodeStatus.SUCCESS
        return None


