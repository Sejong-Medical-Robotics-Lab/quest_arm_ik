class Parameter:
    class Type:
        DOUBLE, INTEGER, BOOL, STRING = 3, 2, 1, 4
    def __init__(self, name, type_=None, value=None): self.name, self.type_, self.value = name, type_, value
