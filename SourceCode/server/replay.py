class ReplayManager:

    @classmethod
    def set_output(cls, path):
        cls.file = open(path, "w")

    @classmethod
    def set_input(cls, path):
        cls.file = open(path, "r")
        cls.lines = list(cls.file.readlines())
        cls.line_index = 0

    @classmethod
    def write_line(cls, string):
        assert "\n" not in string, "Replay stores should be single lines."
        cls.file.write(string + "\n")

    @classmethod
    def read_line(cls) -> str:
        line = cls.lines[cls.line_index]
        cls.line_index += 1
        return line

    @classmethod
    def close(cls):
        cls.file.close()

class ErrorManager:
    
    @classmethod
    def write_error(cls, path, error_string):
        cls.file = open(path, "w")
        cls.file.write(error_string)
