import logging

class MyLogger:
    def __init__(self, logger_name, logger_file_path, format):
        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(logging.DEBUG)

        self.handler = logging.FileHandler(logger_file_path, mode='w')
        self.logger.addHandler(self.handler)

        self.fmt = logging.Formatter(format)
        self.handler.setFormatter(self.fmt)

    def debug(self, msg):
        self.logger.debug(msg)

    def warning(self, msg):
        self.logger.warning(msg)
