#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"


import logging
from logging.handlers import RotatingFileHandler

import multiprocessing
try:
    import queue as PythonQueue
except ImportError:
    import Queue as PythonQueue
import threading


class LogFormatter(logging.Formatter):
    def format(self, record):
        record.name = record.name.upper()
        return logging.Formatter.format(self, record)


class LoggerConfig:
    def __init__(self, filename, level, max_bytes, backup_count):
        self.filename = filename
        self.level = level
        self.max_bytes = max_bytes
        self.backup_count = backup_count
        self.logger_settings = {}


class Logger(multiprocessing.Queue):

    def __init__(self, maxsize):
        super(Logger, self).__init__(maxsize)

    @classmethod
    def getLogger(self, config):
        logging.basicConfig(filename=config.filename, level=config.level,
                            format="%(name)-12s %(levelname)-8s: %(message)s")
        formatter = LogFormatter("%(name)-12s %(levelname)-8s: %(message)s")

        file_handler = RotatingFileHandler(config.filename,
                                           maxBytes=config.max_bytes,
                                           backupCount=config.backup_count)
        file_handler.setLevel(config.level)
        logging.getLogger().addHandler(file_handler)

        for category, level in config.logger_settings.items():
            level = logging.getLevelName(level.upper())
            logger = logging.getLogger(category)
            logger.setLevel(level)

            file_handler = RotatingFileHandler(config.filename,
                                               maxBytes=config.max_bytes,
                                               backupCount=config.backup_count)
            file_handler.setLevel(level)
            file_handler.setFormatter(formatter)

            logger.addHandler(file_handler)

        logger = Logger()
        logger.start()
        return logger

    def start(self):
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        self.running = True
        while self.running:
            try:
                message = self.queue.get(timeout=.25)
            except PythonQueue.Empty:
                continue

            logging.getLogger(message['logger']).log(message['level'], message['message'], message['args'])

    def stop(self):
        self.running = False
        self.thread.join()

    def info(self, log, message, *args, **kwargs):
        self.queue.put({
            'logger': log,
            'level': logging.INFO,
            'message': message,
            'args': args
        })

    def debug(self, log, message, *args, **kwargs):
        self.queue.put({
            'logger': log,
            'level': logging.DEBUG,
            'message': message,
            'args': args
        })

    def error(self, log, message, *args, **kwargs):
        self.queue.put({
            'logger': log,
            'level': logging.ERROR,
            'message': message,
            'args': args
        })

    def warn(self, log, message, *args, **kwargs):
        self.queue.put({
            'logger': log,
            'level': logging.WARN,
            'message': message,
            'args': args
        })

    def setLevel(self, settings):
        for category, level in settings.items():
            level = logging.getLevelName(level.upper())
            logger = logging.getLogger(category)
            logger.setLevel(level)