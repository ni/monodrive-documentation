
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import time
import json
import uuid
import requests
import threading
from copy import deepcopy
from functools import wraps

from monodrive import __version__


class NetworkController(object):
    def __init__(self, manager=None):
        self.success_callback = None
        self.failure_callback = None
        if manager is None:
            raise AttributeError('manager required')
        self.manager = manager

    def _post(self, url, json_data):
        try:
            req = requests.post(url, json=json_data, timeout=60,
                                headers={
                                    'Authorization': 'Token {0}'.format(self.manager.settings.get_setting('auth_token', None))})
            if req.status_code == 200 and self.success_callback is not None:
                self.success_callback(req.json())
            elif self.failure_callback is not None:
                self.failure_callback(req.json())
        except requests.ConnectTimeout:
            self.failure_callback({'error': 'timeout'})

    def _get(self, url, json_data):
        try:
            req = requests.get(url, params=json_data, timeout=10,
                               headers={
                                   'Authorization': 'Token {0}'.format(self.manager.settings.get_setting('auth_token', None))
                               })
            print(req.json())
            if req.status_code == 200 and self.success_callback is not None:
                self.success_callback(req.json())
            elif self.failure_callback is not None:
                self.failure_callback(req.json())
        except requests.ConnectTimeout:
            self.failure_callback({'error': 'timeout'})

    def request(self, url, json_data={}, method='GET', success_callback=None, failure_callback=None):
        self.success_callback = success_callback
        self.failure_callback = failure_callback
        self.request_thread = threading.Thread(target=self._post if method == 'POST' else self._get,
                                               args=(url, json_data))
        self.request_thread.setDaemon(True)
        self.request_thread.start()

    @staticmethod
    def _event_collector(json_data):
        r = requests.post('http://splunk.monodrive.io:8088/services/collector/event', json=json_data,
                          headers={'Authorization': 'Splunk 15a831f4-826a-439c-8e32-9d595db394da'})
        if r.status_code != 200:
            print(r.text)

    @staticmethod
    def event_collector(json_data):
        if isinstance(json_data, (list, tuple)):
            json_data = {
                'payload': json_data
            }
        json_data["client_meta"] = {
            'version': str(__version__),
            'session_id': "not in json",
            'client_id': "not in json",
        }
        event = {
            "event": json_data,
            "time": time.time(),
            "sourcetype": "_json",
            "index": "main",
        }
        request_thread = threading.Thread(target=NetworkController._event_collector,
                                          args=(event,))
        request_thread.setDaemon(True)
        request_thread.start()


def monodrive_list_forwarder(fieldnames=None):
    def _forwarder(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            _fieldnames = kwargs.pop('fieldnames', None)
            result = func(*args, **kwargs)
            _result = deepcopy(result)
            if isinstance(_result, (list, tuple)):
                if fieldnames is not None:
                    _result = {k: v for k, v in zip(fieldnames, _result)}
                else:
                    raise ValueError('List/Tuple forwarded data requires fieldnames kwarg.')
            try:
                NetworkController.event_collector(_result)
            except TypeError as e:
                print(str(e))
                print("Failed to JSON parse forwarded response.")
            return result

        return wrapper

    return _forwarder


def monodrive_forwarder():
    def _forwarder(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            result = func(*args, **kwargs)
            _result = deepcopy(result)
            try:
                NetworkController.event_collector(_result)
            except TypeError as e:
                print(str(e))
                print("Failed to JSON parse forwarded response.")
            return result

        return wrapper

    return _forwarder
