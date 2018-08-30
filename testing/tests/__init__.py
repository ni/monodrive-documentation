import json

class TestResult:
    def __init__(self, success, result=None, errors=[]):
        self.success = success
        self.result = result
        self.errors = errors if isinstance(errors, list) else [errors]

    def __str__(self):
        return json.dumps({
            'success': self.success,
            'result': self.result,
            'errors': [str(e) for e in self.errors]
        })

class BaseTest(object):
    def __init__(self, id, env):
        self.id = id
        self.env = env
        self.result = None

    def before_test(self):
        pass

    def test(self):
        raise NotImplementedError

    def after_test(self):
        pass

    def run(self):
        try:
            self.before_test()
        except Exception as e:
            self.result = TestResult(False, 'unhandled exception when setting up test', e)

        try:
            self.test()
        except Exception as e:
            self.result = TestResult(False, 'unhandled exception when running test', e)

        try:
            self.after_test()
        except Exception as e:
            self.result = TestResult(False, 'unhandled exception after test', [e, self.result])


from .basic import *
from .licensing import *