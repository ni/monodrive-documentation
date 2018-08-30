from . import BaseTest

class LicenseTest(BaseTest):
    def __init__(self):
        super(LicenseTest, self).__init__('licensing')

    def run(self):
        self.test_missing_license()
        self.test_bad_license()
        self.test_good_license()

    def test_missing_license(self):
        pass

    def test_bad_license(self):
        pass

    def test_good_license(self):
        pass
